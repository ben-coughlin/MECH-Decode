package com.example.ftcfieldsimulator;

import com.example.ftcfieldsimulator.UdpPositionListener;

import java.util.ArrayList;
import java.util.function.Consumer;

import javafx.application.Platform;

public class RecordingManager {

    public static class RecordedEvent {
        public final long timestamp; // This should ALWAYS be an ABSOLUTE timestamp
        public final com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData messageData;

        public RecordedEvent(long timestamp, com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData messageData) {
            this.timestamp = timestamp;
            this.messageData = messageData;
        }
    }

    public enum PlaybackState { IDLE, RECORDING, PLAYING, PAUSED }
    private long firstEventTimestamp = -1;    // Absolute timestamp of the first event in recordedSession
    private long recordingStartTimeMs = -1; // Absolute timestamp when current recording began (for live duration)
    private volatile PlaybackState currentState = PlaybackState.IDLE;
    private volatile int playbackIndex = 0;
    private ArrayList<RecordedEvent> recordedSession = new ArrayList<>();

//    private long recordingStartTime = 0;
    private Thread playbackThread;
    private final Consumer<com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData> eventConsumer;
    private final Runnable onPlaybackFinished;
    private Consumer<Integer> onProgressUpdateCallback; // For slider updates
    private final Object pauseLock = new Object();
    private final Object stateLock = new Object(); // Lock for critical state changes

    public RecordingManager(Consumer<com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData> eventConsumer, Runnable onPlaybackFinished) {
        this.eventConsumer = eventConsumer;
        this.onPlaybackFinished = onPlaybackFinished;
    }

    public void setOnProgressUpdate(Consumer<Integer> callback) {
        this.onProgressUpdateCallback = callback;
    }

    public ArrayList<RecordedEvent> getRecordedSession() {
        synchronized (stateLock) {
            return new ArrayList<>(recordedSession);
        }
    }

    public void loadRecording(ArrayList<RecordedEvent> newSession) {
        synchronized (stateLock) {
            stopPlaybackInternal();
            recordedSession = newSession != null ? new ArrayList<>(newSession) : new ArrayList<>();
            playbackIndex = 0;
            if (!recordedSession.isEmpty()) {
                firstEventTimestamp = recordedSession.get(0).timestamp; // Get timestamp of the first loaded event
            } else {
                firstEventTimestamp = -1;
            }
            recordingStartTimeMs = -1; // Not actively recording
            currentState = PlaybackState.IDLE;
            System.out.println("Loaded recording with " + recordedSession.size() + " events.");
        }
    }

    /**
     * Gets the time lapsed from the start of the recording to the current event.
     * @return Time in milliseconds, or -1 if no recording or first event timestamp is not set.
     */
    public synchronized long getCurrentEventTimeLapsed() {
        if (currentState == PlaybackState.RECORDING) {
            // This case should ideally be handled by getCurrentRecordingDuration in the App,
            // but as a fallback, return live duration if someone calls this while recording.
            return getCurrentRecordingDuration();
        }
        if (recordedSession.isEmpty() || playbackIndex < 0 || playbackIndex >= recordedSession.size() || firstEventTimestamp < 0) {
            return -1; // Indicate no valid time
        }
//        return recordedSession.get(playbackIndex).timestamp - firstEventTimestamp;
        // Ensure the current event's timestamp is not before the first event's timestamp (shouldn't happen with proper data)
        long currentEventTimestamp = recordedSession.get(playbackIndex).timestamp;
        return Math.max(0, currentEventTimestamp - firstEventTimestamp);
    }

    /**
     * Gets the duration of the current ongoing recording.
     * Relevant only for PlaybackState.RECORDING.
     * @return Time in milliseconds, or -1 if not recording.
     */
    public synchronized long getCurrentRecordingDuration() {
        if (currentState == PlaybackState.RECORDING && recordingStartTimeMs != -1) {
            return System.currentTimeMillis() - recordingStartTimeMs;
        }
        return -1;
    }

    /**
     * Gets the time lapsed from the start of the recording to the event at the given index.
     * @param index The index of the event.
     * @return Time in milliseconds, or -1 if no recording or index is invalid.
     */
    public synchronized long getTimeLapsedForEventIndex(int index) {
        if (recordedSession.isEmpty() || index < 0 || index >= recordedSession.size() || firstEventTimestamp < 0) {
            return -1;
        }
        return recordedSession.get(index).timestamp - firstEventTimestamp;
    }

    public void startRecording() {
        synchronized (stateLock) {
            stopPlaybackInternal();
            if (currentState != PlaybackState.IDLE) {
                System.out.println("Start recording called when not IDLE. Current state: " + currentState + ". Aborting start.");
                currentState = PlaybackState.IDLE; // Ensure it's IDLE
//                return;
            }
            currentState = PlaybackState.RECORDING;
            recordedSession.clear();
            playbackIndex = 0;
//            recordingStartTime = System.currentTimeMillis();
            recordingStartTimeMs = System.currentTimeMillis(); // Capture start time for duration
//            this.firstEventTimestamp = -1; // Reset for new recording
            firstEventTimestamp = -1; // Will be set by the first call to addEvent
//            System.out.println("Recording started.");
            System.out.println("Recording started at: " + recordingStartTimeMs);
        }
    }

    public void addEvent(com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData messageData) {
        synchronized (stateLock) {
            if (currentState != PlaybackState.RECORDING) return;

            long absoluteTimestamp = System.currentTimeMillis(); // Use ABSOLUTE timestamp

//            long timestamp = System.currentTimeMillis() - recordingStartTime;
            if (recordedSession.isEmpty()) {
                firstEventTimestamp = absoluteTimestamp; // This is the first actual event's ABSOLUTE timestamp
                System.out.println("First event timestamp set to: " + firstEventTimestamp);

//                long currentTimestamp = System.currentTimeMillis(); // Or your specific time source
//                firstEventTimestamp = currentTimestamp; // Capture timestamp of the first event
            }
            // Store the ABSOLUTE timestamp in RecordedEvent
            recordedSession.add(new RecordedEvent(absoluteTimestamp, messageData));
//            recordedSession.add(new RecordedEvent(timestamp, messageData));
        }
    }

    public void stopRecording() {
        synchronized (stateLock) {
            if (currentState != PlaybackState.RECORDING) return;
            currentState = PlaybackState.IDLE;
            recordingStartTimeMs = -1; // Reset
            System.out.println("Recording stopped. Total events: " + recordedSession.size());
            if (!recordedSession.isEmpty() && firstEventTimestamp == -1) {
                // Fallback: If somehow no events were added but recording was started and stopped.
                // Or if firstEventTimestamp wasn't set, which shouldn't happen with the logic in addEvent.
                System.err.println("Warning: Recording stopped with events, but firstEventTimestamp was not set.");
            } else if (recordedSession.isEmpty()) {
                firstEventTimestamp = -1; // No events, no first timestamp
            }
        }
    }

    public void play() {
        synchronized (stateLock) {
            if (recordedSession.isEmpty()) {
                System.out.println("Play called but no recording available.");
                return;
            }
//            if (currentState == PlaybackState.PLAYING) {
//                System.out.println("Play called but already playing.");
//                return;
//            }

            // Check if playback is at or past the end of the session.
            // recordedSession.size() - 1 is the last valid index.
            // So, if playbackIndex is >= recordedSession.size(), it's effectively past the end or at an invalid state post-end.
            // Or if playbackIndex is at the very last event (size -1) and we are not PAUSED (meaning it finished).
            if (playbackIndex >= recordedSession.size() ||
                    (playbackIndex == recordedSession.size() - 1 && currentState != PlaybackState.PAUSED && currentState != PlaybackState.PLAYING) ) {

                // If at the end (or past it), and play is called, reset to the beginning.
                System.out.println("Playback was at/past end. Resetting to beginning for replay.");
                playbackIndex = 0; // Reset to start
                // Ensure firstEventTimestamp is still valid, though it should be from load/record.
                if (firstEventTimestamp == -1 && !recordedSession.isEmpty()) {
                    firstEventTimestamp = recordedSession.get(0).timestamp;
                    System.err.println("Warning: firstEventTimestamp was -1 at replay. Reset from first event.");
                }
                // If it was PLAYING and somehow reached here (e.g. external modification of index), stop old thread.
                if (currentState == PlaybackState.PLAYING && playbackThread != null && playbackThread.isAlive()) {
                    stopPlaybackInternal(); // Stop existing playback before restarting
                }
                currentState = PlaybackState.IDLE; // Treat as a fresh play start
            }

            if (firstEventTimestamp == -1 && !recordedSession.isEmpty()) {
                // This is a critical fallback. If a recording exists (e.g. fresh recording)
                // but firstEventTimestamp somehow didn't get set (which it should in addEvent),
                // set it now from the first event in the session.
                firstEventTimestamp = recordedSession.get(0).timestamp;
                System.err.println("Warning: firstEventTimestamp was not set before play. Setting from first event in session: " + firstEventTimestamp);
            }

            // If playbackIndex is out of bounds or at the end, reset to start.
            // Using recordedSession.size() for the upper bound.
            if (playbackIndex < 0 || playbackIndex >= recordedSession.size()) {
                playbackIndex = 0;
            }

            if (currentState == PlaybackState.PAUSED) {
                currentState = PlaybackState.PLAYING;
                synchronized (pauseLock) {
                    pauseLock.notifyAll();
                }
                System.out.println("Playback resumed from index: " + playbackIndex);
                return;
            }

            currentState = PlaybackState.PLAYING;
            if (playbackThread != null && playbackThread.isAlive()) {
                System.err.println("Warning: Old playbackThread was still alive when play() was called. Attempting to stop it.");
                stopPlaybackInternal(); // Ensure the old one is properly cleaned up
            }
            playbackThread = new Thread(this::runPlaybackLoop);
            playbackThread.setDaemon(true);
            playbackThread.start();
            System.out.println("Playback started from index: " + playbackIndex);
        }
//            if (recordedSession.isEmpty() || currentState == PlaybackState.PLAYING) {
//                return;
//            }
//            if (currentState == PlaybackState.PAUSED) {
//                currentState = PlaybackState.PLAYING;
//                synchronized (pauseLock) {
//                    pauseLock.notifyAll();
//                }
//                System.out.println("Playback resumed from index: " + playbackIndex);
//                return;
//            }
//            if (playbackIndex >= recordedSession.size() - (recordedSession.isEmpty() ? 0 : 1)) {
//                playbackIndex = 0;
//            }
//            if (recordedSession.isEmpty()) return;
//
//            currentState = PlaybackState.PLAYING;
//            if (playbackThread != null && playbackThread.isAlive()) {
//                System.err.println("Warning: Old playbackThread was still alive when play() was called. Attempting to stop it.");
//                stopPlaybackInternal();
//            }
//            playbackThread = new Thread(this::runPlaybackLoop);
//            playbackThread.setDaemon(true);
//            playbackThread.start();
//            System.out.println("Playback started from index: " + playbackIndex);
//        }
    }

    public void pause() {
        synchronized (stateLock) {
            if (currentState != PlaybackState.PLAYING) return;
            currentState = PlaybackState.PAUSED;
            System.out.println("Playback paused at index: " + playbackIndex);
        }
    }

    public void seekTo(int eventIndex) { // eventIndex is the raw value from the slider
        synchronized (stateLock) {
            if (recordedSession.isEmpty()) {
                System.out.println("Seek called but no recording.");
                return;
            }

            int targetRawIndex = Math.max(0, Math.min(eventIndex, recordedSession.size() - 1));

            // Find the index to snap to
            int snappedTargetPositionDataIndex = findClosestPositionEventIndex(targetRawIndex);

            if (snappedTargetPositionDataIndex < 0 || snappedTargetPositionDataIndex >= recordedSession.size()) {
                System.out.println("Seek: Could not find a valid PositionData event to snap to for index " + targetRawIndex);
                // Fallback: just go to the raw index or do nothing further.
                // For now, let's proceed with the best guess if findClosestPositionEventIndex had a fallback.
                // If it returned targetRawIndex, we might not get the intermediate drawing behavior.
                // Let's assume findClosestPositionEventIndex returns a valid index in the list.
                if (snappedTargetPositionDataIndex < 0 && !recordedSession.isEmpty()) snappedTargetPositionDataIndex = 0;
                else if (snappedTargetPositionDataIndex >= recordedSession.size() && !recordedSession.isEmpty()) snappedTargetPositionDataIndex = recordedSession.size() -1;
                else if (recordedSession.isEmpty()) return; // Should have been caught earlier
            }

            System.out.println("Seek: Raw index " + targetRawIndex + ", Snapped to PositionData index: " + snappedTargetPositionDataIndex);

            boolean wasPlayingOrPaused = (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED);
            if (wasPlayingOrPaused) {
                stopPlaybackInternal(); // Sets state to IDLE and calls onPlaybackFinished
            }
            currentState = PlaybackState.IDLE; // Ensure state is IDLE

            // Now, similar to stepBackward/stepForward, replay events to get the visual state correct
            // up to snappedTargetPositionDataIndex.

            // Find the PositionData event that occurred *before* our snappedTargetPositionDataIndex.
            int dispatchStartIndex = 0; // Default to start of recording
            for (int i = snappedTargetPositionDataIndex - 1; i >= 0; i--) {
                if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                    dispatchStartIndex = i; // This is the anchor PositionData
                    break;
                }
            }

            // The FtcFieldSimulatorApp might need to be told to clear the screen FIRST
            // if this seek operation is substantial (e.g., jumping far).
            // For now, RecordingManager will just dispatch the events.

            // Dispatch events from the anchor (or start) up to the target index
            for (int i = dispatchStartIndex; i <= snappedTargetPositionDataIndex; i++) {
                playbackIndex = i; // Update playbackIndex for each dispatched event
                dispatchCurrentEvent(); // This will call eventConsumer and onProgressUpdateCallback
            }
            // playbackIndex is now at snappedTargetPositionDataIndex

            System.out.println("Seeked. Current index: " + playbackIndex + " (Snapped from raw: " + targetRawIndex + ")");

            // After dispatching, the slider in the UI should also reflect the snapped index.
            // The onProgressUpdateCallback called by dispatchCurrentEvent() should handle this.
        }
    }

//    // +++ MODIFIED to use dispatchCurrentEvent +++
//    public void seekTo(int eventIndex) {
//        synchronized (stateLock) {
//            if (recordedSession.isEmpty() || eventIndex < 0 || eventIndex >= recordedSession.size()) {
//                System.out.println("Seek out of bounds or no recording. Index: " + eventIndex + ", Size: " + recordedSession.size());
//                return;
//            }
//
//            boolean wasPlayingOrPaused = (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED);
//            if (wasPlayingOrPaused) { // Only stop if it was actually playing/paused
//                stopPlaybackInternal();
//            }
//            // After stopping, state is IDLE.
//
//            this.playbackIndex = eventIndex;
//            dispatchCurrentEvent(); // Use the helper
//
//            currentState = PlaybackState.IDLE;
//            System.out.println("Seeked to event index: " + eventIndex + ". State set to IDLE.");
//        }
//    }

//    // +++ NEW HELPER METHOD +++
//    /**
//     * Helper method to dispatch the current event at playbackIndex to the UI.
//     * Assumes stateLock is held by the caller.
//     */
//    private void dispatchCurrentEvent() {
//        if (playbackIndex >= 0 && playbackIndex < recordedSession.size()) {
//            final RecordedEvent currentEventToDispatch = recordedSession.get(playbackIndex);
//            final int currentIndexToDispatch = playbackIndex;
//            Platform.runLater(() -> {
//                eventConsumer.accept(currentEventToDispatch.messageData);
//                if (onProgressUpdateCallback != null) {
//                    onProgressUpdateCallback.accept(currentIndexToDispatch);
//                }
//            });
//        }
//    }

    // In RecordingManager.java

    /**
     * Steps forward.
     * Finds the next RecordedEvent that contains PositionData.
     * All events from the current playbackIndex up to and including the event BEFORE
     * the found PositionData are dispatched. Then, the PositionData event itself is dispatched.
     * If no further PositionData is found, it steps to the end of the recording, dispatching all events.
     */
    public void stepForward() {
        synchronized (stateLock) {
            if (recordedSession.isEmpty() || playbackIndex >= recordedSession.size() - 1) {
                // Already at or past the last event, or no session
                if (!recordedSession.isEmpty() && playbackIndex < recordedSession.size()){
                    // If simply at the last event, ensure it's dispatched.
                    dispatchCurrentEvent();
                }
                System.out.println("Step forward: Already at end or no recording.");
                return;
            }

            if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
                stopPlaybackInternal(); // Sets state to IDLE and calls onPlaybackFinished
            }
            // Ensure state is IDLE for stepping logic. stopPlaybackInternal should handle this.
            currentState = PlaybackState.IDLE;

            int startIndex = playbackIndex; // The event we are currently at
            int targetIndex = -1;

            // Find the index of the next PositionData event
            for (int i = startIndex + 1; i < recordedSession.size(); i++) {
                if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                    targetIndex = i;
                    break;
                }
            }

            if (targetIndex == -1) {
                // No more PositionData events found, so step to the very last event of the recording
                targetIndex = recordedSession.size() - 1;
            }

            // Dispatch all events from the one AFTER the current playbackIndex
            // up to and including the targetIndex.
            // The FtcFieldSimulatorApp's eventConsumer will handle drawing them.
            for (int i = startIndex + 1; i <= targetIndex; i++) {
                playbackIndex = i; // Update playbackIndex for each dispatched event
                dispatchCurrentEvent(); // This will call eventConsumer and onProgressUpdateCallback
            }

            // playbackIndex is now at targetIndex
            System.out.println("Stepped forward. Current index: " + playbackIndex);
        }
    }

    /**
     * Steps backward.
     * Finds the previous RecordedEvent that contains PositionData.
     * The state is set to that PositionData event by dispatching it. All drawing events
     * that occurred *before* this target PositionData (since the PositionData before *that*)
     * would ideally be re-applied. This is trickier for true "undo" of drawing.
     *
     * SIMPLIFIED APPROACH for now: Step to the previous PositionData and dispatch it.
     * To correctly redraw the state *before* it, the FieldDisplay would need to be
     * cleared and all events from the start of the recording up to the new playbackIndex replayed.
     *
     * For now, let's make it step to the previous PositionData, and dispatch intervening non-PositionData events
     * to ensure the *drawing commands leading up to that PositionData point* are applied.
     */
    public void stepBackward() {
        synchronized (stateLock) {
            if (recordedSession.isEmpty() || playbackIndex <= 0) {
                // Already at or before the first event, or no session
                if (!recordedSession.isEmpty() && playbackIndex < recordedSession.size() && playbackIndex >= 0){
                    // If simply at the first event, ensure it's dispatched.
                    dispatchCurrentEvent();
                }
                System.out.println("Step backward: Already at beginning or no recording.");
                return;
            }

            if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
                stopPlaybackInternal(); // Sets state to IDLE
            }
            currentState = PlaybackState.IDLE;

            int currentIndex = playbackIndex;
            int targetPositionDataIndex = -1;

            // Find the index of the PREVIOUS PositionData event
            for (int i = currentIndex - 1; i >= 0; i--) {
                if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                    targetPositionDataIndex = i;
                    break;
                }
            }

            if (targetPositionDataIndex == -1 && currentIndex > 0) {
                // No prior PositionData, but not at the very start.
                // Step to the very first event.
                targetPositionDataIndex = 0;
            } else if (targetPositionDataIndex == -1 && currentIndex == 0) {
                // Already at the first event, do nothing more than dispatching current.
                dispatchCurrentEvent();
                System.out.println("Step backward: Already at the first event.");
                return;
            }


            // To accurately reflect the visual state at targetPositionDataIndex,
            // we need to ensure all drawing commands *up to that point* are processed.
            // The simplest way to do this with the current dispatchCurrentEvent is to
            // set playbackIndex and dispatch.
            // However, this doesn't "undo" drawings that happened after targetPositionDataIndex.
            //
            // A more robust solution for "stepping back" and seeing the correct visual state
            // often involves the FieldDisplay clearing and redrawing based on all events up to playbackIndex.

            // For now, let's iterate from the start of the recording up to the targetPositionDataIndex,
            // dispatching all events. This ensures the FieldDisplay gets all drawing commands
            // that should have occurred before or at targetPositionDataIndex.
            // This can be slow for very long recordings if not optimized.

            // Alternative: Find the PositionData event *before* our targetPositionDataIndex (if any),
            // then dispatch all events between that and our targetPositionDataIndex.

            // Let's try this:
            // 1. Set playbackIndex to the targetPositionDataIndex.
            // 2. Clear the screen (responsibility of FieldDisplay, triggered by app).
            // 3. Replay events from start up to targetPositionDataIndex (could be done in app).

            // Simpler for RecordingManager: just set the index and dispatch.
            // The app might need more logic if full visual history is required.

            // Let's refine for stepping backward:
            // We want to land on targetPositionDataIndex.
            // All drawing commands that occurred *after* the PositionData *before* targetPositionDataIndex
            // and *before or at* targetPositionDataIndex should be part of the state.

            // Find the PositionData event that occurred *before* our targetPositionDataIndex.
            int previousPositionDataAnchor = -1;
            for (int i = targetPositionDataIndex - 1; i >= 0; i--) {
                if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                    previousPositionDataAnchor = i;
                    break;
                }
            }

            // The events to "replay" to get the correct visual state for targetPositionDataIndex are
            // those from (previousPositionDataAnchor + 1) up to targetPositionDataIndex.
            // The FtcFieldSimulatorApp would need to tell FieldDisplay to clear first.

            // For now, to make `stepBackward` consistent with `stepForward` in dispatching a sequence:
            // Dispatch events from the *start* of the segment leading to `targetPositionDataIndex`.
            // The "segment" starts after the `PositionData` just before our `targetPositionDataIndex`.

            // Let FtcFieldSimulatorApp handle the screen clearing and potential full replay if needed for backward steps.
            // Here, we'll dispatch events from the *previous `playbackIndex` down to the new one,
            // or more simply, just set to the target and dispatch it, and assume FieldDisplay might need a refresh.

            // New approach for stepBackward to show intermediate drawings:
            // Find the target PositionData. Then, to show the state *at* that PositionData,
            // we need all drawing commands that happened *since the PositionData before it*.

            if (targetPositionDataIndex != -1) {
                // To display the state correctly at targetPositionDataIndex,
                // we'll dispatch events starting from the PositionData event *before* it (if any),
                // or from the beginning of the recording, up to targetPositionDataIndex.

                int dispatchStartIndex = 0; // Default to start of recording
                // Find the PositionData immediately preceding targetPositionDataIndex
                for(int i = targetPositionDataIndex - 1; i >= 0; i--) {
                    if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                        dispatchStartIndex = i; // This is the anchor PositionData
                        break;
                    }
                }

                // Clear visual state (App tells FieldDisplay to do this)
                // For now, we assume the app will handle this.
                // Platform.runLater(() -> eventConsumer.accept(new ClearScreenPseudoEvent())); // Example

                // Dispatch events from the anchor (or start) up to the target index
                // Note: The first event dispatched here will be a PositionData (or start of list)
                // The subsequent events will be drawing commands, ending with the targetPositionDataIndex.
                for (int i = dispatchStartIndex; i <= targetPositionDataIndex; i++) {
                    playbackIndex = i;
                    dispatchCurrentEvent(); // This will update UI and call onProgressUpdate
                }
                System.out.println("Stepped backward. Current index: " + playbackIndex);

            } else {
                // This case should be handled by the initial check (playbackIndex <= 0)
                // but as a fallback, if we end up here, just dispatch the current index.
                if (playbackIndex >=0 && playbackIndex < recordedSession.size()){
                    dispatchCurrentEvent();
                }
                System.out.println("Step backward: No suitable previous PositionData found or already at start.");
            }
        }
    }

    // The dispatchCurrentEvent method already calls onProgressUpdateCallback,
    // which in FtcFieldSimulatorApp calls updateTimeLapsedDisplay().
    // It also calls eventConsumer.accept() which should trigger drawing in FieldDisplay.
    private void dispatchCurrentEvent() {
        if (playbackIndex >= 0 && playbackIndex < recordedSession.size()) {
            final RecordedEvent currentEventToDispatch = recordedSession.get(playbackIndex);
            final int currentIndexToDispatch = playbackIndex; // Capture for lambda

            Platform.runLater(() -> {
                eventConsumer.accept(currentEventToDispatch.messageData); // This is where drawing commands go
                if (onProgressUpdateCallback != null) {
                    onProgressUpdateCallback.accept(currentIndexToDispatch); // Updates slider & time
                }
            });
        }
    }


//    // +++ NEW METHODS for stepping +++
//    /**
//     * Steps forward to the next RecordedEvent that contains PositionData.
//     * Updates playbackIndex and dispatches the event.
//     */
//    public void stepForward() {
//        synchronized (stateLock) {
//            if (recordedSession.isEmpty()) return;
//
//            if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
//                stopPlaybackInternal(); // Sets state to IDLE
//            }
//            // Ensure state is IDLE for stepping logic
//            currentState = PlaybackState.IDLE;
//
//            int searchIndex = playbackIndex + 1;
//            while (searchIndex < recordedSession.size()) {
//                if (recordedSession.get(searchIndex).messageData instanceof com.example.ftcfieldsimulator.UdpPositionListener.PositionData) {
//                    playbackIndex = searchIndex;
//                    dispatchCurrentEvent();
//                    System.out.println("Stepped forward to PositionData at index: " + playbackIndex);
//                    return;
//                }
//                searchIndex++;
//            }
//            System.out.println("Step forward: No further PositionData found. Remained at index: " + playbackIndex);
//            // Optional: if no PositionData found, dispatch current index again to ensure UI consistency if needed
//            // dispatchCurrentEvent();
//        }
//    }
//
//    /**
//     * Steps backward to the previous RecordedEvent that contains PositionData.
//     * Updates playbackIndex and dispatches the event.
//     */
//    public void stepBackward() {
//        synchronized (stateLock) {
//            if (recordedSession.isEmpty()) return;
//
//            if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
//                stopPlaybackInternal(); // Sets state to IDLE
//            }
//            currentState = PlaybackState.IDLE;
//
//            int searchIndex = playbackIndex - 1;
//            while (searchIndex >= 0) {
//                if (recordedSession.get(searchIndex).messageData instanceof UdpPositionListener.PositionData) {
//                    playbackIndex = searchIndex;
//                    dispatchCurrentEvent();
//                    System.out.println("Stepped backward to PositionData at index: " + playbackIndex);
//                    return;
//                }
//                searchIndex--;
//            }
//            System.out.println("Step backward: No previous PositionData found. Remained at index: " + playbackIndex);
//            // Optional: if no PositionData found, dispatch current index again
//            // dispatchCurrentEvent();
//        }
//    }

    // +++ MODIFIED to use dispatchCurrentEvent +++
    private void runPlaybackLoop() {
        // Initial event display before loop starts
        // This needs to be done under stateLock to ensure playbackIndex is read consistently
        synchronized(stateLock) {
            if (playbackIndex < recordedSession.size()) {
                dispatchCurrentEvent(); // Use helper
            }
        }

        try {
            while (true) { // Loop condition is now managed by internal state checks and breaks
                synchronized (pauseLock) {
                    while (currentState == PlaybackState.PAUSED) {
                        pauseLock.wait();
                    }
                }

                // Check state before proceeding (must be done under stateLock for visibility)
                synchronized (stateLock) {
                    if (currentState != PlaybackState.PLAYING) {
                        System.out.println("Playback loop exiting due to state change from PLAYING. Current state: " + currentState);
                        break;
                    }

                    if (playbackIndex >= recordedSession.size() - 1) { // Reached end of recording (current is last event)
                        System.out.println("Reached end of recording in playback loop (displaying last event).");
                        break; // Exit loop, finally block will handle cleanup
                    }
                }
                // Calculate delay to the NEXT event (playbackIndex + 1)
                long delayMillis = recordedSession.get(playbackIndex + 1).timestamp - recordedSession.get(playbackIndex).timestamp;
                if (delayMillis < 0) delayMillis = 0; // Guard against negative/zero delays from bad data

                Thread.sleep(delayMillis == 0 ? 1 : delayMillis); // Ensure at least 1ms sleep if timestamps are identical to yield CPU

                synchronized(stateLock) {
                    if (currentState != PlaybackState.PLAYING) {
                        System.out.println("Playback loop: State changed during sleep. Exiting before increment. State: " + currentState);
                        break;
                    }
                    playbackIndex++; // Move to the next event
                    dispatchCurrentEvent(); // Dispatch the new current event
                }
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            System.out.println("Playback thread interrupted. Current state: " + currentState);
        } finally {
            synchronized (stateLock) {
                if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
                    // If the loop finished naturally or was interrupted while "active"
                    currentState = PlaybackState.IDLE;
                    if (onPlaybackFinished != null) {
                        Platform.runLater(onPlaybackFinished);
                    }
                    System.out.println("Playback loop finished or interrupted. Final state set to IDLE. onPlaybackFinished invoked.");
                }
            }
        }
    }

    private void stopPlaybackInternal() {
        PlaybackState stateBeforeStopping = currentState;
        boolean wasPlayingOrPaused = (stateBeforeStopping == PlaybackState.PLAYING || stateBeforeStopping == PlaybackState.PAUSED);
        currentState = PlaybackState.IDLE;

        if (playbackThread != null) {
            if (playbackThread.isAlive()) {
                playbackThread.interrupt();
                try {
                    playbackThread.join(100);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    System.err.println("Interrupted while joining playback thread.");
                }
                if (playbackThread.isAlive()) {
                    System.err.println("Warning: Playback thread did not terminate after interrupt and join timeout.");
                }
            }
            playbackThread = null;
        }

        if (wasPlayingOrPaused && onPlaybackFinished != null) {
            Platform.runLater(onPlaybackFinished);
        }
    }

    public void stopPlayback() {
        synchronized(stateLock) {
            stopPlaybackInternal();
        }
    }

    public PlaybackState getCurrentState() {
        synchronized (stateLock) {
            return currentState;
        }
    }

    public boolean hasRecording() {
        synchronized (stateLock) {
            return !recordedSession.isEmpty();
        }
    }

    public int getPlaybackIndex() {
        synchronized (stateLock) {
            return playbackIndex;
        }
    }

    public int getTotalEvents() {
        synchronized (stateLock) {
            return recordedSession.size();
        }
    }


    /**
     * Finds the index of the nearest PositionData event to a given target index.
     * It can search backward first, then forward, or just pick the absolute closest.
     * For snapping behavior, usually, you'd find the one the targetIndex is closest to,
     * or the one just before/after depending on drag direction (more complex).
     *
     * Let's simplify: find the PositionData event <= targetIndex. If none, find the one >= targetIndex.
     * Or, for a "snap to current segment's PositionData" feel:
     *
     * Given targetIndex, find PositionData_A (index <= targetIndex) and PositionData_B (index > targetIndex).
     * Decide whether to snap to A or B. Often, snap to A if targetIndex is closer to A, else B.
     * Or, more simply, always snap to the PositionData that *starts* the segment targetIndex falls into.
     *
     * Revised simple approach: Given targetIndex,
     * 1. Find the LAST PositionData event at or BEFORE targetIndex.
     * 2. If none, find the FIRST PositionData event at or AFTER targetIndex.
     * 3. If still none, return targetIndex (or -1, or 0).
     *
     * @param targetIndex The approximate index the user is trying to reach.
     * @return The index of the "snapped" PositionData event, or the original targetIndex if no suitable snap point.
     */
    private int findClosestPositionEventIndex(int targetIndex) {
        if (recordedSession.isEmpty()) {
            return targetIndex; // Or -1
        }

        // Ensure targetIndex is within bounds
        targetIndex = Math.max(0, Math.min(targetIndex, recordedSession.size() - 1));

        // Case 1: The targetIndex itself is a PositionData event
        if (recordedSession.get(targetIndex).messageData instanceof UdpPositionListener.PositionData) {
            return targetIndex;
        }

        int prevPosIndex = -1;
        for (int i = targetIndex; i >= 0; i--) {
            if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                prevPosIndex = i;
                break;
            }
        }

        int nextPosIndex = -1;
        for (int i = targetIndex; i < recordedSession.size(); i++) {
            if (recordedSession.get(i).messageData instanceof UdpPositionListener.PositionData) {
                nextPosIndex = i;
                break;
            }
        }

        if (prevPosIndex != -1 && nextPosIndex != -1) {
            // Both found, choose the closer one to the original targetIndex
            if (Math.abs(targetIndex - prevPosIndex) <= Math.abs(targetIndex - nextPosIndex)) {
                return prevPosIndex;
            } else {
                return nextPosIndex;
            }
        } else if (prevPosIndex != -1) {
            return prevPosIndex; // Only previous found
        } else if (nextPosIndex != -1) {
            return nextPosIndex; // Only next found
        }

        // No PositionData events found at all (should be rare if session has positions)
        // or if the list only contains non-PositionData events around targetIndex.
        // Fallback: return the first or last event, or the original target.
        // For snapping, ideally we *always* find one if they exist.
        // If truly no PositionData in whole recording, slider shouldn't really "snap".
        if (!recordedSession.isEmpty()) return 0; // Fallback to start
        return targetIndex; // Fallback to original if truly nothing makes sense
    }
}

