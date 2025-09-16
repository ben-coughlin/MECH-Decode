package com.example.ftcfieldsimulator;

import javafx.application.Platform;
import java.util.ArrayList;
import java.util.function.Consumer;

public class RecordingManager {

    // +++ Make RecordedEvent public so it can be used by the main app for file operations +++
    public static class RecordedEvent {
        public final long timestamp;
        public final UdpPositionListener.UdpMessageData messageData;

        public RecordedEvent(long timestamp, UdpPositionListener.UdpMessageData messageData) {
            this.timestamp = timestamp;
            this.messageData = messageData;
        }
    }

    public enum PlaybackState { IDLE, RECORDING, PLAYING, PAUSED }

    private volatile PlaybackState currentState = PlaybackState.IDLE;
    private volatile int playbackIndex = 0;
    private ArrayList<RecordedEvent> recordedSession = new ArrayList<>(); // No longer final to allow loading

    private long recordingStartTime = 0;
    private Thread playbackThread;
    private final Consumer<UdpPositionListener.UdpMessageData> eventConsumer;
    private final Runnable onPlaybackFinished;
    private final Object pauseLock = new Object();

    public RecordingManager(Consumer<UdpPositionListener.UdpMessageData> eventConsumer, Runnable onPlaybackFinished) {
        this.eventConsumer = eventConsumer;
        this.onPlaybackFinished = onPlaybackFinished;
    }

    // --- NEW Methods for Save/Open ---

    /**
     * Returns a copy of the recorded session for saving.
     */
    public ArrayList<RecordedEvent> getRecordedSession() {
        // Return a copy to prevent external modification of the live session
        return new ArrayList<>(recordedSession);
    }

    /**
     * Loads a new recording session from a file, replacing the current one.
     */
    public void loadRecording(ArrayList<RecordedEvent> newSession) {
        stopPlayback(); // Ensure any current playback is fully stopped
        recordedSession = newSession;
        playbackIndex = 0; // Reset index to the start of the new recording
        currentState = PlaybackState.IDLE;
        System.out.println("Loaded recording with " + recordedSession.size() + " events.");
    }

    // --- Existing API for Controlling Recording and Playback ---

    public void startRecording() {
        stopPlayback(); // Stop any playback before starting a new recording
        if (currentState != PlaybackState.IDLE) return;
        currentState = PlaybackState.RECORDING;
        recordedSession.clear();
        playbackIndex = 0;
        recordingStartTime = System.currentTimeMillis();
        System.out.println("Recording started.");
    }

    public void addEvent(UdpPositionListener.UdpMessageData messageData) {
        if (currentState != PlaybackState.RECORDING) return;
        long timestamp = System.currentTimeMillis() - recordingStartTime;
        recordedSession.add(new RecordedEvent(timestamp, messageData));
    }

    public void stopRecording() {
        if (currentState != PlaybackState.RECORDING) return;
        currentState = PlaybackState.IDLE;
        System.out.println("Recording stopped. Total events: " + recordedSession.size());
    }

    public void play() {
        if (recordedSession.isEmpty() || currentState == PlaybackState.PLAYING) {
            return;
        }
        if (currentState == PlaybackState.PAUSED) {
            currentState = PlaybackState.PLAYING;
            synchronized (pauseLock) {
                pauseLock.notifyAll();
            }
            System.out.println("Playback resumed from index: " + playbackIndex);
            return;
        }
        if (playbackIndex >= recordedSession.size() - 1) {
            playbackIndex = 0;
        }
        currentState = PlaybackState.PLAYING;
        playbackThread = new Thread(this::runPlaybackLoop);
        playbackThread.setDaemon(true);
        playbackThread.start();
        System.out.println("Playback started from index: " + playbackIndex);
    }

    public void pause() {
        if (currentState != PlaybackState.PLAYING) return;
        currentState = PlaybackState.PAUSED;
        System.out.println("Playback paused at index: " + playbackIndex);
    }

    public void seekTo(int eventIndex) {
        if (eventIndex < 0 || eventIndex >= recordedSession.size()) {
            return;
        }
        if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
            stopPlayback();
        }
        this.playbackIndex = eventIndex;
        Platform.runLater(() -> eventConsumer.accept(recordedSession.get(playbackIndex).messageData));
        System.out.println("Seeked to event index: " + eventIndex);
    }

    private void runPlaybackLoop() {
        Platform.runLater(() -> eventConsumer.accept(recordedSession.get(playbackIndex).messageData));
        try {
            while (playbackIndex < recordedSession.size() - 1 && currentState != PlaybackState.IDLE) {
                synchronized (pauseLock) {
                    if (currentState == PlaybackState.PAUSED) {
                        pauseLock.wait();
                    }
                }
                if (currentState == PlaybackState.IDLE) break;
                long delayMillis = recordedSession.get(playbackIndex + 1).timestamp - recordedSession.get(playbackIndex).timestamp;
                if (delayMillis <= 0) delayMillis = 1;
                Thread.sleep(delayMillis);
                playbackIndex++;
                final RecordedEvent nextEvent = recordedSession.get(playbackIndex);
                Platform.runLater(() -> eventConsumer.accept(nextEvent.messageData));
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            System.out.println("Playback thread interrupted.");
        } finally {
            if (currentState != PlaybackState.IDLE) {
                Platform.runLater(this::stopPlayback);
            }
        }
    }

    private void stopPlayback() {
        if (currentState == PlaybackState.IDLE && (playbackThread == null || !playbackThread.isAlive())) {
            return;
        }
        currentState = PlaybackState.IDLE;
        if (playbackThread != null && playbackThread.isAlive()) {
            playbackThread.interrupt();
        }
        if (onPlaybackFinished != null) {
            Platform.runLater(onPlaybackFinished);
        }
        System.out.println("Playback stopped/finished.");
    }

    // --- Getters for UI state ---
    public PlaybackState getCurrentState() { return currentState; }
    public boolean hasRecording() { return !recordedSession.isEmpty(); }
    public int getPlaybackIndex() { return playbackIndex; }
    public int getTotalEvents() { return recordedSession.size(); }
}


//package com.example.ftcfieldsimulator; // Adjust package as needed
//
//import javafx.application.Platform;
//import java.util.ArrayList;
//import java.util.function.Consumer;
//
//public class RecordingManager {
//
//    // +++ The RecordedEvent class remains the same +++
//    private static class RecordedEvent {
//        final long timestamp; // Milliseconds relative to the start of the recording
//        final UdpPositionListener.UdpMessageData messageData;
//
//        RecordedEvent(long timestamp, UdpPositionListener.UdpMessageData messageData) {
//            this.timestamp = timestamp;
//            this.messageData = messageData;
//        }
//    }
//
//    // +++ The State Enum remains the same +++
//    public enum PlaybackState {
//        IDLE,
//        RECORDING,
//        PLAYING,
//        PAUSED
//    }
//
//    // +++ STATE VARIABLES: Use 'volatile' for thread safety +++
//    private volatile PlaybackState currentState = PlaybackState.IDLE;
//    private volatile int playbackIndex = 0;
//    private final ArrayList<RecordedEvent> recordedSession = new ArrayList<>();
//    private long recordingStartTime = 0;
//
//    // +++ THREADING AND CALLBACKS +++
//    private Thread playbackThread;
//    private final Consumer<UdpPositionListener.UdpMessageData> eventConsumer;
//    private final Runnable onPlaybackFinished;
//    private final Object pauseLock = new Object(); // Used for pausing/resuming the thread
//
//    public RecordingManager(Consumer<UdpPositionListener.UdpMessageData> eventConsumer, Runnable onPlaybackFinished) {
//        this.eventConsumer = eventConsumer;
//        this.onPlaybackFinished = onPlaybackFinished;
//    }
//
//    // --- Public API for Controlling Recording and Playback ---
//
//    public void startRecording() {
//        stopPlayback(); // Ensure any previous playback is stopped
//        if (currentState != PlaybackState.IDLE) return;
//        currentState = PlaybackState.RECORDING;
//        recordedSession.clear();
//        playbackIndex = 0;
//        recordingStartTime = System.currentTimeMillis();
//        System.out.println("Recording started.");
//    }
//
//    public void addEvent(UdpPositionListener.UdpMessageData messageData) {
//        if (currentState != PlaybackState.RECORDING) return;
//        long timestamp = System.currentTimeMillis() - recordingStartTime;
//        recordedSession.add(new RecordedEvent(timestamp, messageData));
//    }
//
//    public void stopRecording() {
//        if (currentState != PlaybackState.RECORDING) return;
//        currentState = PlaybackState.IDLE;
//        System.out.println("Recording stopped. Total events: " + recordedSession.size());
//    }
//
//    /**
//     * +++ REFACTORED: Plays the recording using a background thread. +++
//     */
//    public void play() {
//        if (recordedSession.isEmpty() || currentState == PlaybackState.PLAYING) {
//            return;
//        }
//
//        // If resuming from a PAUSED state
//        if (currentState == PlaybackState.PAUSED) {
//            currentState = PlaybackState.PLAYING;
//            // Notify the waiting playback thread to resume
//            synchronized (pauseLock) {
//                pauseLock.notifyAll();
//            }
//            System.out.println("Playback resumed from index: " + playbackIndex);
//            return;
//        }
//
//        // If starting fresh (from IDLE)
//        if (playbackIndex >= recordedSession.size() - 1) {
//            playbackIndex = 0; // Reset if at the end
//        }
//        currentState = PlaybackState.PLAYING;
//        // Create and start a new thread for the playback loop
//        playbackThread = new Thread(this::runPlaybackLoop);
//        playbackThread.setDaemon(true); // Ensures thread doesn't prevent app from closing
//        playbackThread.start();
//        System.out.println("Playback started from index: " + playbackIndex);
//    }
//
//    /**
//     * +++ REFACTORED: Pauses playback by changing the state flag. +++
//     */
//    public void pause() {
//        if (currentState != PlaybackState.PLAYING) return;
//        currentState = PlaybackState.PAUSED;
//        System.out.println("Playback paused at index: " + playbackIndex);
//    }
//
//    /**
//     * +++ REFACTORED: Seeks to an event by stopping the old thread. +++
//     */
//    public void seekTo(int eventIndex) {
//        if (eventIndex < 0 || eventIndex >= recordedSession.size()) {
//            return; // Invalid index
//        }
//
//        // Stop any currently running playback before seeking
//        if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
//            stopPlayback();
//        }
//
//        this.playbackIndex = eventIndex;
//
//        // Immediately update the UI to the seeked position
//        Platform.runLater(() -> eventConsumer.accept(recordedSession.get(playbackIndex).messageData));
//        System.out.println("Seeked to event index: " + eventIndex);
//    }
//
//    // --- Private Playback Loop and Helpers ---
//
//    /**
//     * +++ NEW: This is the core logic that runs on the background thread. +++
//     */
//    private void runPlaybackLoop() {
//        // Dispatch the first event immediately
//        Platform.runLater(() -> eventConsumer.accept(recordedSession.get(playbackIndex).messageData));
//
//        try {
//            while (playbackIndex < recordedSession.size() - 1 && currentState != PlaybackState.IDLE) {
//                // --- Handle Pausing ---
//                synchronized (pauseLock) {
//                    if (currentState == PlaybackState.PAUSED) {
//                        pauseLock.wait(); // The thread will sleep here until notified
//                    }
//                }
//
//                // If playback was stopped while paused, exit the loop
//                if (currentState == PlaybackState.IDLE) break;
//
//                // --- Handle Timing and Dispatch ---
//                long delayMillis = recordedSession.get(playbackIndex + 1).timestamp - recordedSession.get(playbackIndex).timestamp;
//                if (delayMillis <= 0) delayMillis = 1; // Minimum delay
//
//                Thread.sleep(delayMillis);
//
//                // Advance index and dispatch the next event on the UI thread
//                playbackIndex++;
//                final RecordedEvent nextEvent = recordedSession.get(playbackIndex);
//                Platform.runLater(() -> eventConsumer.accept(nextEvent.messageData));
//            }
//        } catch (InterruptedException e) {
//            // This is expected when stopPlayback() interrupts the thread.
//            Thread.currentThread().interrupt(); // Preserve the interrupted status
//            System.out.println("Playback thread interrupted.");
//        } finally {
//            // --- Final Cleanup ---
//            // Ensure that no matter how the loop exits, we end in a clean state.
//            if (currentState != PlaybackState.IDLE) {
//                Platform.runLater(this::stopPlayback);
//            }
//        }
//    }
//
//    /**
//     * +++ REFACTORED: Stops playback by interrupting the background thread. +++
//     */
//    private void stopPlayback() {
//        if (currentState == PlaybackState.IDLE && (playbackThread == null || !playbackThread.isAlive())) {
//            return; // Already stopped
//        }
//
//        currentState = PlaybackState.IDLE;
//        if (playbackThread != null && playbackThread.isAlive()) {
//            playbackThread.interrupt(); // This will break the sleep/wait in the loop
//        }
//
//        // Notify the main app that playback is done to reset UI buttons
//        if (onPlaybackFinished != null) {
//            Platform.runLater(onPlaybackFinished);
//        }
//        System.out.println("Playback stopped/finished.");
//    }
//
//
//    // --- Getters for UI state (unchanged) ---
//    public PlaybackState getCurrentState() { return currentState; }
//    public boolean hasRecording() { return !recordedSession.isEmpty(); }
//    public int getPlaybackIndex() { return playbackIndex; }
//    public int getTotalEvents() { return recordedSession.size(); }
//}
//
////package com.example.ftcfieldsimulator; // Adjust package as needed
////
////import javafx.animation.KeyFrame;
////import javafx.animation.Timeline;
////import javafx.util.Duration;
////
////import java.util.ArrayList;
////import java.util.function.Consumer;
////
////public class RecordingManager {
////
////    // --- Inner Class for Recorded Events ---
////    private static class RecordedEvent {
////        final long timestamp; // Milliseconds relative to the start of the recording
////        final UdpPositionListener.UdpMessageData messageData;
////
////        RecordedEvent(long timestamp, UdpPositionListener.UdpMessageData messageData) {
////            this.timestamp = timestamp;
////            this.messageData = messageData;
////        }
////    }
////
////    // --- Enums and State ---
////    public enum PlaybackState {
////        IDLE,
////        RECORDING,
////        PLAYING,
////        PAUSED
////    }
////
////    private PlaybackState currentState = PlaybackState.IDLE;
////    private final ArrayList<RecordedEvent> recordedSession = new ArrayList<>();
////    private long recordingStartTime = 0;
////    private int playbackIndex = 0;
////
////    // --- Playback and Callbacks ---
////    private final Timeline playbackTimeline;
////    private final Consumer<UdpPositionListener.UdpMessageData> eventConsumer;
////    private final Runnable onPlaybackFinished;
////
////    public RecordingManager(Consumer<UdpPositionListener.UdpMessageData> eventConsumer, Runnable onPlaybackFinished) {
////        this.eventConsumer = eventConsumer;
////        this.onPlaybackFinished = onPlaybackFinished;
////        this.playbackTimeline = new Timeline();
////        this.playbackTimeline.setOnFinished(e -> {
////            // This lambda is intentionally left empty. Our logic is handled by the KeyFrame action.
////        });
////    }
////
////    // --- Public API for Controlling Recording and Playback ---
////
////    public void startRecording() {
////        if (currentState != PlaybackState.IDLE) return;
////        currentState = PlaybackState.RECORDING;
////        recordedSession.clear();
////        playbackIndex = 0;
////        recordingStartTime = System.currentTimeMillis();
////        System.out.println("Recording started.");
////    }
////
////    public void addEvent(UdpPositionListener.UdpMessageData messageData) {
////        if (currentState != PlaybackState.RECORDING) return;
////        long timestamp = System.currentTimeMillis() - recordingStartTime;
////        recordedSession.add(new RecordedEvent(timestamp, messageData));
////    }
////
////    public void stopRecording() {
////        if (currentState != PlaybackState.RECORDING) return;
////        currentState = PlaybackState.IDLE;
////        System.out.println("Recording stopped. Total events: " + recordedSession.size());
////    }
////
////    /**
////     * --- MODIFIED METHOD ---
////     * Plays the recording from the current playbackIndex.
////     */
////    public void play() {
////        if (currentState == PlaybackState.PLAYING || recordedSession.isEmpty()) {
////            return; // Already playing or nothing to play
////        }
////
////        // If the playback index is at or past the end of the recording,
////        // reset it to the beginning so the user can easily play it again.
////        if (playbackIndex >= recordedSession.size() - 1) {
////            playbackIndex = 0;
////        }
////
////        // --- THE FIX: THIS BLOCK IS MODIFIED/ADDED ---
////        // Immediately dispatch the event at the current index to ensure
////        // the playback starts from the correct visual state. This is redundant
////        // if coming from seekTo, but crucial if resuming from pause or starting fresh.
////        if (!recordedSession.isEmpty()) {
////            eventConsumer.accept(recordedSession.get(playbackIndex).messageData);
////        }
////        // --- END OF FIX ---
////
////        currentState = PlaybackState.PLAYING;
////
////        // Start the timeline process.
////        scheduleNextPlaybackEvent();
////        playbackTimeline.play();
////        System.out.println("Playback started from index: " + playbackIndex);
////    }
////
////    public void pause() {
////        if (currentState != PlaybackState.PLAYING) return;
////        currentState = PlaybackState.PAUSED;
////        playbackTimeline.pause();
////        System.out.println("Playback paused at index: " + playbackIndex);
////    }
////
////    public void seekTo(int eventIndex) {
////        if (eventIndex < 0 || eventIndex >= recordedSession.size()) {
////            return; // Invalid index
////        }
////
////        // If playing, pause first to stop the timeline.
////        if (currentState == PlaybackState.PLAYING) {
////            pause();
////        }
////
////        this.playbackIndex = eventIndex;
////
////        // Immediately dispatch the event at the new position to update the robot's state
////        RecordedEvent targetEvent = recordedSession.get(playbackIndex);
////        eventConsumer.accept(targetEvent.messageData);
////        System.out.println("Seeked to event index: " + eventIndex);
////    }
////
////    // --- Getters for UI state ---
////
////    public PlaybackState getCurrentState() {
////        return currentState;
////    }
////
////    public boolean hasRecording() {
////        return !recordedSession.isEmpty();
////    }
////
////    public int getPlaybackIndex() {
////        return playbackIndex;
////    }
////
////    public int getTotalEvents() {
////        return recordedSession.size();
////    }
////
////    // --- Private Helper Methods ---
////
////    /**
////     * Schedules the next event in the timeline based on the current playbackIndex.
////     * The delay is calculated using the original timestamps.
////     */
////    private void scheduleNextPlaybackEvent() {
////        // Stop if we've reached the end of the recording.
////        if (playbackIndex >= recordedSession.size() - 1) {
////            stopPlayback();
////            return;
////        }
////
////        // Calculate the delay until the *next* event based on original timestamps.
////        long delayMillis = recordedSession.get(playbackIndex + 1).timestamp - recordedSession.get(playbackIndex).timestamp;
////
////        // Handle cases with zero or negative delay to prevent timeline errors.
////        if(delayMillis <= 0) {
////            delayMillis = 1; // Use a minimal 1ms delay.
////        }
////
////        KeyFrame keyFrame = new KeyFrame(Duration.millis(delayMillis), e -> {
////            playbackIndex++; // Move to the next event
////
////            // Dispatch the event data to the main app
////            RecordedEvent nextEvent = recordedSession.get(playbackIndex);
////            eventConsumer.accept(nextEvent.messageData);
////
////            // Schedule the event that comes after this one
////            scheduleNextPlaybackEvent();
////        });
////
////        playbackTimeline.getKeyFrames().setAll(keyFrame);
////    }
////
////    private void stopPlayback() {
////        currentState = PlaybackState.IDLE;
////        playbackTimeline.stop();
////        System.out.println("Playback finished.");
////        if (onPlaybackFinished != null) {
////            onPlaybackFinished.run();
////        }
////    }
////}
////
//////package com.example.ftcfieldsimulator;
//////
//////import javafx.animation.KeyFrame;
//////import javafx.animation.Timeline;
//////import javafx.application.Platform;
//////import javafx.util.Duration;
//////
////////import java.io.FileDescriptor;
////////import java.lang.classfile.Attributes;
//////import java.util.ArrayList;
//////import java.util.List;
//////import java.util.function.Consumer;
////////import java.util.function.Runnable;
//////
//////public class RecordingManager {
//////
//////    // A private inner class to hold each recorded event
//////    private static class RecordedEvent {
//////        final long timestampMs; // Time in milliseconds from the start of the recording
//////        final UdpPositionListener.UdpMessageData messageData;
//////
//////        RecordedEvent(long timestampMs, UdpPositionListener.UdpMessageData messageData) {
//////            this.timestampMs = timestampMs;
//////            this.messageData = messageData;
//////        }
//////    }
//////
//////    // Enum to manage the current state
//////    public enum PlaybackState {
//////        IDLE,
//////        RECORDING,
//////        PLAYING,
//////        PAUSED
//////    }
//////
//////    private PlaybackState currentState = PlaybackState.IDLE;
//////    private final List<RecordedEvent> recordedSession = new ArrayList<>();
//////    private long recordingStartTime;
//////
//////    private Timeline playbackTimeline;
//////    private int playbackIndex = 0;
//////    private final Consumer<UdpPositionListener.UdpMessageData> eventConsumer;
//////
//////    private final Runnable onPlaybackFinishedCallback;
//////
//////    /**
//////     * The RecordingManager needs a way to send events back into the application.
//////     * @param eventConsumer A method reference (like handleUdpMessage) to process replayed events.
//////     * @param onPlaybackFinishedCallback A method reference to call when playback stops or finishes.
//////     */
//////    public RecordingManager(Consumer<UdpPositionListener.UdpMessageData> eventConsumer, Runnable onPlaybackFinishedCallback) {
//////        this.eventConsumer = eventConsumer;
//////        this.onPlaybackFinishedCallback = onPlaybackFinishedCallback; // Assign the callback
//////    }
//////
//////    public PlaybackState getCurrentState() {
//////        return currentState;
//////    }
//////
//////    /**
//////     * Starts a new recording session. Clears any previous recording.
//////     */
//////    public void startRecording() {
//////        if (currentState == PlaybackState.PLAYING || currentState == PlaybackState.PAUSED) {
//////            stopPlayback();
//////        }
//////        recordedSession.clear();
//////        recordingStartTime = System.currentTimeMillis();
//////        currentState = PlaybackState.RECORDING;
//////        System.out.println("Recording started.");
//////    }
//////
//////    /**
//////     * Adds a new UDP message event to the current recording.
//////     * @param messageData The message data received from the listener.
//////     */
//////    public void addEvent(UdpPositionListener.UdpMessageData messageData) {
//////        if (currentState != PlaybackState.RECORDING) return;
//////        long timestamp = System.currentTimeMillis() - recordingStartTime;
//////        recordedSession.add(new RecordedEvent(timestamp, messageData));
//////    }
//////
//////    /**
//////     * Stops the current recording session.
//////     */
//////    public void stopRecording() {
//////        if (currentState != PlaybackState.RECORDING) return;
//////        currentState = PlaybackState.IDLE;
//////        System.out.println("Recording stopped. " + recordedSession.size() + " events recorded.");
//////    }
//////
//////    /**
//////     * Starts or resumes playback of the recorded session.
//////     */
//////    public void play() {
//////        if (recordedSession.isEmpty()) {
//////            System.out.println("No recording to play.");
//////            return;
//////        }
//////
//////        if (currentState == PlaybackState.IDLE || currentState == PlaybackState.PAUSED) {
//////            // If starting fresh, reset the playback index
//////            if (currentState == PlaybackState.IDLE) {
//////                playbackIndex = 0;
//////            }
//////
//////            currentState = PlaybackState.PLAYING;
//////            System.out.println("Playback started/resumed.");
//////            scheduleNextPlaybackEvent();
//////        }
//////    }
//////
//////    /**
//////     * Pauses the current playback.
//////     */
//////    public void pause() {
//////        if (currentState != PlaybackState.PLAYING) return;
//////        if (playbackTimeline != null) {
//////            playbackTimeline.stop();
//////        }
//////        currentState = PlaybackState.PAUSED;
//////        System.out.println("Playback paused.");
//////    }
//////
//////    /**
//////     * Schedules the next event in the recording using a Timeline.
//////     */
//////    private void scheduleNextPlaybackEvent() {
//////        if (playbackIndex >= recordedSession.size() || currentState != PlaybackState.PLAYING) {
//////            stopPlayback();
//////            return;
//////        }
//////
//////        RecordedEvent currentEvent = recordedSession.get(playbackIndex);
//////        long timeOfCurrentEvent = currentEvent.timestampMs;
//////
//////        // Dispatch the event immediately
//////        eventConsumer.accept(currentEvent.messageData);
//////        playbackIndex++;
//////
//////        // If there's a next event, schedule it based on the time difference
//////        if (playbackIndex < recordedSession.size()) {
//////            RecordedEvent nextEvent = recordedSession.get(playbackIndex);
//////            long delay = nextEvent.timestampMs - timeOfCurrentEvent;
//////
//////            playbackTimeline = new Timeline(new KeyFrame(Duration.millis(delay), e -> scheduleNextPlaybackEvent()));
//////            playbackTimeline.play();
//////        } else {
//////            // End of playback
//////            stopPlayback();
//////        }
//////    }
//////
//////    /**
//////     * Stops and resets the playback state.
//////     */
//////    private void stopPlayback() {
//////        if (playbackTimeline != null) {
//////            playbackTimeline.stop();
//////        }
//////        // Only trigger the full stop logic if we weren't already IDLE
//////        if (currentState != PlaybackState.IDLE) {
//////            currentState = PlaybackState.IDLE;
//////            playbackIndex = 0;
//////            System.out.println("Playback finished/stopped.");
//////
//////            // +++ NEW: Call the UI callback to reset the buttons +++
//////            if (onPlaybackFinishedCallback != null) {
//////                // Use Platform.runLater to ensure the callback runs on the FX Application thread,
//////                // which is required for making UI changes.
//////                Platform.runLater(onPlaybackFinishedCallback);
//////            }
//////        }
//////    }
//////
//////    public boolean hasRecording() {
//////        return !recordedSession.isEmpty();
//////    }
//////
//////    public int getPlaybackIndex() {
//////        return playbackIndex;
//////    }
//////
//////    public int getTotalEvents() {
//////        return recordedSession.size();
//////    }
//////
//////    /**
//////     * Seeks to a specific event in the recording.
//////     * This will pause playback if it is currently running.
//////     * @param eventIndex The index of the event to seek to.
//////     */
//////    public void seekTo(int eventIndex) {
//////        if (eventIndex < 0 || eventIndex >= recordedSession.size()) {
//////            return; // Invalid index
//////        }
//////
//////        // If playing, pause first
//////        if (currentState == PlaybackState.PLAYING) {
//////            pause();
//////        }
//////
//////        // Set the new playback index
//////        this.playbackIndex = eventIndex;
//////
//////        // Immediately dispatch the event at the new position to update the robot's state
//////        RecordedEvent targetEvent = recordedSession.get(playbackIndex);
//////        eventConsumer.accept(targetEvent.messageData);
//////
//////        System.out.println("Seeked to event index: " + eventIndex);
//////    }
//////}
