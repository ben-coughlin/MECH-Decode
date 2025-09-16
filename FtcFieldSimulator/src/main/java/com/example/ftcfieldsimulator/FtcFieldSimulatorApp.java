package com.example.ftcfieldsimulator; // Adjust package as needed

import com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData;
import com.example.ftcfieldsimulator.UdpPositionListener.PositionData;
import com.example.ftcfieldsimulator.UdpPositionListener.CircleData;
import com.example.ftcfieldsimulator.UdpPositionListener.LineData;
import com.example.ftcfieldsimulator.UdpPositionListener.TextData;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.geometry.Insets;
import javafx.geometry.Point2D;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.scene.paint.Color;

// +++ ADD IMPORTS for file operations +++
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

public class FtcFieldSimulatorApp extends Application {

    // --- UI and State Components ---
    private FieldDisplay fieldDisplay;
    private ControlPanel controlPanel;
    private Robot robot;
    private RecordingManager recordingManager;
    private UdpPositionListener udpListener;
    private Thread udpListenerThread;
    private Label instructionLabel;
    private Stage primaryStage; // +++ ADD: Field to hold the main stage for FileChooser +++

    // --- Path and Line Management ---
    private List<CurvePoint> currentPath = new ArrayList<>();
    private boolean isCreatingPath = false;
    private Map<String, UdpPositionListener.LineData> namedLinesToDraw = new HashMap<>();
    private final Object namedLinesLock = new Object();

    // --- Configuration Constants (simplified for brevity) ---
    public static final double FIELD_WIDTH_INCHES = 144.0;
    public static final double FIELD_HEIGHT_INCHES = 144.0;
    private static final int FIELD_DISPLAY_WIDTH_PIXELS = 720;
    private static final int FIELD_DISPLAY_HEIGHT_PIXELS = 720;
    private static final String FIELD_IMAGE_PATH = "/decode_field.png";
    private static final String ROBOT_IMAGE_PATH = "/robot.png";
    public static final double FIELD_IMAGE_ALPHA = 0.3;
    public static final double BACKGROUND_ALPHA = 0.1;
    public static final double ROBOT_START_FIELD_X = 0.0;
    public static final double ROBOT_START_FIELD_Y = 0.0;
    public static final double ROBOT_START_HEADING_DEGREES = 0.0;
    private static final int UDP_LISTENER_PORT = 7777;

    private static final double ROBOT_MOVE_INCREMENT_INCHES = 2.0; // Move 2 inches per key press
    private static final double ROBOT_TURN_INCREMENT_DEGREES = 5.0; // Turn 5 degrees per key press

    public static void main(String[] args) {
        Application.launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        // +++ FIX: Store the primary stage for later use +++
        this.primaryStage = primaryStage;
        primaryStage.setTitle("FTC Field Simulator");

        double controlPanelWidth = FIELD_DISPLAY_WIDTH_PIXELS * ControlPanel.PREFERRED_WIDTH_RATIO_TO_FIELD;
        double totalAppWidth = FIELD_DISPLAY_WIDTH_PIXELS + controlPanelWidth;
        double totalAppHeight = FIELD_DISPLAY_HEIGHT_PIXELS;

        this.recordingManager = new RecordingManager(this::handleUdpMessage, this::onPlaybackFinished);

        this.robot = new Robot(ROBOT_START_FIELD_X, ROBOT_START_FIELD_Y, ROBOT_START_HEADING_DEGREES, ROBOT_IMAGE_PATH);

        fieldDisplay = new FieldDisplay(FIELD_DISPLAY_WIDTH_PIXELS, FIELD_DISPLAY_HEIGHT_PIXELS, FIELD_WIDTH_INCHES, FIELD_HEIGHT_INCHES, FIELD_IMAGE_PATH, robot, BACKGROUND_ALPHA, FIELD_IMAGE_ALPHA);
        fieldDisplay.setNamedLinesMap(namedLinesToDraw);
        fieldDisplay.setRobotTextMessage(null);

        controlPanel = new ControlPanel(controlPanelWidth);

        instructionLabel = new Label("Click 'New Path' to start drawing.");
        instructionLabel.setPadding(new Insets(5));
        instructionLabel.setMaxWidth(Double.MAX_VALUE);
        instructionLabel.setAlignment(Pos.CENTER);
        HBox instructionPane = new HBox(instructionLabel);
        instructionPane.setAlignment(Pos.CENTER);
        instructionPane.setStyle("-fx-background-color: #CFD8DC;");

        BorderPane mainLayout = new BorderPane();
        mainLayout.setLeft(controlPanel);
        mainLayout.setCenter(fieldDisplay);
        mainLayout.setBottom(instructionPane);

        double instructionPaneHeight = 30;
        Scene scene = new Scene(mainLayout, totalAppWidth, totalAppHeight + instructionPaneHeight);

        // This ensures arrow keys work for robot movement even if a button has focus.
        scene.addEventFilter(KeyEvent.KEY_PRESSED, this::handleSceneKeyPress);

        // --- Event Handlers Setup ---
        setupControlPanelActions(primaryStage); // Pass stage for CSV export
        setupRecordingControlActions();          // This now includes Open/Save logic
//        scene.setOnKeyPressed(this::handleSceneKeyPress);

        startUdpListener();

        primaryStage.setScene(scene);
        primaryStage.setResizable(true);
        primaryStage.show();
        primaryStage.setOnCloseRequest(event -> stopApp());

        updateUIFromRobotState();
    }

    private void setupControlPanelActions(Stage ownerStage) {
        controlPanel.setOnNewPathAction(event -> startNewPathCreation());
        controlPanel.setOnDeletePathAction(event -> deleteCurrentPath());
        controlPanel.setOnExportPathAction(event -> exportPathToCSV(ownerStage));
        controlPanel.setOnClearTrailAction(event -> {
            fieldDisplay.clearTrail();
            fieldDisplay.drawCurrentState();
            instructionLabel.setText("Robot trail cleared.");
        });
        controlPanel.setOnClearNamedLinesAction(event -> {
            clearAllNamedLines();
            instructionLabel.setText("All custom lines cleared.");
        });
        controlPanel.enablePathControls(!currentPath.isEmpty());
    }

    // +++ FIX: Updated to include Open/Save handlers and button state logic +++
    private void setupRecordingControlActions() {
        // Connect the Open and Save buttons to their handler methods
        controlPanel.setOnOpenAction(e -> handleOpenRecording());
        controlPanel.setOnSaveAction(e -> handleSaveRecording());

        // Action for the Record/Stop button
        controlPanel.setOnRecordAction(() -> {
            if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
                recordingManager.stopRecording();
                controlPanel.toggleRecordButtonIcon(false);

                if (recordingManager.hasRecording()) {
                    controlPanel.setPlaybackControlsDisabled(false);
                    controlPanel.setTimelineSliderDisabled(false);
                    controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
                    controlPanel.setSaveButtonDisabled(false); // Enable Save button
                }
            } else {
                recordingManager.startRecording();
                controlPanel.toggleRecordButtonIcon(true);
                controlPanel.setPlaybackControlsDisabled(true);
                controlPanel.togglePlayPauseButtonIcon(false);
                controlPanel.setTimelineSliderDisabled(true);
                controlPanel.setSaveButtonDisabled(true); // Disable Save while recording
            }
        });

        // Action for the Play/Pause button
        controlPanel.setOnPlayPauseAction(() -> {
            RecordingManager.PlaybackState state = recordingManager.getCurrentState();
            if (state == RecordingManager.PlaybackState.PLAYING) {
                recordingManager.pause();
                controlPanel.togglePlayPauseButtonIcon(false);
            } else if (state == RecordingManager.PlaybackState.IDLE || state == RecordingManager.PlaybackState.PAUSED) {
                recordingManager.play();
                controlPanel.togglePlayPauseButtonIcon(true);
            }
        });

        // Action for the Timeline Slider
        controlPanel.setOnTimelineSliderChanged((observable, oldValue, newValue) -> {
            if (controlPanel.getTimelineSlider().isValueChanging()) {
                recordingManager.seekTo(newValue.intValue());
            }
        });
        controlPanel.setOnSliderMouseReleased(event -> {
            int seekIndex = (int) controlPanel.getTimelineSlider().getValue();
            recordingManager.seekTo(seekIndex);
        });
    }

    // +++ ADD: New methods for saving and opening recording files +++

    private void handleSaveRecording() {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Save Recording");
        fileChooser.setInitialFileName("recording.rec");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));

        File file = fileChooser.showSaveDialog(primaryStage);
        if (file == null) return; // User cancelled

        ArrayList<RecordingManager.RecordedEvent> events = recordingManager.getRecordedSession();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            for (RecordingManager.RecordedEvent event : events) {
                String line = formatEventToString(event);
                if (line != null) {
                    writer.write(line);
                    writer.newLine();
                }
            }
            System.out.println("Recording saved to: " + file.getAbsolutePath());
            instructionLabel.setText("Recording saved successfully.");
        } catch (IOException e) {
            System.err.println("Error saving recording: " + e.getMessage());
            instructionLabel.setText("Error saving recording.");
        }
    }


    /**
     * Converts a RecordedEvent object into a string format for saving to a file.
     * The new format is: "timestamp|prefix:data,values,..."
     */
    private String formatEventToString(RecordingManager.RecordedEvent event) {
        // First, get the string representation of the data payload
        String payload;
        UdpPositionListener.UdpMessageData data = event.messageData;

        if (data instanceof UdpPositionListener.PositionData) {
            UdpPositionListener.PositionData d = (UdpPositionListener.PositionData) data;
            payload = String.format("pos:%.3f,%.3f,%.3f", d.x, d.y, d.heading);
        } else if (data instanceof UdpPositionListener.CircleData) {
            UdpPositionListener.CircleData d = (UdpPositionListener.CircleData) data;
            payload = String.format("cir:%.3f,%.3f", d.radiusInches, d.heading);
        } else if (data instanceof UdpPositionListener.LineData) {
            UdpPositionListener.LineData d = (UdpPositionListener.LineData) data;
            payload = String.format("line:%s,%.3f,%.3f,%.3f,%.3f,%d", d.name, d.x1, d.y1, d.x2, d.y2, d.styleCode);
        } else if (data instanceof UdpPositionListener.TextData) {
            UdpPositionListener.TextData d = (UdpPositionListener.TextData) data;
            payload = "txt:" + d.text;
        } else {
            return null; // Don't save unknown types
        }

        // Prepend the timestamp and a separator to the payload
        return event.timestamp + "|" + payload;
    }

//    /**
//     * Converts a UdpMessageData object into a string format for saving to a file.
//     * This format now mirrors the UDP message format (e.g., "pos:x,y,h").
//     */
//    private String formatEventToString(RecordingManager.RecordedEvent event) {
//        UdpPositionListener.UdpMessageData data = event.messageData;
//
//        if (data instanceof UdpPositionListener.PositionData) {
//            UdpPositionListener.PositionData d = (UdpPositionListener.PositionData) data;
//            return String.format("pos:%.3f,%.3f,%.3f", d.x, d.y, d.heading);
//        } else if (data instanceof UdpPositionListener.CircleData) {
//            UdpPositionListener.CircleData d = (UdpPositionListener.CircleData) data;
//            return String.format("cir:%.3f,%.3f", d.radiusInches, d.heading);
//        } else if (data instanceof UdpPositionListener.LineData) {
//            UdpPositionListener.LineData d = (UdpPositionListener.LineData) data;
//            // --- FIX: Ensure the styleCode is saved to the file ---
//            return String.format("line:%s,%.3f,%.3f,%.3f,%.3f,%d", d.name, d.x1, d.y1, d.x2, d.y2, d.styleCode);
//        } else if (data instanceof UdpPositionListener.TextData) {
//            UdpPositionListener.TextData d = (UdpPositionListener.TextData) data;
//            return "txt:" + d.text;
//        }
//        return null;
//    }

//    private String formatEventToString(RecordingManager.RecordedEvent event) {
//        long ts = event.timestamp;
//        UdpMessageData data = event.messageData;
//
//        if (data instanceof PositionData) {
//            PositionData d = (PositionData) data;
//            return String.join("|", "POS", String.valueOf(ts), String.valueOf(d.x), String.valueOf(d.y), String.valueOf(d.heading));
//        } else if (data instanceof CircleData) {
//            CircleData d = (CircleData) data;
//            return String.join("|", "CIRCLE", String.valueOf(ts), String.valueOf(d.radiusInches), String.valueOf(d.heading));
//        } else if (data instanceof LineData) {
//            LineData d = (LineData) data;
//            // --- FIX: Add the styleCode to the saved string format ---
//            return String.join("|", "LINE", String.valueOf(ts), d.name, String.valueOf(d.x1), String.valueOf(d.y1), String.valueOf(d.x2), String.valueOf(d.y2), String.valueOf(d.styleCode));
//        } else if (data instanceof TextData) {
//            TextData d = (TextData) data;
//            return String.join("|", "TEXT", String.valueOf(ts), d.text);
//        }
//        return null; // Unknown type
//    }

//    private String formatEventToString(RecordingManager.RecordedEvent event) {
//        long ts = event.timestamp;
//        UdpMessageData data = event.messageData;
//
//        if (data instanceof PositionData) {
//            PositionData d = (PositionData) data;
//            return String.join("|", "POS", String.valueOf(ts), String.valueOf(d.x), String.valueOf(d.y), String.valueOf(d.heading));
//        } else if (data instanceof CircleData) {
//            // Per your request, CircleData only uses radius and heading from the packet.
//            CircleData d = (CircleData) data;
//            // The file format will only store what's used, to match the implementation.
//            return String.join("|", "CIRCLE", String.valueOf(ts), String.valueOf(d.radiusInches), String.valueOf(d.heading));
//        } else if (data instanceof LineData) {
//            LineData d = (LineData) data;
//            // Split by 3 to handle names that might have a pipe, although it's not robust.
//            // Using a more complex format like JSON would be better for such cases.
//            return String.join("|", "LINE", String.valueOf(ts), d.name, String.valueOf(d.x1), String.valueOf(d.y1), String.valueOf(d.x2), String.valueOf(d.y2));
//        } else if (data instanceof TextData) {
//            TextData d = (TextData) data;
//            return String.join("|", "TEXT", String.valueOf(ts), d.text);
//        }
//        return null; // Unknown type
//    }

    /**
     * Handles opening a .rec file, parsing its contents (including timestamps),
     * and loading it into the RecordingManager.
     */
    private void handleOpenRecording() {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Open Recording");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));

        File file = fileChooser.showOpenDialog(primaryStage);
        if (file == null) return; // User cancelled

        ArrayList<RecordingManager.RecordedEvent> loadedEvents = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                // --- FIX: Split the line into timestamp and payload ---
                String[] lineParts = line.split("\\|", 2);
                if (lineParts.length != 2) {
                    System.err.println("Skipping malformed line in recording file: " + line);
                    continue;
                }

                long timestamp = Long.parseLong(lineParts[0]);
                String payload = lineParts[1];

                UdpPositionListener.UdpMessageData parsedData = null;

                // The rest of the parsing logic now operates on the 'payload'
                if (payload.startsWith("pos:")) {
                    String content = payload.substring(4);
                    String[] parts = content.split(",");
                    if (parts.length == 3) {
                        parsedData = new UdpPositionListener.PositionData(
                                Double.parseDouble(parts[0].trim()),
                                Double.parseDouble(parts[1].trim()),
                                Double.parseDouble(parts[2].trim())
                        );
                    }
                } else if (payload.startsWith("cir:")) {
                    String content = payload.substring(4);
                    String[] parts = content.split(",");
                    if (parts.length == 2) {
                        parsedData = new UdpPositionListener.CircleData(
                                Double.parseDouble(parts[0].trim()),
                                Double.parseDouble(parts[1].trim())
                        );
                    }
                } else if (payload.startsWith("line:")) {
                    String content = payload.substring(5);
                    String[] parts = content.split(",", 6);
                    if (parts.length == 6) {
                        parsedData = new UdpPositionListener.LineData(
                                parts[0].trim(),
                                Double.parseDouble(parts[1].trim()),
                                Double.parseDouble(parts[2].trim()),
                                Double.parseDouble(parts[3].trim()),
                                Double.parseDouble(parts[4].trim()),
                                Integer.parseInt(parts[5].trim())
                        );
                    }
                } else if (payload.startsWith("txt:")) {
                    parsedData = new UdpPositionListener.TextData(payload.substring(4));
                }

                if (parsedData != null) {
                    // --- FIX: Use the actual timestamp from the file ---
                    loadedEvents.add(new RecordingManager.RecordedEvent(timestamp, parsedData));
                }
            }

            // --- Successfully loaded, now update the app state ---
            recordingManager.loadRecording(loadedEvents);
            controlPanel.setPlaybackControlsDisabled(loadedEvents.isEmpty());
            controlPanel.setTimelineSliderDisabled(loadedEvents.isEmpty());
            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
            controlPanel.setSaveButtonDisabled(loadedEvents.isEmpty());
            controlPanel.togglePlayPauseButtonIcon(false);

            instructionLabel.setText("Opened recording: " + file.getName());
            System.out.println("Successfully opened recording: " + file.getAbsolutePath());

        } catch (IOException | NumberFormatException | ArrayIndexOutOfBoundsException e) {
            System.err.println("Error opening or parsing recording file: " + e.getMessage());
            instructionLabel.setText("Error opening recording file.");
        }
    }

//    /**
//     * Handles opening a .rec file, parsing its contents, and loading it into the RecordingManager.
//     */
//    private void handleOpenRecording() {
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Open Recording");
//        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
//
//        File file = fileChooser.showOpenDialog(primaryStage);
//        if (file == null) return; // User cancelled
//
//        ArrayList<RecordingManager.RecordedEvent> loadedEvents = new ArrayList<>();
//        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
//            String line;
//            long lastTimestamp = 0; // To reconstruct relative timestamps for playback
//
//            while ((line = reader.readLine()) != null) {
//                UdpPositionListener.UdpMessageData parsedData = null;
//
//                // --- FIX: The parsing logic now mirrors the UdpPositionListener ---
//                if (line.startsWith("pos:")) {
//                    String content = line.substring(4);
//                    String[] parts = content.split(",");
//                    if (parts.length == 3) {
//                        parsedData = new UdpPositionListener.PositionData(
//                                Double.parseDouble(parts[0].trim()),
//                                Double.parseDouble(parts[1].trim()),
//                                Double.parseDouble(parts[2].trim())
//                        );
//                    }
//                } else if (line.startsWith("cir:")) {
//                    String content = line.substring(4);
//                    String[] parts = content.split(",");
//                    if (parts.length == 2) {
//                        parsedData = new UdpPositionListener.CircleData(
//                                Double.parseDouble(parts[0].trim()),
//                                Double.parseDouble(parts[1].trim())
//                        );
//                    }
//                } else if (line.startsWith("line:")) {
//                    String content = line.substring(5);
//                    String[] parts = content.split(",", 6);
//                    if (parts.length == 6) {
//                        // This now matches the 6-argument constructor
//                        parsedData = new UdpPositionListener.LineData(
//                                parts[0].trim(),
//                                Double.parseDouble(parts[1].trim()),
//                                Double.parseDouble(parts[2].trim()),
//                                Double.parseDouble(parts[3].trim()),
//                                Double.parseDouble(parts[4].trim()),
//                                Integer.parseInt(parts[5].trim())
//                        );
//                    }
//                } else if (line.startsWith("txt:")) {
//                    parsedData = new UdpPositionListener.TextData(line.substring(4));
//                }
//
//                if (parsedData != null) {
//                    // The original live timestamps are not saved in this format.
//                    // We create artificial timestamps to ensure playback works correctly.
//                    lastTimestamp += 50; // Assign a 50ms interval for each event
//                    loadedEvents.add(new RecordingManager.RecordedEvent(lastTimestamp, parsedData));
//                }
//            }
//
//            // --- Successfully loaded, now update the app state ---
//            recordingManager.loadRecording(loadedEvents);
//            controlPanel.setPlaybackControlsDisabled(loadedEvents.isEmpty());
//            controlPanel.setTimelineSliderDisabled(loadedEvents.isEmpty());
//            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
//            controlPanel.setSaveButtonDisabled(loadedEvents.isEmpty());
//            controlPanel.togglePlayPauseButtonIcon(false);
//
//            instructionLabel.setText("Opened recording: " + file.getName());
//            System.out.println("Successfully opened recording: " + file.getAbsolutePath());
//
//        } catch (IOException | NumberFormatException | ArrayIndexOutOfBoundsException e) {
//            System.err.println("Error opening or parsing recording file: " + e.getMessage());
//            instructionLabel.setText("Error opening recording file.");
//        }
//    }

//    private void handleOpenRecording() {
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Open Recording");
//        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
//
//        File file = fileChooser.showOpenDialog(primaryStage);
//        if (file == null) return; // User cancelled
//
//        ArrayList<RecordingManager.RecordedEvent> loadedEvents = new ArrayList<>();
//        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
//            String line;
//            while ((line = reader.readLine()) != null) {
//                String[] parts = line.split("\\|", -1);
//                if (parts.length < 2) continue;
//
//                String type = parts[0];
//                long ts = Long.parseLong(parts[1]);
//                UdpMessageData data = null;
//
//                switch (type) {
//                    case "POS":
//                        data = new PositionData(Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), Double.parseDouble(parts[4]));
//                        break;
//                    case "CIRCLE":
//                        data = new CircleData(0, 0, Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), "");
//                        break;
//                    case "LINE":
//                        // --- FIX: Parse the styleCode (part 7) and pass it to the constructor ---
//                        data = new LineData(parts[2], Double.parseDouble(parts[3]), Double.parseDouble(parts[4]), Double.parseDouble(parts[5]), Double.parseDouble(parts[6]), Integer.parseInt(parts[7]));
//                        break;
//                    case "TEXT":
//                        data = new TextData(parts.length > 2 ? parts[2] : "");
//                        break;
//                }
//                if (data != null) {
//                    loadedEvents.add(new RecordingManager.RecordedEvent(ts, data));
//                }
//            }
//
//            // --- Successfully loaded, now update the app state ---
//            recordingManager.loadRecording(loadedEvents);
//            controlPanel.setPlaybackControlsDisabled(false);
//            controlPanel.setTimelineSliderDisabled(false);
//            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
//            controlPanel.setSaveButtonDisabled(false);
//            controlPanel.togglePlayPauseButtonIcon(false);
//
//            instructionLabel.setText("Opened recording: " + file.getName());
//            System.out.println("Successfully opened recording: " + file.getAbsolutePath());
//
//        } catch (IOException | NumberFormatException | ArrayIndexOutOfBoundsException e) {
//            System.err.println("Error opening or parsing recording file: " + e.getMessage());
//            instructionLabel.setText("Error opening recording file.");
//        }
//    }

//    private void handleOpenRecording() {
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Open Recording");
//        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
//
//        File file = fileChooser.showOpenDialog(primaryStage);
//        if (file == null) return; // User cancelled
//
//        ArrayList<RecordingManager.RecordedEvent> loadedEvents = new ArrayList<>();
//        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
//            String line;
//            while ((line = reader.readLine()) != null) {
//                String[] parts = line.split("\\|", -1); // Use -1 limit to preserve trailing empty fields
//                if (parts.length < 2) continue; // Malformed line
//
//                String type = parts[0];
//                long ts = Long.parseLong(parts[1]);
//                UdpMessageData data = null;
//
//                switch (type) {
//                    case "POS":
//                        data = new PositionData(Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), Double.parseDouble(parts[4]));
//                        break;
//                    case "CIRCLE":
//                        // Create a CircleData with dummy x/y (0,0) since they aren't used in your implementation.
//                        data = new CircleData(0, 0, Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), "");
//                        break;
//                    case "LINE":
//                        data = new LineData(parts[2], Double.parseDouble(parts[3]), Double.parseDouble(parts[4]), Double.parseDouble(parts[5]), Double.parseDouble(parts[6]));
//                        break;
//                    case "TEXT":
//                        // Re-join parts in case the text itself had a pipe character
//                        // A more robust solution would be to split only the first 2 parts.
//                        data = new TextData(parts.length > 2 ? parts[2] : "");
//                        break;
//                }
//                if (data != null) {
//                    loadedEvents.add(new RecordingManager.RecordedEvent(ts, data));
//                }
//            }
//
//            // --- Successfully loaded, now update the app state ---
//            recordingManager.loadRecording(loadedEvents);
//
//            // Update UI controls to reflect the loaded recording
//            controlPanel.setPlaybackControlsDisabled(false);
//            controlPanel.setTimelineSliderDisabled(false);
//            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
//            controlPanel.setSaveButtonDisabled(false); // Enable save, user might want to re-save it
//            controlPanel.togglePlayPauseButtonIcon(false); // Ensure play icon is shown
//
//            instructionLabel.setText("Opened recording: " + file.getName());
//            System.out.println("Successfully opened recording: " + file.getAbsolutePath());
//
//        } catch (IOException | NumberFormatException | ArrayIndexOutOfBoundsException e) {
//            System.err.println("Error opening or parsing recording file: " + e.getMessage());
//            instructionLabel.setText("Error opening recording file.");
//        }
//    }

    /**
     * +++ THIS METHOD IS NOW FIXED +++
     * This method handles all incoming UDP messages.
     * The recording logic is now moved inside Platform.runLater() to prevent race conditions.
     * All interactions with RecordingManager now happen safely on the JavaFX Application Thread.
     */
    private void handleUdpMessage(UdpMessageData messageData) {
        if (messageData == null) return;

        // By moving all logic inside runLater, we ensure that both adding an event
        // and updating the UI happen sequentially on the main JavaFX thread.
        Platform.runLater(() -> {
            // --- FIX: Check recording state and add the event here. This is now thread-safe. ---
            if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
                recordingManager.addEvent(messageData);
            }
            // --- END OF FIX ---

            // The rest of the UI update logic can now proceed safely.
            if (messageData instanceof PositionData) {
                PositionData pose = (PositionData) messageData;
                if (robot != null && fieldDisplay != null) {
                    fieldDisplay.addTrailDot(robot.getXInches(), robot.getYInches());
                    robot.setPosition(pose.x, pose.y);
                    robot.setHeading(pose.heading);
                }
            } else if (messageData instanceof CircleData) {
                CircleData circle = (CircleData) messageData;
                if (fieldDisplay != null) {
                    // This draws the circle at the robot's current location.
                    fieldDisplay.addDebugCircle(
                            robot.getXInches(),
                            robot.getYInches(),
                            circle.radiusInches,
                            circle.heading,
                            Color.rgb(255, 165, 0, 0.7)
                    );
                }
            } else if (messageData instanceof LineData) {
                LineData line = (LineData) messageData;
                synchronized (namedLinesLock) {
                    namedLinesToDraw.put(line.name, line);
                }
            } else if (messageData instanceof TextData) {
                TextData text = (TextData) messageData;
                if (fieldDisplay != null) {
                    fieldDisplay.setRobotTextMessage(text.text);
                }
            }

            // Update the UI
            if (messageData instanceof PositionData) {
                updateUIFromRobotState();
            } else {
                fieldDisplay.drawCurrentState();
            }
        });
    }

//    // +++ The rest of your methods remain largely the same +++
//
//    private void handleUdpMessage(UdpMessageData messageData) {
//        if (messageData == null) return;
//
//        if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
//            recordingManager.addEvent(messageData);
//        }
//
//        Platform.runLater(() -> {
//            if (messageData instanceof PositionData) {
//                PositionData pose = (PositionData) messageData;
//                if (robot != null && fieldDisplay != null) {
//                    fieldDisplay.addTrailDot(robot.getXInches(), robot.getYInches());
//                    robot.setPosition(pose.x, pose.y);
//                    robot.setHeading(pose.heading);
//                }
//            } else if (messageData instanceof CircleData) {
//                CircleData circle = (CircleData) messageData;
//                if (fieldDisplay != null) {
//                    // This draws the circle at the robot's current location, as you requested.
//                    fieldDisplay.addDebugCircle(robot.getXInches(), robot.getYInches(), circle.radiusInches, circle.heading, Color.rgb(255, 165, 0, 0.7));
//                }
//            } else if (messageData instanceof LineData) {
//                LineData line = (LineData) messageData;
//                synchronized (namedLinesLock) {
//                    namedLinesToDraw.put(line.name, line);
//                }
//            } else if (messageData instanceof TextData) {
//                TextData text = (TextData) messageData;
//                if (fieldDisplay != null) {
//                    fieldDisplay.setRobotTextMessage(text.text);
//                }
//            }
//
//            if (messageData instanceof PositionData) {
//                updateUIFromRobotState();
//            } else {
//                fieldDisplay.drawCurrentState();
//            }
//        });
//    }

    private void onPlaybackFinished() {
        Platform.runLater(() -> controlPanel.togglePlayPauseButtonIcon(false));
    }

    private void startUdpListener() {
        try {
            udpListener = new UdpPositionListener(UDP_LISTENER_PORT, this::handleUdpMessage);
            udpListenerThread = new Thread(udpListener, "UdpListenerThread");
            udpListenerThread.setDaemon(true);
            udpListenerThread.start();
            System.out.println("UDP Listener started on port " + UDP_LISTENER_PORT);
        } catch (Exception e) {
            System.err.println("Error starting UDP listener: " + e.getMessage());
            instructionLabel.setText("ERROR: Could not start UDP listener on port " + UDP_LISTENER_PORT);
        }
    }

    private void updateUIFromRobotState() {
        if (robot != null && controlPanel != null && fieldDisplay != null) {
            double displayHeading = robot.getHeadingDegrees() % 360;
            if (displayHeading < 0) displayHeading += 360;
            controlPanel.updateRobotStatus(robot.getXInches(), robot.getYInches(), displayHeading);
            fieldDisplay.drawCurrentState();
        }
    }

    private void stopApp() {
        if (udpListener != null) udpListener.stopListener();
        if (udpListenerThread != null && udpListenerThread.isAlive()) {
            try {
                udpListenerThread.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        System.out.println("Exiting application.");
        Platform.exit();
        System.exit(0);
    }

    // --- Placeholder/Dummy methods from your file to ensure it compiles ---
//    private void clearAllNamedLines() { /* ... */ }
//    private void handleSceneKeyPress(KeyEvent event) { /* ... */ }
//    private void startNewPathCreation() { /* ... */ }
//    private void handleFieldClickForPath(Point2D pixelCoords) { /* ... */ }
//    private void finishPathCreation(boolean cancelled) { /* ... */ }
//    private void deleteCurrentPath() { /* ... */ }
//    private void exportPathToCSV(Stage ownerStage) { /* ... */ }

    private void exportPathToCSV(Stage ownerStage) {
        if (currentPath.isEmpty()) {
            instructionLabel.setText("No path to export.");
            return;
        }

        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Save Path to CSV");
        fileChooser.setInitialFileName("ftc_path.csv");
        fileChooser.getExtensionFilters().add(
                new FileChooser.ExtensionFilter("CSV Files (*.csv)", "*.csv")
        );

        File file = fileChooser.showSaveDialog(ownerStage);
        if (file != null) {
            try (PrintWriter writer = new PrintWriter(file)) {
                writer.println("x_inches,y_inches"); // CSV Header
                for (CurvePoint point : currentPath) {
                    writer.printf("%.3f,%.3f\n", point.x, point.y);
                }
                instructionLabel.setText("Path exported successfully to " + file.getName());
            } catch (Exception e) {
                instructionLabel.setText("Error exporting path: " + e.getMessage());
                e.printStackTrace();
            }
        } else {
            instructionLabel.setText("Path export cancelled.");
        }
    }

    private void deleteCurrentPath() {
        currentPath.clear();
        isCreatingPath = false; // Ensure path creation mode is exited
        fieldDisplay.setPathCreationMode(
                false,
                null,  // For single clicks to add points
                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
        );
        fieldDisplay.setPathToDraw(currentPath);
        fieldDisplay.drawCurrentState();
        instructionLabel.setText("Path deleted. Click 'New Path' to start drawing.");
        controlPanel.enablePathControls(false);
        controlPanel.setPathEditingActive(false); // Reset path editing buttons
    }

    /**
     * Finishes the path creation process, either by completing or cancelling it.
     * This method is now responsible for resetting the state of the UI controls.
     *
     * @param cancelled True if the path was cancelled (e.g., by pressing ESC).
     */
    private void finishPathCreation(boolean cancelled) {
        // Guard against being called when not in creation mode
        if (!isCreatingPath) return;

        isCreatingPath = false;

        // Tell the FieldDisplay to stop listening for path creation mouse events
        fieldDisplay.setPathCreationMode(false, null, null);

        // --- FIX: Logic to handle state after finishing ---
        if (cancelled) {
            currentPath.clear(); // If the user cancelled, wipe the incomplete path
            instructionLabel.setText("Path creation cancelled. Click 'New Path' to start again.");
        } else {
            // Path was finished successfully (e.g., by double-clicking)
            if (currentPath.isEmpty()) {
                instructionLabel.setText("Path finished with no points. Click 'New Path' to start again.");
            } else {
                instructionLabel.setText("Path finished. You can now Export or Delete it.");
            }
        }

        // 1. Exit the "path editing" mode to show the main path buttons again.
        controlPanel.setPathEditingActive(false);

        // 2. Enable or disable the "Delete" and "Export" buttons based on whether the path has points.
        controlPanel.enablePathControls(!currentPath.isEmpty());

        // Redraw the field to show the final path (or clear it if cancelled)
        fieldDisplay.setPathToDraw(currentPath);
        fieldDisplay.drawCurrentState();
    }

//    // Call this when path creation is done (e.g., ESC, Double Click, or a "Finish" button)
//    private void finishPathCreation(boolean cancelled) {
//        if (!isCreatingPath) return;
//
//        System.out.println("Finishing path creation. Cancelled: " + cancelled + ". Current isCreatingPath: " + isCreatingPath);
//        isCreatingPath = false;
//        System.out.println("Set isCreatingPath to: " + isCreatingPath);
//        fieldDisplay.setPathCreationMode(
//                false,
//                null,  // For single clicks to add points
//                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
//        );
//    }

    private void handleFieldClickForPath(Point2D pixelCoords) { // pixelCoords are from top-left of canvas
        if (!isCreatingPath) return;

    //        Point2D inchesCoords = fieldDisplay.pixelToInches(pixelCoords.getX(), pixelCoords.getY());
        Point2D inchesCoordsFieldCenter = fieldDisplay.pixelToInches(pixelCoords.getX(), pixelCoords.getY());
        // Optional: Prevent adding a point if it's too close to the previous one on double-click
        // if (isDoubleClick && !currentPath.isEmpty()) {
        //     CurvePoint lastPoint = currentPath.get(currentPath.size() - 1);
        //     if (Math.hypot(inchesCoords.getX() - lastPoint.x, inchesCoords.getY() - lastPoint.y) < SOME_THRESHOLD) {
        //          // It's a double click very near the last point, don't add it again.
        //          // The finishPathCreation will be triggered by the separate onPathFinishListener.
        //          return;
        //     }
        // }
        currentPath.add(new CurvePoint(inchesCoordsFieldCenter.getX(), inchesCoordsFieldCenter.getY()));
        fieldDisplay.setPathToDraw(currentPath); // Update the path being drawn
        fieldDisplay.drawCurrentState();

        if (currentPath.size() == 1) {
            instructionLabel.setText("Click the next waypoint. ESC to cancel.");
        } else {
            instructionLabel.setText("Click next waypoint, Double-click last point, or ESC to cancel.");
        }

        // Check for double click to finish path
        // FieldDisplay mouse event handler needs to be more specific for this
        // For simplicity now, let's rely on ESC or a dedicated "Finish Path" button later.
        // Alternative: The passed MouseEvent in FieldDisplay could check clickCount.
        // For now, focusing on ESC / explicit finish.
    }

    private void clearAllNamedLines() {
        synchronized (namedLinesLock) {
            namedLinesToDraw.clear();
        }
        if (fieldDisplay != null) {
            fieldDisplay.drawCurrentState(); // Redraw to reflect the cleared lines
        }
    }

    // This is the handler that the scene's event filter now calls.
    private void handleSceneKeyPress(KeyEvent event) {
        if (isCreatingPath) {
            if (event.getCode() == KeyCode.ESCAPE) {
                finishPathCreation(true);
                event.consume();
            }
        } else {
            // This is the important part: it will be called for robot movement
            // *before* any button can use the arrow keys for focus traversal.
            handleRobotMovementKeyPress(event);
        }
    }


    private void handleRobotMovementKeyPress(KeyEvent event) {
        if (robot == null || isCreatingPath) {
            return;
        }

        double currentX = robot.getXInches();
        double currentY = robot.getYInches();
        double currentHeading_CCW = robot.getHeadingDegrees();
        boolean moved = false;

        double newFieldX = currentX;
        double newFieldY = currentY;
        double angleRad_CCW = Math.toRadians(currentHeading_CCW);

        switch (event.getCode()) {
            case UP:
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
                moved = true;
                break;
            case DOWN:
                newFieldX = currentX - ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
                newFieldY = currentY - ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
                moved = true;
                break;
            case LEFT:
                robot.setHeading(currentHeading_CCW + ROBOT_TURN_INCREMENT_DEGREES);
                moved = true;
                break;
            case RIGHT:
                robot.setHeading(currentHeading_CCW - ROBOT_TURN_INCREMENT_DEGREES);
                moved = true;
                break;
            case A:
                double strafeLeftAngleRad_CCW = Math.toRadians(currentHeading_CCW + 90.0);
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeLeftAngleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeLeftAngleRad_CCW);
                moved = true;
                break;
            case D:
                double strafeRightAngleRad_CCW = Math.toRadians(currentHeading_CCW - 90.0);
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeRightAngleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeRightAngleRad_CCW);
                moved = true;
                break;
            default:
                break;
        }

        if (moved) {
            if (event.getCode() == KeyCode.UP || event.getCode() == KeyCode.DOWN ||
                    event.getCode() == KeyCode.A || event.getCode() == KeyCode.D) {
                robot.setPosition(newFieldX, newFieldY);
            }
            updateUIFromRobotState();
            // This is critical: we consume the event so the button doesn't also process it.
            event.consume();
        }
    }

//    private void handleSceneKeyPress(KeyEvent event) {
//        if (isCreatingPath) {
//            // --- Keys active ONLY during path creation ---
//            if (event.getCode() == KeyCode.ESCAPE) {
//                System.out.println("Path Creation: ESC pressed.");
//                finishPathCreation(true); // true = cancel by ESC
//                event.consume(); // Consume the event if handled
//            }
//            // Add other path-creation specific key actions here if needed
//        } else {
//            System.out.println("Key Press - Attempting Robot Movement for: " + event.getCode());
//            // --- Keys active when NOT creating a path ---
//            if (event.getCode() == KeyCode.DELETE && !currentPath.isEmpty()) {
//                System.out.println("Path Management: DELETE pressed.");
//                deleteCurrentPath();
//                event.consume(); // Consume the event if handled
//            } else {
//                // If no specific non-path-creation key was handled above,
//                // try to handle it as robot movement.
//                // handleRobotMovementKeyPress itself checks if robot is null.
//                System.out.println("Key Press - Attempting Robot Movement for: " + event.getCode());
//                handleRobotMovementKeyPress(event);
//                // Don't consume here unless handleRobotMovementKeyPress explicitly handles and consumes it.
//                // If arrow keys are meant for other things too (like navigating controls),
//                // consumption needs careful thought. For now, assume it's for robot movement.
//            }
//        }
//    }

    private void startNewPathCreation() {
        if (isCreatingPath) return; // Already creating

        currentPath.clear();
        isCreatingPath = true;
        instructionLabel.setText("Click the first waypoint on the field. ESC to cancel, Double-click last point to finish.");
        // Pass both the point click listener AND the path finish listener
        fieldDisplay.setPathCreationMode(
                true,
                this::handleFieldClickForPath,  // For single clicks to add points
                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
        );
        controlPanel.setPathEditingActive(true);
        fieldDisplay.setPathToDraw(currentPath); // Show path as it's built
        fieldDisplay.drawCurrentState();
    }
}

//package com.example.ftcfieldsimulator; // Adjust package as needed
//
//import com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData;
//import com.example.ftcfieldsimulator.UdpPositionListener.PositionData;
//import com.example.ftcfieldsimulator.UdpPositionListener.CircleData;
//import com.example.ftcfieldsimulator.UdpPositionListener.LineData;
//import com.example.ftcfieldsimulator.UdpPositionListener.TextData;
//
//import javafx.application.Application;
//import javafx.application.Platform;
//import javafx.beans.value.ChangeListener;
//import javafx.geometry.Insets;
//import javafx.geometry.Point2D;
//import javafx.geometry.Pos;
//import javafx.scene.Scene;
//import javafx.scene.control.Label;
//import javafx.scene.input.KeyCode;
//import javafx.scene.input.KeyEvent;
//import javafx.scene.input.MouseButton; // For double click
//import javafx.scene.layout.BorderPane;
//import javafx.scene.layout.HBox; // For instruction panel
//import javafx.stage.FileChooser;
//import javafx.stage.Stage;
//import javafx.scene.paint.Color;
//
//import java.io.File;
//import java.io.PrintWriter;
//import java.util.ArrayList;
//import java.util.List;
//import java.util.Map;
//import java.util.HashMap;
//import java.util.concurrent.ConcurrentHashMap;
//
//public class FtcFieldSimulatorApp extends Application {
//
//    // --- Configuration Constants ---
//    public static final double FIELD_WIDTH_INCHES = 144.0; // 12 feet
//    public static final double FIELD_HEIGHT_INCHES = 144.0; // 12 feet
//
//    // Desired window size (can be adjusted, scaling will handle it)
//    private static final int FIELD_DISPLAY_WIDTH_PIXELS = 720; // This is now for the FieldDisplay itself
//    private static final int FIELD_DISPLAY_HEIGHT_PIXELS = 720;
//
//    // Path to your field image (REPLACE WITH YOUR ACTUAL PATH or put in resources)
//    // For simplicity now, let's assume it's next to the compiled classes or in a known relative path.
//    // A better way is to load from resources folder if using a build system like Maven/Gradle.
////    private static final String FIELD_IMAGE_PATH = "/field.png"; // e.g., src/main/resources/field.png
//    private static final String FIELD_IMAGE_PATH = "/decode_field.png"; // e.g., src/main/resources/field.png
//    private static final String ROBOT_IMAGE_PATH = "/robot.png"; // Add if Robot class doesn't define it itself
//    public static final double FIELD_IMAGE_ALPHA = 0.3;
//    public static final double BACKGROUND_ALPHA = 0.1;
//    private FieldDisplay fieldDisplay;
//    private ControlPanel controlPanel;
//    private Robot robot; // Add robot instance
//
//    private RecordingManager recordingManager;
//
//    // --- Robot Starting Configuration ---
//    // FieldX: Positive UP, FieldY: Positive LEFT
//    public static final double ROBOT_START_FIELD_X = 0.0;  // Initial position on Field X-axis (vertical, 0=center)
//    public static final double ROBOT_START_FIELD_Y = 0.0;  // Initial position on Field Y-axis (horizontal, 0=center)
//    // ANGLE: 0=UP, 90=LEFT (Robotics Convention). Let's start at 0 (pointing UP).
//    public static final double ROBOT_START_HEADING_DEGREES = 0.0;
//
//    private static final double ROBOT_MOVE_INCREMENT_INCHES = 2.0; // Move 2 inches per key press
//    private static final double ROBOT_TURN_INCREMENT_DEGREES = 5.0; // Turn 5 degrees per key press
//
//    private List<CurvePoint> initialPath; // We'll populate this later
//
//    private UdpPositionListener udpListener;
//    private Thread udpListenerThread;
//    private static final int UDP_LISTENER_PORT = 7777; // Example port
//
//    // --- Path Management ---
//    private List<CurvePoint> currentPath = new ArrayList<>();
//    private boolean isCreatingPath = false;
//    private Label instructionLabel; // For user instructions
//
//    // +++ Storage for Named Lines +++
//    // Using ConcurrentHashMap if FieldDisplay might iterate it from a non-FX thread,
//    // otherwise HashMap is fine if all access (add/iterate for drawing) is on FX thread.
//    // Since FieldDisplay.drawCurrentState() is called from Platform.runLater, HashMap is okay.
//    private Map<String, UdpPositionListener.LineData> namedLinesToDraw = new HashMap<>();
//    private final Object namedLinesLock = new Object(); // Lock for synchronizing access to namedLinesToDraw
//
//    public static void main(String[] args) {
//        Application.launch(args);
//    }
//
//    @Override
//    public void start(Stage primaryStage) {
//        primaryStage.setTitle("FTC Field Simulator");
//
//        // --- Calculate dimensions ---
//        double controlPanelWidth = FIELD_DISPLAY_WIDTH_PIXELS * ControlPanel.PREFERRED_WIDTH_RATIO_TO_FIELD;
//        double totalAppWidth = FIELD_DISPLAY_WIDTH_PIXELS + controlPanelWidth;
//        double totalAppHeight = FIELD_DISPLAY_HEIGHT_PIXELS; // Adjusted for instruction label later
//
//        // --- Define the 80% bounding box limits ---
//        double margin = FIELD_WIDTH_INCHES * 0.10; // 10% margin on each side for 80% coverage
//        double pathMinX = margin;
//        double pathMaxX = FIELD_WIDTH_INCHES - margin;
//        double pathMinY = margin;
//        double pathMaxY = FIELD_HEIGHT_INCHES - margin;
//
//        double pathWidth = pathMaxX - pathMinX;
//        double pathHeight = pathMaxY - pathMinY;
//
//        // --- Create Recording Manager ---
//        // We pass two method references:
//        // 1. this::handleUdpMessage: The method to call when replaying an event.
//        // 2. this::onPlaybackFinished: The method to call when playback stops on its own.
//        this.recordingManager = new RecordingManager(this::handleUdpMessage, this::onPlaybackFinished);
//
//        this.robot = new Robot(
//                ROBOT_START_FIELD_X,
//                ROBOT_START_FIELD_Y,
//                ROBOT_START_HEADING_DEGREES,
//                ROBOT_IMAGE_PATH
//        );
//
//        // Create the field display
//        fieldDisplay = new FieldDisplay(
//                FIELD_DISPLAY_WIDTH_PIXELS,
//                FIELD_DISPLAY_HEIGHT_PIXELS,
//                FIELD_WIDTH_INCHES,
//                FIELD_HEIGHT_INCHES,
//                FIELD_IMAGE_PATH,
//                robot,
//                BACKGROUND_ALPHA,
//                FIELD_IMAGE_ALPHA
//        );
////        fieldDisplay.setPath(initialPath); // Set path (even if empty)
//
//        // Pass the map of named lines to FieldDisplay
//        fieldDisplay.setNamedLinesMap(namedLinesToDraw);
//
//        // Example: Initial text or clear it
//        fieldDisplay.setRobotTextMessage(null); // Or some default like "Ready"
//
//        controlPanel = new ControlPanel(controlPanelWidth);
//
//        // --- Instruction Label ---
//        instructionLabel = new Label("Click 'New Path' to start drawing.");
//        instructionLabel.setPadding(new Insets(5));
//        instructionLabel.setMaxWidth(Double.MAX_VALUE);
//        instructionLabel.setAlignment(Pos.CENTER);
//        HBox instructionPane = new HBox(instructionLabel); // Put label in HBox for potential styling/centering
//        instructionPane.setAlignment(Pos.CENTER);
//        instructionPane.setStyle("-fx-background-color: #CFD8DC;"); // Light blue-gray background
//
//        // --- Setup Main Layout (BorderPane) ---
//        BorderPane mainLayout = new BorderPane();
//        mainLayout.setLeft(controlPanel);
//        mainLayout.setCenter(fieldDisplay);
//        mainLayout.setBottom(instructionPane); // Add instruction panel at the bottom
//
//
//        // --- Setup Scene ---
//        // Adjust totalAppHeight if instructionPane has significant height
//        double instructionPaneHeight = 30; // Estimate or calculate dynamically
//        Scene scene = new Scene(mainLayout, totalAppWidth, totalAppHeight + instructionPaneHeight);
//
//        // --- Event Handlers for ControlPanel Buttons ---
//        setupControlPanelActions(primaryStage);
//
//        // --- Set up actions for recording controls ---
//        setupRecordingControlActions();
//
//        // --- Event Handlers for FieldDisplay and Scene (Path Creation) ---
//        scene.setOnKeyPressed(this::handleSceneKeyPress); // For ESC to cancel path
//
//        // --- Start UDP Listener ---
//        startUdpListener();
//
//        primaryStage.setScene(scene);
//        primaryStage.setResizable(true);
//        primaryStage.show();
//        primaryStage.setOnCloseRequest(event -> stopApp());
//
//        primaryStage.setOnCloseRequest(event -> { // Ensure listener stops on window close
//            System.out.println("Window close requested. Stopping UDP listener.");
//            if (udpListener != null) {
//                udpListener.stopListener();
//            }
//            if (udpListenerThread != null && udpListenerThread.isAlive()) {
//                try {
//                    udpListenerThread.join(1000); // Wait for thread to die
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                    System.err.println("Interrupted while waiting for UDP listener thread to stop.");
//                }
//            }
//            System.out.println("Exiting application.");
//        });
//
//        // --- Initial UI Updates ---
//        updateUIFromRobotState(); // Call method to update status and draw
//    }
//
//    private void setupControlPanelActions(Stage ownerStage) {
//        controlPanel.setOnNewPathAction(event -> startNewPathCreation());
//        controlPanel.setOnDeletePathAction(event -> deleteCurrentPath());
//        controlPanel.setOnExportPathAction(event -> exportPathToCSV(ownerStage));
//        controlPanel.setOnClearTrailAction(event -> {
//            fieldDisplay.clearTrail();
//            fieldDisplay.drawCurrentState();
//            instructionLabel.setText("Robot trail cleared.");
//        });
//        controlPanel.setOnClearNamedLinesAction(event -> {
//            clearAllNamedLines();
//            instructionLabel.setText("All custom lines cleared.");
//        });
//        controlPanel.enablePathControls(!currentPath.isEmpty());
//        controlPanel.setTimelineSliderDisabled(true);
//    }
//
//    private void setupRecordingControlActions() {
//        // Action for the Record/Stop button
//        controlPanel.setOnRecordAction(() -> {
//            if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
//                // --- Stop Recording ---
//                recordingManager.stopRecording();
//                controlPanel.toggleRecordButtonIcon(false); // Show record icon
//
//                // Enable playback controls ONLY if a recording was successfully made
//                if (recordingManager.hasRecording()) {
//                    controlPanel.setPlaybackControlsDisabled(false);
//                    controlPanel.setTimelineSliderDisabled(false);
//                    controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
//                }
//
//            } else {
//                // --- Start Recording ---
//                recordingManager.startRecording();
//                controlPanel.toggleRecordButtonIcon(true); // Show stop icon
//                controlPanel.setPlaybackControlsDisabled(true); // Disable playback while recording
//                controlPanel.togglePlayPauseButtonIcon(false); // Ensure play icon is shown
//                controlPanel.setTimelineSliderDisabled(true);
//            }
//        });
//
//        // Action for the Play/Pause button
//        controlPanel.setOnPlayPauseAction(() -> {
//            RecordingManager.PlaybackState state = recordingManager.getCurrentState();
//
//            if (state == RecordingManager.PlaybackState.PLAYING) {
//                // --- Pause Playback ---
//                recordingManager.pause();
//                controlPanel.togglePlayPauseButtonIcon(false); // Show play icon
//
//            } else if (state == RecordingManager.PlaybackState.IDLE || state == RecordingManager.PlaybackState.PAUSED) {
//                // --- Start or Resume Playback ---
//                recordingManager.play();
//                controlPanel.togglePlayPauseButtonIcon(true); // Show pause icon
//            }
//        });
//
//        // --- FIX: COMBINED EVENT HANDLING FOR THE SLIDER ---
//
//        // This listener handles the LIVE update while the user is DRAGGING the slider.
//        controlPanel.setOnTimelineSliderChanged((observable, oldValue, newValue) -> {
//            if (controlPanel.getTimelineSlider().isValueChanging()) {
//                recordingManager.seekTo(newValue.intValue());
//                updateUIFromRobotState();
//            }
//        });
//
//        // This NEW listener handles the final update after a CLICK or at the END of a drag.
//        controlPanel.setOnSliderMouseReleased(event -> {
//            // We get the slider's final value and tell the manager to seek to that event index.
//            // This robustly handles both clicks and the end of a drag action.
//            int seekIndex = (int) controlPanel.getTimelineSlider().getValue();
//            recordingManager.seekTo(seekIndex);
//            updateUIFromRobotState();
//        });
//    }
//
//    // +++ Callback for when playback is finished or stopped +++
//    private void onPlaybackFinished() {
//        // This is called by the RecordingManager on the FX thread.
//        // It ensures the UI is reset correctly.
//        Platform.runLater(() -> {
//            controlPanel.togglePlayPauseButtonIcon(false); // Reset to show the "play" icon
//            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
//        });
//    }
//
//    private void clearAllNamedLines() {
//        synchronized (namedLinesLock) {
//            namedLinesToDraw.clear();
//        }
//        if (fieldDisplay != null) {
//            fieldDisplay.drawCurrentState(); // Redraw to reflect the cleared lines
//        }
//    }
//
//    private void handleSceneKeyPress(KeyEvent event) {
//        if (isCreatingPath) {
//            // --- Keys active ONLY during path creation ---
//            if (event.getCode() == KeyCode.ESCAPE) {
//                System.out.println("Path Creation: ESC pressed.");
//                finishPathCreation(true); // true = cancel by ESC
//                event.consume(); // Consume the event if handled
//            }
//            // Add other path-creation specific key actions here if needed
//        } else {
//            System.out.println("Key Press - Attempting Robot Movement for: " + event.getCode());
//            // --- Keys active when NOT creating a path ---
//            if (event.getCode() == KeyCode.DELETE && !currentPath.isEmpty()) {
//                System.out.println("Path Management: DELETE pressed.");
//                deleteCurrentPath();
//                event.consume(); // Consume the event if handled
//            } else {
//                // If no specific non-path-creation key was handled above,
//                // try to handle it as robot movement.
//                // handleRobotMovementKeyPress itself checks if robot is null.
//                System.out.println("Key Press - Attempting Robot Movement for: " + event.getCode());
//                handleRobotMovementKeyPress(event);
//                // Don't consume here unless handleRobotMovementKeyPress explicitly handles and consumes it.
//                // If arrow keys are meant for other things too (like navigating controls),
//                // consumption needs careful thought. For now, assume it's for robot movement.
//            }
//        }
//    }
//
//    private void startNewPathCreation() {
//        if (isCreatingPath) return; // Already creating
//
//        currentPath.clear();
//        isCreatingPath = true;
//        instructionLabel.setText("Click the first waypoint on the field. ESC to cancel, Double-click last point to finish.");
//        // Pass both the point click listener AND the path finish listener
//        fieldDisplay.setPathCreationMode(
//                true,
//                this::handleFieldClickForPath,  // For single clicks to add points
//                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
//        );
//        controlPanel.setPathEditingActive(true);
//        fieldDisplay.setPathToDraw(currentPath); // Show path as it's built
//        fieldDisplay.drawCurrentState();
//    }
//
//    private void handleFieldClickForPath(Point2D pixelCoords) { // pixelCoords are from top-left of canvas
//        if (!isCreatingPath) return;
//
////        Point2D inchesCoords = fieldDisplay.pixelToInches(pixelCoords.getX(), pixelCoords.getY());
//        Point2D inchesCoordsFieldCenter = fieldDisplay.pixelToInches(pixelCoords.getX(), pixelCoords.getY());
//        // Optional: Prevent adding a point if it's too close to the previous one on double-click
//        // if (isDoubleClick && !currentPath.isEmpty()) {
//        //     CurvePoint lastPoint = currentPath.get(currentPath.size() - 1);
//        //     if (Math.hypot(inchesCoords.getX() - lastPoint.x, inchesCoords.getY() - lastPoint.y) < SOME_THRESHOLD) {
//        //          // It's a double click very near the last point, don't add it again.
//        //          // The finishPathCreation will be triggered by the separate onPathFinishListener.
//        //          return;
//        //     }
//        // }
//        currentPath.add(new CurvePoint(inchesCoordsFieldCenter.getX(), inchesCoordsFieldCenter.getY()));
//        fieldDisplay.setPathToDraw(currentPath); // Update the path being drawn
//        fieldDisplay.drawCurrentState();
//
//        if (currentPath.size() == 1) {
//            instructionLabel.setText("Click the next waypoint. ESC to cancel.");
//        } else {
//            instructionLabel.setText("Click next waypoint, Double-click last point, or ESC to cancel.");
//        }
//
//        // Check for double click to finish path
//        // FieldDisplay mouse event handler needs to be more specific for this
//        // For simplicity now, let's rely on ESC or a dedicated "Finish Path" button later.
//        // Alternative: The passed MouseEvent in FieldDisplay could check clickCount.
//        // For now, focusing on ESC / explicit finish.
//    }
//
//    // Call this when path creation is done (e.g., ESC, Double Click, or a "Finish" button)
//    private void finishPathCreation(boolean cancelled) {
//        if (!isCreatingPath) return;
//
//        System.out.println("Finishing path creation. Cancelled: " + cancelled + ". Current isCreatingPath: " + isCreatingPath);
//        isCreatingPath = false;
//        System.out.println("Set isCreatingPath to: " + isCreatingPath);
//        fieldDisplay.setPathCreationMode(
//                false,
//                null,  // For single clicks to add points
//                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
//        );
//        controlPanel.setPathEditingActive(false);
//
//        if (cancelled) {
//            currentPath.clear();
//            instructionLabel.setText("Path creation cancelled. Click 'New Path' to start again.");
//        } else if (currentPath.isEmpty()) {
//            instructionLabel.setText("Path is empty. Click 'New Path' to start again.");
//        } else {
//            instructionLabel.setText("Path finished with " + currentPath.size() + " points. You can now Export or Delete it.");
//        }
//        fieldDisplay.setPathToDraw(currentPath);
//        fieldDisplay.drawCurrentState();
//        controlPanel.enablePathControls(!currentPath.isEmpty());
//    }
//
//    private void deleteCurrentPath() {
//        currentPath.clear();
//        isCreatingPath = false; // Ensure path creation mode is exited
//        fieldDisplay.setPathCreationMode(
//                false,
//                null,  // For single clicks to add points
//                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
//        );
//        fieldDisplay.setPathToDraw(currentPath);
//        fieldDisplay.drawCurrentState();
//        instructionLabel.setText("Path deleted. Click 'New Path' to start drawing.");
//        controlPanel.enablePathControls(false);
//        controlPanel.setPathEditingActive(false); // Reset path editing buttons
//    }
//
//    private void exportPathToCSV(Stage ownerStage) {
//        if (currentPath.isEmpty()) {
//            instructionLabel.setText("No path to export.");
//            return;
//        }
//
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Save Path to CSV");
//        fileChooser.setInitialFileName("ftc_path.csv");
//        fileChooser.getExtensionFilters().add(
//                new FileChooser.ExtensionFilter("CSV Files (*.csv)", "*.csv")
//        );
//
//        File file = fileChooser.showSaveDialog(ownerStage);
//        if (file != null) {
//            try (PrintWriter writer = new PrintWriter(file)) {
//                writer.println("x_inches,y_inches"); // CSV Header
//                for (CurvePoint point : currentPath) {
//                    writer.printf("%.3f,%.3f\n", point.x, point.y);
//                }
//                instructionLabel.setText("Path exported successfully to " + file.getName());
//            } catch (Exception e) {
//                instructionLabel.setText("Error exporting path: " + e.getMessage());
//                e.printStackTrace();
//            }
//        } else {
//            instructionLabel.setText("Path export cancelled.");
//        }
//    }
//
//
//    private void handleUdpMessage(UdpMessageData messageData) {
//        if (messageData == null) return;
//
//        // --- FIX: This block was missing. It adds the live data to the recording. ---
//        if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
//            recordingManager.addEvent(messageData);
//        }
//        // --- End of the fix ---
//
//        // This block updates the slider's visual position during playback.
//        if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.PLAYING ||
//                recordingManager.getCurrentState() == RecordingManager.PlaybackState.PAUSED) {
//            Platform.runLater(() ->
//                    controlPanel.updateTimelineSlider(
//                            recordingManager.getPlaybackIndex(),
//                            recordingManager.getTotalEvents()
//                    )
//            );
//        }
//
//        // The rest of the method processes the message for LIVE display or for PLAYBACK.
//        Platform.runLater(() -> { // Ensure all UI updates are on the JavaFX Application Thread
//            if (messageData instanceof PositionData) {
//                PositionData pose = (PositionData) messageData;
//                if (robot != null && fieldDisplay != null) {
//                    // For live display, add a trail dot at the robot's previous position before moving it.
//                    // During playback, this recreates the trail.
//                    fieldDisplay.addTrailDot(robot.getXInches(), robot.getYInches());
//                    robot.setPosition(pose.x, pose.y);
//                    robot.setHeading(pose.heading);
//                }
//            } else if (messageData instanceof CircleData) {
//                CircleData circle = (CircleData) messageData;
//                if (fieldDisplay != null) {
//                    fieldDisplay.addDebugCircle(
//                            robot.getXInches(),    // Center X at current robot X
//                            robot.getYInches(),    // Center Y at current robot Y
//                            circle.radiusInches,
//                            circle.heading,        // <<< PASS THE HEADING
//                            Color.rgb(255, 165, 0, 0.7) // Orange with some transparency
//                    );
//                }
//            } else if (messageData instanceof LineData) {
//                LineData line = (LineData) messageData;
//                synchronized (namedLinesLock) {
//                    namedLinesToDraw.put(line.name, line); // Add or replace line by its name
//                }
//            } else if (messageData instanceof TextData) {
//                TextData text = (TextData) messageData;
//                if (fieldDisplay != null) {
//                    fieldDisplay.setRobotTextMessage(text.text);
//                }
//            }
//
//            // Always redraw after processing any message that might change the display
//            if (fieldDisplay != null) {
//                if (messageData instanceof PositionData) {
//                    // This updates the status panel (X, Y, Heading) AND redraws the field
//                    updateUIFromRobotState();
//                } else {
//                    // For other shapes, just redraw the field without updating the status panel
//                    fieldDisplay.drawCurrentState();
//                }
//            }
//        });
//    }
//
//    private void startUdpListener() {
//        try {
//            udpListener = new UdpPositionListener(UDP_LISTENER_PORT, this::handleUdpMessage);
//            udpListenerThread = new Thread(udpListener, "UdpListenerThread");
//            udpListenerThread.setDaemon(true);
//            udpListenerThread.start();
//            System.out.println("UDP Listener started on port " + UDP_LISTENER_PORT);
//        } catch (Exception e) {
//            System.err.println("Error starting UDP listener: " + e.getMessage());
//            e.printStackTrace();
//            instructionLabel.setText("ERROR: Could not start UDP listener on port " + UDP_LISTENER_PORT);
//        }
//    }
//
//    private void handleRobotMovementKeyPress(KeyEvent event) {
//        // System.out.println("handleRobotMovementKeyPress checking for: " + event.getCode()); // DEBUG
//        if (robot == null || isCreatingPath) {
//            if (isCreatingPath) System.out.println("Robot movement skipped: isCreatingPath is true.");
//            if (robot == null) System.out.println("Robot movement skipped: robot is null.");
//            return;
//        }
//
//        double currentX = robot.getXInches(); // Current Field X (vertical)
//        double currentY = robot.getYInches(); // Current Field Y (horizontal)
//        double currentHeading_CCW = robot.getHeadingDegrees(); // 0=UP, 90=LEFT, CCW positive
//        boolean moved = false;
//
//        // Declare newFieldX and newFieldY here, to be assigned in the switch cases
//        double newFieldX = currentX; // Initialize with current position
//        double newFieldY = currentY; // Initialize with current position
//
//        double angleRad_CCW = Math.toRadians(currentHeading_CCW);
//
//        switch (event.getCode()) {
//            case UP: // Move "forward" along the robot's current heading
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
//                moved = true;
//                break;
//            case DOWN: // Move "backward"
//                newFieldX = currentX - ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
//                newFieldY = currentY - ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
//                moved = true;
//                break;
//            case LEFT: // Turn left (increase heading in CCW system)
//                robot.setHeading(currentHeading_CCW + ROBOT_TURN_INCREMENT_DEGREES);
//                moved = true; // Still counts as a move for UI update
//                break;
//            case RIGHT: // Turn right (decrease heading in CCW system)
//                robot.setHeading(currentHeading_CCW - ROBOT_TURN_INCREMENT_DEGREES);
//                moved = true; // Still counts as a move for UI update
//                break;
//            case A: // Strafe Left (move at heading + 90 degrees)
//                double strafeLeftAngleRad_CCW = Math.toRadians(currentHeading_CCW + 90.0);
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeLeftAngleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeLeftAngleRad_CCW);
//                moved = true;
//                break;
//            case D: // Strafe Right (move at heading - 90 degrees)
//                double strafeRightAngleRad_CCW = Math.toRadians(currentHeading_CCW - 90.0);
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeRightAngleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeRightAngleRad_CCW);
//                moved = true;
//                break;
//            default:
//                // Do nothing for other keys
//                break;
//        }
//
//        if (moved) {
//            // Set the robot's position only if it's a translational move (UP, DOWN, A, D)
//            // For turns (LEFT, RIGHT), only the heading was changed.
//            // We could also call setPosition for turns with currentX/Y, but it's redundant.
//            if (event.getCode() == KeyCode.UP || event.getCode() == KeyCode.DOWN ||
//                    event.getCode() == KeyCode.A || event.getCode() == KeyCode.D) {
//                robot.setPosition(newFieldX, newFieldY);
//            }
//
//            updateUIFromRobotState(); // This will typically redraw the field display
//            event.consume();      // Consume the event if it resulted in robot movement
//        }
//    }
//
//    private void updateUIFromRobotState() {
//        if (robot != null && controlPanel != null && fieldDisplay != null) {
//            // Normalize heading to 0-359.9 degrees for display if desired
//            double displayHeading = robot.getHeadingDegrees() % 360;
//            if (displayHeading < 0) displayHeading += 360;
//
//            controlPanel.updateRobotStatus(robot.getXInches(), robot.getYInches(), displayHeading);
//            fieldDisplay.drawCurrentState();
//        }
//    }
//
//    private void stopApp() {
//        System.out.println("Window close requested / App stopping. Stopping UDP listener.");
//        if (udpListener != null) {
//            udpListener.stopListener();
//        }
//        if (udpListenerThread != null && udpListenerThread.isAlive()) {
//            try {
//                udpListenerThread.join(1000);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//                System.err.println("Interrupted while waiting for UDP listener thread to stop.");
//            }
//        }
//        System.out.println("Exiting application.");
//        Platform.exit(); // Ensure JavaFX platform exits cleanly
//        System.exit(0); // Force exit if needed
//    }
//
//    // Make sure to stop the listener when the application exits
//    @Override
//    public void stop() throws Exception {
//        stopApp(); // Call our common stop logic
//        super.stop(); // Call superclass method
//    }
//
//    // --- Call updateUIFromRobotState() whenever the robot's state changes ---
//    // Example method that changes robot state - this would be called by actual controls
//    public void moveRobotProgrammatically(double newX, double newY, double newHeading) {
//        if (robot != null) {
//            robot.setPosition(newX, newY);
//            robot.setHeading(newHeading);
//            updateUIFromRobotState();
//        }
//    }
//    private List<CurvePoint> createDefaultSPath() {
//        List<CurvePoint> path = new ArrayList<>();
//        double margin = FIELD_WIDTH_INCHES * 0.10;
//        double pathMinX = margin;
//        double pathMaxX = FIELD_WIDTH_INCHES - margin;
//        double pathMinY = margin;
//        double pathMaxY = FIELD_HEIGHT_INCHES - margin;
//        double pathWidth = pathMaxX - pathMinX;
//        double pathHeight = pathMaxY - pathMinY;
//
//        path.add(new CurvePoint(pathMinX, pathMinY));
//        path.add(new CurvePoint(pathMaxX, pathMinY + pathHeight * 0.25));
//        path.add(new CurvePoint(pathMinX, pathMinY + pathHeight * 0.50));
//        path.add(new CurvePoint(pathMaxX, pathMinY + pathHeight * 0.75));
//        path.add(new CurvePoint(pathMinX, pathMaxY));
//        path.add(new CurvePoint(pathMinX + pathWidth * 0.25, pathMaxY));
//        return path;
//    }
//    private Robot createInitialRobot(List<CurvePoint> path) {
//        if (path != null && !path.isEmpty()) {
//            return new Robot(path.get(0).x, path.get(0).y, 0.0, ROBOT_IMAGE_PATH);
//        } else {
//            return new Robot(
//                    FIELD_WIDTH_INCHES / 2.0,
//                    FIELD_HEIGHT_INCHES / 2.0,
//                    0.0,
//                    ROBOT_IMAGE_PATH
//            );
//        }
//    }
//}
