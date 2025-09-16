package com.example.ftcfieldsimulator;

import java.nio.file.Paths;

import javafx.beans.value.ChangeListener;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

public class ControlPanel extends VBox {
    public static final double PREFERRED_WIDTH_RATIO_TO_FIELD = 1.0 / 3.0; // 1/3 of field width

    // --- UI Elements ---
    private Label xPosLabel, yPosLabel, headingLabel, pathStatusLabel;
    private Button newPathButton, deletePathButton, exportPathButton, clearTrailButton, clearNamedLinesButton;
    private Button recordButton, playPauseButton, reverseButton, forwardButton;
    // +++ NEW: Open and Save buttons +++
    private Button openButton, saveButton;
    private ImageView recordIcon, stopIcon, playIcon, pauseIcon, reverseIcon, forwardIcon;
    private Slider timelineSlider;

    public ControlPanel(double preferredWidth) {
        super(10);
        setPadding(new Insets(15));
        setPrefWidth(preferredWidth);
        setStyle("-fx-background-color: #ECEFF1;");

        // --- Main Title ---
        Label controlsTitle = new Label("Controls");
        controlsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 20));
        controlsTitle.setTextFill(Color.DARKSLATEBLUE);
        controlsTitle.setMaxWidth(Double.MAX_VALUE);
        controlsTitle.setAlignment(Pos.CENTER);
        controlsTitle.setPadding(new Insets(0, 0, 10, 0));

        // ... [Path and Utility sections remain the same] ...
        Label pathTitle = new Label("Path Management");
        pathTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        newPathButton = createMaxWidthButton("New Path");
        deletePathButton = createMaxWidthButton("Delete Path");
        exportPathButton = createMaxWidthButton("Export Path (CSV)");
        pathStatusLabel = new Label("No path created.");
        pathStatusLabel.setStyle("-fx-text-fill: #546E7A;");
        VBox pathControlsBox = new VBox(8, pathTitle, newPathButton, deletePathButton, exportPathButton, pathStatusLabel);
        Label utilityTitle = new Label("Utilities");
        utilityTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        clearTrailButton = createMaxWidthButton("Clear Robot Trail");
        clearNamedLinesButton = createMaxWidthButton("Clear Custom Lines");
        VBox utilityControlsBox = new VBox(8, utilityTitle, clearTrailButton, clearNamedLinesButton);


        // --- Recording Controls Section ---
        loadPlaybackIcons();
        Label recordingTitle = new Label("Recording");
        recordingTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));

        // +++ NEW: HBox for Open/Save buttons +++
        openButton = createMaxWidthButton("Open");
        saveButton = createMaxWidthButton("Save");
        saveButton.setDisable(true); // Disabled by default
        HBox fileButtons = new HBox(10, openButton, saveButton);
        HBox.setHgrow(openButton, Priority.ALWAYS);
        HBox.setHgrow(saveButton, Priority.ALWAYS);

        // Playback buttons
        recordButton = new Button();
        recordButton.setGraphic(recordIcon);
        playPauseButton = new Button();
        playPauseButton.setGraphic(playIcon);
        reverseButton = new Button();
        reverseButton.setGraphic(reverseIcon);
        forwardButton = new Button();
        forwardButton.setGraphic(forwardIcon);
        HBox recordingButtons = new HBox(10, reverseButton, playPauseButton, forwardButton, recordButton);
        recordingButtons.setAlignment(Pos.CENTER);

        timelineSlider = new Slider(0, 100, 0);
        timelineSlider.setDisable(true);

        // +++ Add the new fileButtons HBox to the VBox +++
        VBox recordingControlsBox = new VBox(8, recordingTitle, fileButtons, recordingButtons, timelineSlider);
        setPlaybackControlsDisabled(true);

        // ... [Rest of the constructor and methods remain the same] ...
        VBox spacer = new VBox();
        VBox.setVgrow(spacer, Priority.ALWAYS);
        Label statusTitle = new Label("Robot Status");
        statusTitle.setFont(Font.font("Arial", FontWeight.BOLD, 18));
        statusTitle.setTextFill(Color.DARKSLATEBLUE);
        statusTitle.setMaxWidth(Double.MAX_VALUE);
        statusTitle.setAlignment(Pos.CENTER);
        statusTitle.setPadding(new Insets(10, 0, 5, 0));
        Font valueFont = Font.font("Consolas", 14);
        xPosLabel = createStatusLabel("X Position:", valueFont);
        yPosLabel = createStatusLabel("Y Position:", valueFont);
        headingLabel = createStatusLabel("Heading:", valueFont);
        VBox statusInfoBox = new VBox(5);
        statusInfoBox.setPadding(new Insets(5, 10, 10, 10));
        statusInfoBox.setStyle("-fx-border-color: #B0BEC5; -fx-border-width: 1; -fx-border-radius: 5;");
        statusInfoBox.getChildren().addAll(xPosLabel.getParent(), yPosLabel.getParent(), headingLabel.getParent());
        this.getChildren().addAll(controlsTitle, pathControlsBox, utilityControlsBox, recordingControlsBox, spacer, statusTitle, statusInfoBox);
    }

    private Button createMaxWidthButton(String text) {
        Button button = new Button(text);
        button.setMaxWidth(Double.MAX_VALUE);
        return button;
    }

    private Label createStatusLabel(String labelText, Font valueFont) {
        Label label = new Label(labelText);
        label.setStyle("-fx-text-fill: #37474F;");
        Label valueLabel = new Label("0.0");
        valueLabel.setFont(valueFont);
        valueLabel.setStyle("-fx-text-fill: #000000;");
        HBox hbox = new HBox(5, label, valueLabel);
        hbox.setAlignment(Pos.CENTER_LEFT);
        return valueLabel;
    }

    private void loadPlaybackIcons() {
        double iconSize = 20;
        try {
            recordIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/record.png"), iconSize, iconSize, true, true));
            stopIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/stop.png"), iconSize, iconSize, true, true));
            playIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/play.png"), iconSize, iconSize, true, true));
            pauseIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/pause.png"), iconSize, iconSize, true, true));
            reverseIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/reverse.png"), iconSize, iconSize, true, true));
            forwardIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/forward.png"), iconSize, iconSize, true, true));
        } catch (Exception e) {
            System.err.println("Error loading playback icons.");
        }
    }

    // --- PUBLIC METHODS ---

    // +++ NEW: Methods to control the Open/Save buttons +++
    public void setOnOpenAction(EventHandler<ActionEvent> handler) { openButton.setOnAction(handler); }
    public void setOnSaveAction(EventHandler<ActionEvent> handler) { saveButton.setOnAction(handler); }
    public void setSaveButtonDisabled(boolean isDisabled) { saveButton.setDisable(isDisabled); }

    // ... [Other public methods are unchanged] ...
    public void updateRobotStatus(double x, double y, double h) { xPosLabel.setText(String.format("%.2f", x)); yPosLabel.setText(String.format("%.2f", y)); headingLabel.setText(String.format("%.2f°", h)); }
    public void setOnNewPathAction(EventHandler<ActionEvent> handler) { newPathButton.setOnAction(handler); }
    public void setOnDeletePathAction(EventHandler<ActionEvent> handler) { deletePathButton.setOnAction(handler); }
    public void setOnExportPathAction(EventHandler<ActionEvent> handler) { exportPathButton.setOnAction(handler); }
    public void setOnClearTrailAction(EventHandler<ActionEvent> handler) { clearTrailButton.setOnAction(handler); }
    public void setOnClearNamedLinesAction(EventHandler<ActionEvent> handler) { clearNamedLinesButton.setOnAction(handler); }
    public void setOnRecordAction(Runnable action) { recordButton.setOnAction(e -> action.run()); }
    public void setOnPlayPauseAction(Runnable action) { playPauseButton.setOnAction(e -> action.run()); }
    public void setOnTimelineSliderChanged(ChangeListener<Number> listener) { timelineSlider.valueProperty().addListener(listener); }
    public void setOnSliderMouseReleased(EventHandler<MouseEvent> handler) { timelineSlider.setOnMouseReleased(handler); }
    public void setPathEditingActive(boolean isActive) { newPathButton.setDisable(isActive); deletePathButton.setDisable(isActive); exportPathButton.setDisable(isActive); }
    public void updateTimelineSlider(int currentEventIndex, int totalEvents) { if (totalEvents > 1) { timelineSlider.setMax(totalEvents - 1); timelineSlider.setValue(currentEventIndex); } else { timelineSlider.setMax(100); timelineSlider.setValue(0); } }
    public void setTimelineSliderDisabled(boolean isDisabled) { timelineSlider.setDisable(isDisabled); }
    public void toggleRecordButtonIcon(boolean isRecording) { recordButton.setGraphic(isRecording ? stopIcon : recordIcon); }
    public void togglePlayPauseButtonIcon(boolean isPlaying) { playPauseButton.setGraphic(isPlaying ? pauseIcon : playIcon); }
    public void setPlaybackControlsDisabled(boolean isDisabled) { playPauseButton.setDisable(isDisabled); reverseButton.setDisable(isDisabled); forwardButton.setDisable(isDisabled); }
    public void enablePathControls(boolean pathExists) { deletePathButton.setDisable(!pathExists); exportPathButton.setDisable(!pathExists); }
    public void setPathStatus(String text) { pathStatusLabel.setText(text); }
    public Slider getTimelineSlider() { return this.timelineSlider; }
}


//package com.example.ftcfieldsimulator;
//
//import javafx.event.ActionEvent;
//import javafx.event.EventHandler;
//import javafx.geometry.Insets;
//import javafx.geometry.Pos;
//import javafx.scene.control.Button;
//import javafx.scene.control.Label;
//import javafx.scene.layout.HBox;
//import javafx.scene.layout.Priority;
//import javafx.scene.image.Image;
//import javafx.scene.image.ImageView;
//import javafx.scene.layout.VBox;
//import javafx.scene.paint.Color;
//import javafx.scene.text.Font;
//import javafx.scene.text.FontWeight;
//import javafx.beans.value.ChangeListener;
//import javafx.scene.control.Slider;
//import java.util.function.Consumer;
//import javafx.scene.input.MouseEvent;
//
//public class ControlPanel extends VBox {
//
//    public static final double PREFERRED_WIDTH_RATIO_TO_FIELD = 1.0 / 3.0; // 1/3 of field width
//
//    // Labels for status display
////    private Label robotXLabel;
////    private Label robotYLabel;
////    private Label robotHeadingLabel;
//    private Label xPosLabel, yPosLabel, headingLabel;
//    private Button newPathButton, deletePathButton, exportPathButton, clearTrailButton, clearNamedLinesButton;
//    private Label pathStatusLabel;
//
//    private Button recordButton, playPauseButton, reverseButton, forwardButton;
//    private ImageView recordIcon, stopIcon, playIcon, pauseIcon, reverseIcon, forwardIcon;
//    private Slider timelineSlider;
//
////    // Labels for the actual values (to be updated)
////    private Label robotXValueLabel;
////    private Label robotYValueLabel;
////    private Label robotHeadingValueLabel;
////
////    private Button newPathButton;
////    private Button deletePathButton;
////    private Button exportPathButton;
////    private Button clearTrailButton;
////    private Button clearNamedLinesButton;
////
////    private Button recordButton, playPauseButton, reverseButton, forwardButton;
////    private ImageView recordIcon, stopIcon, playIcon, pauseIcon, reverseIcon, forwardIcon;
//public ControlPanel(double preferredWidth) {
//    super(10); // Spacing between vertical elements
//    setPadding(new Insets(15));
//    setPrefWidth(preferredWidth);
//    setStyle("-fx-background-color: #ECEFF1;"); // Reverted to light gray background
//
//    // --- Main Title ---
//    Label controlsTitle = new Label("Controls");
//    controlsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 20));
//    controlsTitle.setTextFill(Color.DARKSLATEBLUE);
//    controlsTitle.setMaxWidth(Double.MAX_VALUE);
//    controlsTitle.setAlignment(Pos.CENTER);
//    controlsTitle.setPadding(new Insets(0, 0, 10, 0));
//
//    // --- Path Controls Section ---
//    Label pathTitle = new Label("Path Management");
//    pathTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
//    // +++ FIX: Renamed method call +++
//    newPathButton = createMaxWidthButton("New Path");
//    deletePathButton = createMaxWidthButton("Delete Path");
//    exportPathButton = createMaxWidthButton("Export Path (CSV)");
//    pathStatusLabel = new Label("No path created.");
//    pathStatusLabel.setStyle("-fx-text-fill: #546E7A;");
//    VBox pathControlsBox = new VBox(8, pathTitle, newPathButton, deletePathButton, exportPathButton, pathStatusLabel);
//
//    // --- Utility Controls Section ---
//    Label utilityTitle = new Label("Utilities");
//    utilityTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
//    // +++ FIX: Renamed method call +++
//    clearTrailButton = createMaxWidthButton("Clear Robot Trail");
//    clearNamedLinesButton = createMaxWidthButton("Clear Custom Lines");
//    VBox utilityControlsBox = new VBox(8, utilityTitle, clearTrailButton, clearNamedLinesButton);
//
//    // --- Recording Controls Section ---
//    loadPlaybackIcons(); // Load icons from resources
//    Label recordingTitle = new Label("Recording");
//    recordingTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
//    recordButton = new Button();
//    recordButton.setGraphic(recordIcon);
//    playPauseButton = new Button();
//    playPauseButton.setGraphic(playIcon);
//    reverseButton = new Button();
//    reverseButton.setGraphic(reverseIcon);
//    forwardButton = new Button();
//    forwardButton.setGraphic(forwardIcon);
//    HBox recordingButtons = new HBox(10, reverseButton, playPauseButton, forwardButton, recordButton);
//    recordingButtons.setAlignment(Pos.CENTER);
//    timelineSlider = new Slider(0, 100, 0); // min, max, initial
//    timelineSlider.setDisable(true);
//    VBox recordingControlsBox = new VBox(8, recordingTitle, recordingButtons, timelineSlider);
//    setPlaybackControlsDisabled(true); // Disable buttons initially
//
//    // --- Spacer to push status to the bottom ---
//    VBox spacer = new VBox();
//    VBox.setVgrow(spacer, Priority.ALWAYS);
//
//    // --- Robot Status Section (at the bottom) ---
//    Label statusTitle = new Label("Robot Status");
//    statusTitle.setFont(Font.font("Arial", FontWeight.BOLD, 18));
//    statusTitle.setTextFill(Color.DARKSLATEBLUE);
//    statusTitle.setMaxWidth(Double.MAX_VALUE);
//    statusTitle.setAlignment(Pos.CENTER);
//    statusTitle.setPadding(new Insets(10, 0, 5, 0));
//
//    Font valueFont = Font.font("Consolas", 14); // Monospaced font
//    xPosLabel = createStatusLabel("X Position:", valueFont);
//    yPosLabel = createStatusLabel("Y Position:", valueFont);
//    headingLabel = createStatusLabel("Heading:", valueFont);
//
//    VBox statusInfoBox = new VBox(5);
//    statusInfoBox.setPadding(new Insets(5, 10, 10, 10));
//    statusInfoBox.setStyle("-fx-border-color: #B0BEC5; -fx-border-width: 1; -fx-border-radius: 5;");
//    statusInfoBox.getChildren().addAll(xPosLabel.getParent(), yPosLabel.getParent(), headingLabel.getParent());
//
//    // --- Add all sections to the main VBox layout ---
//    this.getChildren().addAll(
//            controlsTitle,
//            pathControlsBox,
//            utilityControlsBox,
//            recordingControlsBox,
//            spacer,
//            statusTitle,
//            statusInfoBox
//    );
//}
//
//    // Helper method to create a button that fills the width
//    // +++ FIX: Renamed method definition +++
//    private Button createMaxWidthButton(String text) {
//        Button button = new Button(text);
//        button.setMaxWidth(Double.MAX_VALUE);
//        return button;
//    }
//
//    // Helper method to create a single status line (Label + Value)
//    private Label createStatusLabel(String labelText, Font valueFont) {
//        Label label = new Label(labelText);
//        label.setStyle("-fx-text-fill: #37474F;"); // Darker text for light background
//        Label valueLabel = new Label("0.0");
//        valueLabel.setFont(valueFont);
//        valueLabel.setStyle("-fx-text-fill: #000000;"); // Black text for value
//        HBox hbox = new HBox(5, label, valueLabel);
//        hbox.setAlignment(Pos.CENTER_LEFT);
//        return valueLabel;
//    }
////
////    // Method to load all playback icons
////    private void loadPlaybackIcons() {
////        double iconSize = 20;
////        try {
////            recordIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/record.png"), iconSize, iconSize, true, true));
////            stopIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/stop.png"), iconSize, iconSize, true, true));
////            playIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/play.png"), iconSize, iconSize, true, true));
////            pauseIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/pause.png"), iconSize, iconSize, true, true));
////            reverseIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/reverse.png"), iconSize, iconSize, true, true));
////            forwardIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/forward.png"), iconSize, iconSize, true, true));
////        } catch (Exception e) {
////            System.err.println("Error loading playback icons. Make sure they are in 'src/main/resources/icons/'");
////            recordIcon = new ImageView();
////            stopIcon = new ImageView();
////            playIcon = new ImageView();
////            pauseIcon = new ImageView();
////            reverseIcon = new ImageView();
////            forwardIcon = new ImageView();
////        }
////    }
//
////    public ControlPanel(double preferredWidth) {
////        super(10); // Spacing between elements in the VBox
////        this.setPadding(new Insets(10));
////
////        this.setStyle("-fx-background-color: #37474F;"); // A dark theme background
////        this.setPrefWidth(preferredWidth);
////        this.setAlignment(Pos.TOP_CENTER);
////
////        Font labelFont = Font.font("Arial", FontWeight.BOLD, 14);
////        Font valueFont = Font.font("Consolas", 14); // Monospaced for stable layout
////
////        // --- Robot Status ---
////        Label statusTitle = new Label("ROBOT STATUS");
////        statusTitle.setFont(labelFont);
////        statusTitle.setStyle("-fx-text-fill: #ECEFF1;");
////
////        xPosLabel = createStatusLabel("X Position:", valueFont);
////        yPosLabel = createStatusLabel("Y Position:", valueFont);
////        headingLabel = createStatusLabel("Heading:", valueFont);
////
////        // --- Path Controls ---
////        Label pathTitle = new Label("PATH CONTROLS");
////        pathTitle.setFont(labelFont);
////        pathTitle.setStyle("-fx-text-fill: #ECEFF1;");
////
////        newPathButton = new Button("New Path");
////        deletePathButton = new Button("Delete Path");
////        exportPathButton = new Button("Export Path");
////        pathStatusLabel = new Label("No path created.");
////        pathStatusLabel.setStyle("-fx-text-fill: #CFD8DC;");
////
////        // --- Utility Controls ---
////        Label utilityTitle = new Label("UTILITIES");
////        utilityTitle.setFont(labelFont);
////        utilityTitle.setStyle("-fx-text-fill: #ECEFF1;");
////        clearTrailButton = new Button("Clear Robot Trail");
////        clearNamedLinesButton = new Button("Clear Custom Lines");
////
////        // --- Initialize and Load Recording Icons ---
////        loadPlaybackIcons();
////
////        // --- Create Recording Buttons ---
////        recordButton = new Button();
////        recordButton.setGraphic(recordIcon); // Default to record icon
////        playPauseButton = new Button();
////        playPauseButton.setGraphic(playIcon); // Default to play icon
////        reverseButton = new Button();
////        reverseButton.setGraphic(reverseIcon);
////        forwardButton = new Button();
////        forwardButton.setGraphic(forwardIcon);
////
////        timelineSlider = new Slider();
////        timelineSlider.setMin(0);
////        timelineSlider.setMax(100); // Default max, will be updated
////        timelineSlider.setValue(0);
////        timelineSlider.setDisable(true); // Disabled by default
////
////        // --- Layout for Recording Buttons ---
////        Label recordingTitle = new Label("RECORDING");
////        recordingTitle.setFont(labelFont);
////        recordingTitle.setStyle("-fx-text-fill: #ECEFF1;");
////        HBox recordingControls = new HBox(10, reverseButton, playPauseButton, forwardButton, recordButton);
////        recordingControls.setAlignment(Pos.CENTER);
////
////        // Add all components to the VBox layout
////        this.getChildren().addAll(
////                statusTitle, xPosLabel.getParent(), yPosLabel.getParent(), headingLabel.getParent(),
////                pathTitle, newPathButton, deletePathButton, exportPathButton, pathStatusLabel,
////                utilityTitle, clearTrailButton, clearNamedLinesButton,
////                recordingTitle, recordingControls, timelineSlider
////        );
////
////        // Set initial state for new buttons
////        setPlaybackControlsDisabled(true); // Playback is disabled until a recording exists
////    }
////
////    public ControlPanel(double preferredWidth) {
////        super(10); // Spacing between elements in VBox
////        setPadding(new Insets(15));
////        setPrefWidth(preferredWidth);
////        setStyle("-fx-background-color: #ECEFF1;"); // Light gray background for the control pane
////
////        Label controlsTitle = new Label("Controls");
////        controlsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 20));
////        controlsTitle.setTextFill(Color.DARKSLATEBLUE);
////        controlsTitle.setMaxWidth(Double.MAX_VALUE); // Allow title to take full width
////        controlsTitle.setAlignment(Pos.CENTER);     // Center title text
////
////        // --- Path Controls Section ---
////        Label pathControlsTitle = new Label("Path Management");
////        pathControlsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
////        pathControlsTitle.setPadding(new Insets(10, 0, 5, 0));
////
////        newPathButton = new Button("New Path");
////        newPathButton.setMaxWidth(Double.MAX_VALUE);
////        deletePathButton = new Button("Delete Path");
////        deletePathButton.setMaxWidth(Double.MAX_VALUE);
////        exportPathButton = new Button("Export Path (CSV)");
////        exportPathButton.setMaxWidth(Double.MAX_VALUE);
////        clearTrailButton = new Button("Clear Robot Trail");
////        clearTrailButton.setMaxWidth(Double.MAX_VALUE);
////        clearNamedLinesButton = new Button("Clear Custom Lines");
////        clearNamedLinesButton.setMaxWidth(Double.MAX_VALUE);
////
////        VBox pathControlsBox = new VBox(8, pathControlsTitle, newPathButton, deletePathButton, exportPathButton, clearTrailButton, clearNamedLinesButton);
////        pathControlsBox.setPadding(new Insets(0,0,10,0));
////
////
////        // --- Placeholder for other future controls (e.g., Robot Manual Control) ---
////        VBox otherControlsArea = new VBox(8);
////        otherControlsArea.setPadding(new Insets(10, 0, 10, 0));
////        // Add other controls here if needed in the future
////
////        // --- Status Section ---
////        Label statusTitle = new Label("Status");
////        statusTitle.setFont(Font.font("Arial", FontWeight.BOLD, 18));
////        statusTitle.setTextFill(Color.DARKSLATEBLUE);
////        statusTitle.setPadding(new Insets(10, 0, 5, 0)); // Top padding for status title
////        statusTitle.setMaxWidth(Double.MAX_VALUE);
////        statusTitle.setAlignment(Pos.CENTER);
////
////        // Initialize status labels
////        robotXLabel = new Label("Robot X:");
////        robotYLabel = new Label("Robot Y:");
////        robotHeadingLabel = new Label("Heading:");
////
////        robotXValueLabel = new Label("N/A");
////        robotYValueLabel = new Label("N/A");
////        robotHeadingValueLabel = new Label("N/A");
////
////        // Style value labels
////        Font valueFont = Font.font("Arial", 14);
////        robotXValueLabel.setFont(valueFont);
////        robotYValueLabel.setFont(valueFont);
////        robotHeadingValueLabel.setFont(valueFont);
////
////        // Layout for status items using HBox for label + value pairs
////        HBox xStatusBox = new HBox(5, robotXLabel, robotXValueLabel);
////        HBox yStatusBox = new HBox(5, robotYLabel, robotYValueLabel);
////        HBox headingStatusBox = new HBox(5, robotHeadingLabel, robotHeadingValueLabel);
////        xStatusBox.setAlignment(Pos.CENTER_LEFT);
////        yStatusBox.setAlignment(Pos.CENTER_LEFT);
////        headingStatusBox.setAlignment(Pos.CENTER_LEFT);
////
////        VBox statusInfoBox = new VBox(5); // Vertical layout for all status items
////        statusInfoBox.setPadding(new Insets(5, 10, 10, 10)); // Padding around the status info
////        statusInfoBox.setStyle("-fx-border-color: #B0BEC5; -fx-border-width: 1; -fx-border-radius: 5;"); // Light border
////        statusInfoBox.getChildren().addAll(xStatusBox, yStatusBox, headingStatusBox);
////
////        // --- Layout within ControlPanel ---
////        // Spacer to push status section to the bottom
////        VBox spacer = new VBox();
////        VBox.setVgrow(spacer, Priority.ALWAYS); // Spacer takes all available vertical space
////
////        getChildren().addAll(controlsTitle, pathControlsBox, otherControlsArea, spacer, statusTitle, statusInfoBox);
////    }
//
////    private Label createStatusLabel(String labelText, Font valueFont) {
////        Label label = new Label(labelText);
////        label.setStyle("-fx-text-fill: #B0BEC5;");
////        Label valueLabel = new Label("0.0");
////        valueLabel.setFont(valueFont);
////        valueLabel.setStyle("-fx-text-fill: #FFFFFF;");
////        HBox hbox = new HBox(5, label, valueLabel);
////        hbox.setAlignment(Pos.CENTER_LEFT);
////        return valueLabel;
////    }
//
//
//    private void loadPlaybackIcons() {
//        double iconSize = 20; // Adjust icon size as needed
//        try {
//            // getClass().getResourceAsStream() is robust for finding resources in JARs
//            recordIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/record.png"), iconSize, iconSize, true, true));
//            stopIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/stop.png"), iconSize, iconSize, true, true));
//            playIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/play.png"), iconSize, iconSize, true, true));
//            pauseIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/pause.png"), iconSize, iconSize, true, true));
//            reverseIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/reverse.png"), iconSize, iconSize, true, true));
//            forwardIcon = new ImageView(new Image(getClass().getResourceAsStream("/icons/forward.png"), iconSize, iconSize, true, true));
//        } catch (Exception e) {
//            System.err.println("Error loading playback icons. Make sure they are in 'src/main/resources/icons/'");
//            // Create empty fallbacks so the app doesn't crash
//            recordIcon = new ImageView();
//            stopIcon = new ImageView();
//            playIcon = new ImageView();
//            pauseIcon = new ImageView();
//            reverseIcon = new ImageView();
//            forwardIcon = new ImageView();
//        }
//    }
//
//    // --- Methods to update status labels ---
//    public void updateRobotStatus(double x, double y, double h) {
//        xPosLabel.setText(String.format("%.2f", x));
//        yPosLabel.setText(String.format("%.2f", y));
//        headingLabel.setText(String.format("%.2f°", h));
//    }
//
////    // --- Public methods to update status display ---
////    public void updateRobotStatus(double x, double y, double heading) {
////        // Format the numbers for display (e.g., to 2 decimal places)
////        robotXValueLabel.setText(String.format("%.2f in", x));
////        robotYValueLabel.setText(String.format("%.2f in", y));
////        robotHeadingValueLabel.setText(String.format("%.1f°", heading));
////    }
//
//    public void setOnNewPathAction(EventHandler<ActionEvent> handler) {
//        newPathButton.setOnAction(handler);
//    }
//
//    public void setOnDeletePathAction(EventHandler<ActionEvent> handler) {
//        deletePathButton.setOnAction(handler);
//    }
//
//    public void setOnExportPathAction(EventHandler<ActionEvent> handler) {
//        exportPathButton.setOnAction(handler);
//    }
//
//    public void setOnClearTrailAction(EventHandler<ActionEvent> handler) {
//        clearTrailButton.setOnAction(handler);
//    }
//
//    public void setOnClearNamedLinesAction(EventHandler<ActionEvent> handler) {
//        clearNamedLinesButton.setOnAction(handler);
//    }
//
//    public void setOnRecordAction(Runnable action) {
//        recordButton.setOnAction(e -> action.run());
//    }
//
//    public void setOnPlayPauseAction(Runnable action) {
//        playPauseButton.setOnAction(e -> action.run());
//    }
//    // public void setOnReverseAction(...) { /* For future implementation */ }
//    // public void setOnForwardAction(...) { /* For future implementation */ }
//
//    // +++ Listener for when the user interacts with the slider +++
//    public void setOnTimelineSliderChanged(ChangeListener<Number> listener) {
//        timelineSlider.valueProperty().addListener(listener);
//    }
//
//    // --- Methods to enable/disable buttons ---
//    public void setPathEditingActive(boolean isActive) {
//        newPathButton.setDisable(isActive); // Disable "New Path" while editing
//        deletePathButton.setDisable(isActive);
//        exportPathButton.setDisable(isActive);
//    }
//
//    /**
//     * Updates the slider's position and range based on playback progress.
//     * @param currentEventIndex The index of the current event.
//     * @param totalEvents The total number of events in the recording.
//     */
//    public void updateTimelineSlider(int currentEventIndex, int totalEvents) {
//        if (totalEvents > 0) {
//            // The max value is the index of the last event
//            timelineSlider.setMax(totalEvents - 1);
//            timelineSlider.setValue(currentEventIndex);
//        } else {
//            timelineSlider.setMax(100); // Reset to default
//            timelineSlider.setValue(0);
//        }
//    }
//
//    /**
//     * Enables or disables the entire timeline slider.
//     * @param isDisabled True to disable, false to enable.
//     */
//    public void setTimelineSliderDisabled(boolean isDisabled) {
//        timelineSlider.setDisable(isDisabled);
//    }
//
//    /**
//     * Toggles the icon on the record/stop button.
//     * @param isRecording If true, show stop icon; otherwise, show record icon.
//     */
//    public void toggleRecordButtonIcon(boolean isRecording) {
//        recordButton.setGraphic(isRecording ? stopIcon : recordIcon);
//    }
//
//    /**
//     * Toggles the icon on the play/pause button.
//     * @param isPlaying If true, show pause icon; otherwise, show play icon.
//     */
//    public void togglePlayPauseButtonIcon(boolean isPlaying) {
//        playPauseButton.setGraphic(isPlaying ? pauseIcon : playIcon);
//    }
//
//    /**
//     * Enables or disables the playback-related buttons (play, reverse, forward).
//     * @param isDisabled True to disable, false to enable.
//     */
//    public void setPlaybackControlsDisabled(boolean isDisabled) {
//        playPauseButton.setDisable(isDisabled);
//        reverseButton.setDisable(isDisabled);
//        forwardButton.setDisable(isDisabled);
//    }
//
//    public void enablePathControls(boolean pathExists) {
//        deletePathButton.setDisable(!pathExists);
//        exportPathButton.setDisable(!pathExists);
//    }
//
//    public void setPathStatus(String text) {
//        pathStatusLabel.setText(text);
//    }
//
//    public javafx.scene.control.Slider getTimelineSlider() {
//        return this.timelineSlider;
//    }
//
//    public void setOnSliderMouseReleased(EventHandler<MouseEvent> handler) {
//        timelineSlider.setOnMouseReleased(handler);
//    }
//}