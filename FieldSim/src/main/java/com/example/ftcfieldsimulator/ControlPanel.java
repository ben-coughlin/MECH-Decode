package com.example.ftcfieldsimulator;

import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections; // +++ ADD
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox; // +++ ADD
import javafx.scene.control.Label;
import javafx.scene.control.ListCell; // +++ ADD
import javafx.scene.control.ListView; // +++ ADD (implicitly used by ComboBox for cellFactory)
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.util.Callback; // +++ ADD

import java.util.Arrays; // +++ ADD
import java.util.ArrayList;
import java.util.List;   // +++ ADD
import java.util.Locale;

public class ControlPanel extends VBox {
    public static final double PREFERRED_WIDTH_RATIO_TO_FIELD = 1.0 / 3.0;
    public static final String ALL_POINTS_MARKER = "ALL Points"; // +++ ADD
    public static final String TEXTFIELD_VARIES_TEXT = "-- Varies --"; // +++ ADD

    // --- UI Elements ---
    private Button newPathButton, deletePathButton, exportPathButton, clearTrailButton, clearNamedLinesButton;
    private Button sendPathButton;
    private Button recordButton, playPauseButton, reverseButton, forwardButton;
    private Button openButton, saveButton;
    private ImageView recordIcon, stopIcon, playIcon, pauseIcon, reverseIcon, forwardIcon;
    private Slider timelineSlider;
    private Button showPlotButton;

    // --- CurvePoint Parameter UI Elements ---
    private ComboBox<Object> pointSelectionComboBox; // +++ ADD: ComboBox for point selection
    private TextField moveSpeedField;
    private TextField turnSpeedField;
    private TextField followDistanceField;
    private TextField pointLengthField;
    private TextField slowDownTurnDegreesField;
    private TextField slowDownTurnAmountField;
    private List<TextField> paramTextFieldsList; // +++ ADD: For easy access

    private Label timeLapsedLabel;

    // Default values for the fields
    public static final String DEFAULT_MOVE_SPEED = String.format(Locale.US, "%.1f", 0.4);
    public static final String DEFAULT_TURN_SPEED = String.format(Locale.US, "%.1f", 0.4);
    public static final String DEFAULT_FOLLOW_DISTANCE = String.format(Locale.US, "%.1f", 10.0);
    public static final String DEFAULT_POINT_LENGTH = String.format(Locale.US, "%.1f", 10.0);
    public static final String DEFAULT_SLOW_DOWN_TURN_DEGREES = String.format(Locale.US, "%.1f", Math.toDegrees(Math.toRadians(60)));
    public static final String DEFAULT_SLOW_DOWN_TURN_AMOUNT = String.format(Locale.US, "%.1f", 0.6);


    public ControlPanel(double preferredWidth) {
        super(10);
        setPadding(new Insets(15));
        setPrefWidth(preferredWidth);
        setStyle("-fx-background-color: #ECEFF1;");

        // --- Path Management Section ---
        Label pathTitle = new Label("Path Management");
        pathTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        newPathButton = createMaxWidthButton("New Path");
        deletePathButton = createMaxWidthButton("Delete Path");
        exportPathButton = createMaxWidthButton("Export Path (CSV)");
        sendPathButton = createMaxWidthButton("Send Path to Robot");
        VBox pathControlsBox = new VBox(8, pathTitle, newPathButton, deletePathButton, exportPathButton, sendPathButton);

        // --- CurvePoint Parameters Section ---
        Label paramsTitle = new Label("CurvePoint Parameters");
        paramsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));

        // +++ ADD: ComboBox for Point Selection +++
        pointSelectionComboBox = new ComboBox<>();
        pointSelectionComboBox.setPromptText("Select Point/ALL");
        pointSelectionComboBox.setMaxWidth(Double.MAX_VALUE);
        // Custom cell factory to display CurvePoint objects nicely
        pointSelectionComboBox.setCellFactory(new Callback<ListView<Object>, ListCell<Object>>() {
            @Override
            public ListCell<Object> call(ListView<Object> param) {
                return new ListCell<Object>() {
                    @Override
                    protected void updateItem(Object item, boolean empty) {
                        super.updateItem(item, empty);
                        if (item == null || empty) {
                            setText(null);
                        } else if (item instanceof String) { // For ALL_POINTS_MARKER
                            setText((String) item);
                        } else if (item instanceof CurvePoint) {
                            CurvePoint cp = (CurvePoint) item;
                            // Find index for display (1-based)
                            int index = pointSelectionComboBox.getItems().indexOf(cp); // Index in full list
                            if (pointSelectionComboBox.getItems().contains(ALL_POINTS_MARKER) && index > 0) {
                                index--; // Adjust if "ALL" is present
                            }
                            setText(String.format(Locale.US, "Point %d (X:%.1f, Y:%.1f)",
                                    index, cp.x, cp.y));
                        } else {
                            setText(item.toString());
                        }
                    }
                };
            }
        });
        // Also set button cell for the displayed value in ComboBox itself
        pointSelectionComboBox.setButtonCell(pointSelectionComboBox.getCellFactory().call(null));


        GridPane paramsGrid = new GridPane();
        paramsGrid.setHgap(10);
        paramsGrid.setVgap(8);
        paramsGrid.setPadding(new Insets(10, 0, 10, 0)); // Added top padding

        ColumnConstraints column1 = new ColumnConstraints();
        column1.setMinWidth(120);
        ColumnConstraints column2 = new ColumnConstraints();
        column2.setHgrow(Priority.SOMETIMES);
        paramsGrid.getColumnConstraints().addAll(column1, column2);

        double textFieldMaxWidth = 100;
        moveSpeedField = new TextField();
        moveSpeedField.setMaxWidth(textFieldMaxWidth);
        turnSpeedField = new TextField();
        turnSpeedField.setMaxWidth(textFieldMaxWidth);
        followDistanceField = new TextField();
        followDistanceField.setMaxWidth(textFieldMaxWidth);
        pointLengthField = new TextField();
        pointLengthField.setMaxWidth(textFieldMaxWidth);
        slowDownTurnDegreesField = new TextField();
        slowDownTurnDegreesField.setMaxWidth(textFieldMaxWidth);
        slowDownTurnAmountField = new TextField();
        slowDownTurnAmountField.setMaxWidth(textFieldMaxWidth);

        // Store TextFields in a list for easier manipulation
        paramTextFieldsList = Arrays.asList(
                moveSpeedField, turnSpeedField, followDistanceField,
                pointLengthField, slowDownTurnDegreesField, slowDownTurnAmountField
        );

        paramsGrid.add(new Label("Move Speed:"), 0, 0);
        paramsGrid.add(moveSpeedField, 1, 0);
        paramsGrid.add(new Label("Turn Speed:"), 0, 1);
        paramsGrid.add(turnSpeedField, 1, 1);
        paramsGrid.add(new Label("Follow Distance:"), 0, 2);
        paramsGrid.add(followDistanceField, 1, 2);
        paramsGrid.add(new Label("Point Length:"), 0, 3);
        paramsGrid.add(pointLengthField, 1, 3);
        paramsGrid.add(new Label("Slow Turn (Deg):"), 0, 4);
        paramsGrid.add(slowDownTurnDegreesField, 1, 4);
        paramsGrid.add(new Label("Slow Turn Amount:"), 0, 5);
        paramsGrid.add(slowDownTurnAmountField, 1, 5);

        // +++ MODIFIED: Add ComboBox to this VBox +++
        VBox curveParamsBox = new VBox(8, paramsTitle, pointSelectionComboBox, paramsGrid);

        // --- Utility Controls Section ---
        // ... (Utility controls remain the same)
        Label utilityTitle = new Label("Utilities");
        utilityTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        clearTrailButton = createMaxWidthButton("Clear Robot Trail");
        clearNamedLinesButton = createMaxWidthButton("Clear Custom Lines");
        VBox utilityControlsBox = new VBox(8, utilityTitle, clearTrailButton, clearNamedLinesButton);


        // --- Recording Controls Section ---
        // ... (Recording controls remain the same)
        loadPlaybackIcons();
        Label recordingTitle = new Label("Recording");
        recordingTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        openButton = createMaxWidthButton("Open");
        saveButton = createMaxWidthButton("Save");
        //saveButton.setDisable(true); // Will be managed by app logic
        HBox fileButtons = new HBox(10, openButton, saveButton);
        HBox.setHgrow(openButton, Priority.ALWAYS);
        HBox.setHgrow(saveButton, Priority.ALWAYS);
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
        timeLapsedLabel = new Label("Time: 0.000"); // Initial text
        timeLapsedLabel.setFont(Font.font("Consolas", 12)); // Monospaced font is good for numbers
        timeLapsedLabel.setMaxWidth(Double.MAX_VALUE);
        timeLapsedLabel.setAlignment(Pos.CENTER_RIGHT); // Align to the right below slider
        timeLapsedLabel.setPadding(new Insets(2, 0, 0, 0)); // Small top padding

        VBox recordingControlsBox = new VBox(8, recordingTitle, fileButtons, recordingButtons, timelineSlider, timeLapsedLabel);

        // --- Tools Section ---
//        Label toolsTitle = new Label("Tools & Views");
//        toolsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        showPlotButton = createMaxWidthButton("Show Time Plot"); // Uses your existing helper
//        VBox toolsControlsBox = new VBox(8, toolsTitle, showPlotButton);
        VBox toolsControlsBox = new VBox(8, showPlotButton);

        // --- Initial State ---
        enablePathControls(false); // New, Delete, Export, Send
        setPlaybackControlsDisabled(true); // Play, Pause, Fwd, Rev, Slider
        setSaveButtonDisabled(true);
        setPointEditingControlsDisabled(true); // ComboBox and parameter TextFields
        loadGlobalDefaultsIntoParameterFields(); // Load defaults into TextFields on startup

        this.getChildren().addAll(pathControlsBox, curveParamsBox, utilityControlsBox, recordingControlsBox, toolsControlsBox);
    }

    private Button createMaxWidthButton(String text) {
        Button button = new Button(text);
        button.setMaxWidth(Double.MAX_VALUE);
        return button;
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
            System.err.println("Error loading playback icons: " + e.getMessage());
            // Consider more robust error handling or default icons
        }
    }

    /**
     * Updates the time lapsed label.
     * @param totalMilliseconds The total time lapsed in milliseconds.
     */
    public void updateTimeLapsed(long totalMilliseconds) {
        if (totalMilliseconds < 0) {
            timeLapsedLabel.setText("Time: --.---");
            return;
        }
        long seconds = totalMilliseconds / 1000;
        long millis = totalMilliseconds % 1000;
        timeLapsedLabel.setText(String.format(Locale.US, "Time: %d.%03d", seconds, millis));
    }

    public void setOnOpenAction(EventHandler<ActionEvent> handler) { openButton.setOnAction(handler); }
    public void setOnSaveAction(EventHandler<ActionEvent> handler) { saveButton.setOnAction(handler); }
    public void setSaveButtonDisabled(boolean isDisabled) { saveButton.setDisable(isDisabled); }

    public void setOnNewPathAction(EventHandler<ActionEvent> handler) { newPathButton.setOnAction(handler); }
    public void setOnDeletePathAction(EventHandler<ActionEvent> handler) { deletePathButton.setOnAction(handler); }
    public void setOnExportPathAction(EventHandler<ActionEvent> handler) { exportPathButton.setOnAction(handler); }
    public void setOnSendPathAction(EventHandler<ActionEvent> handler) { if (sendPathButton != null) { sendPathButton.setOnAction(handler); } }
    public void setOnClearTrailAction(EventHandler<ActionEvent> handler) { clearTrailButton.setOnAction(handler); }
    public void setOnClearNamedLinesAction(EventHandler<ActionEvent> handler) { clearNamedLinesButton.setOnAction(handler); }

    public void setOnRecordAction(Runnable action) { recordButton.setOnAction(e -> action.run()); }
    public void setOnPlayPauseAction(Runnable action) { playPauseButton.setOnAction(e -> action.run()); }
    public void setOnForwardAction(Runnable action) { if (forwardButton != null) { forwardButton.setOnAction(e -> action.run()); } }
    public void setOnReverseAction(Runnable action) { if (reverseButton != null) { reverseButton.setOnAction(e -> action.run()); } }

    public void setOnTimelineSliderChanged(ChangeListener<Number> listener) { timelineSlider.valueProperty().addListener(listener); }
    public void setOnSliderMouseReleased(EventHandler<MouseEvent> handler) { timelineSlider.setOnMouseReleased(handler); }

    public void setOnShowPlotAction(EventHandler<ActionEvent> handler) { if (showPlotButton != null) { showPlotButton.setOnAction(handler); } }

    public void setPathEditingActive(boolean isActive) {
        newPathButton.setDisable(isActive);
        // Delete, Export, Send are primarily controlled by enablePathControls,
        // but also disabled if actively drawing a new path.
        deletePathButton.setDisable(isActive || deletePathButton.isDisabled()); // Preserve existing disabled state from pathExists
        exportPathButton.setDisable(isActive || exportPathButton.isDisabled());
        if (sendPathButton != null) {
            sendPathButton.setDisable(isActive || sendPathButton.isDisabled());
        }
    }

    public void enablePathControls(boolean pathExists) {
        // These buttons are only active if a path exists AND path editing is NOT active.
        boolean pathEditingMode = newPathButton.isDisabled(); // Infer if we are in "new path" mode

        deletePathButton.setDisable(!pathExists || pathEditingMode);
        exportPathButton.setDisable(!pathExists || pathEditingMode);
        if (sendPathButton != null) {
            sendPathButton.setDisable(!pathExists || pathEditingMode);
        }
    }

    public void updateTimelineSlider(int currentEventIndex, int totalEvents) {
        if (totalEvents > 1) {
            timelineSlider.setMax(totalEvents - 1);
            timelineSlider.setValue(currentEventIndex);
        } else {
            timelineSlider.setMax(100); // Default max if no events or one event
            timelineSlider.setValue(0);
        }
    }

    public void setTimelineSliderDisabled(boolean isDisabled) { timelineSlider.setDisable(isDisabled); }
    public void toggleRecordButtonIcon(boolean isRecording) { recordButton.setGraphic(isRecording ? stopIcon : recordIcon); }
    public void togglePlayPauseButtonIcon(boolean isPlaying) { playPauseButton.setGraphic(isPlaying ? pauseIcon : playIcon); }

    public void setPlaybackControlsDisabled(boolean isDisabled) {
        playPauseButton.setDisable(isDisabled);
        reverseButton.setDisable(isDisabled);
        forwardButton.setDisable(isDisabled);
        timelineSlider.setDisable(isDisabled); // Often tied to playback controls state
    }

    public Slider getTimelineSlider() { return this.timelineSlider; }

    // --- Methods for Per-Point Parameter Editing ---

    /**
     * Enables or disables the ComboBox for point selection and all parameter TextFields.
     * @param disabled true to disable, false to enable.
     */
    public void setPointEditingControlsDisabled(boolean disabled) {
        pointSelectionComboBox.setDisable(disabled);
        for (TextField tf : paramTextFieldsList) {
            tf.setDisable(disabled);
        }
    }

    /**
     * Updates the point selection ComboBox with the given path.
     * Tries to re-select objectToSelect if provided and found.
     * If path is empty, only "ALL" is shown (and usually disabled by setPointEditingControlsDisabled).
     * If path is not empty and objectToSelect is null, "ALL" is selected.
     */
    public void updatePointSelectionComboBox(List<CurvePoint> path, Object objectToSelect) {
        // This method is responsible for completely refreshing the ComboBox items
        // and trying to set the selection appropriately.

        // 1. Prepare the new list of items that the ComboBox should display.
        List<Object> newItemsForComboBox = new ArrayList<>(); // This line should be fine with imports
        newItemsForComboBox.add(ALL_POINTS_MARKER); // "ALL Points" is always the first option

        if (path != null && !path.isEmpty()) {
            newItemsForComboBox.addAll(path); // Add all current CurvePoint objects from the given path
        }

        // 2. Set the ComboBox's items to this newly prepared list.
        pointSelectionComboBox.setItems(FXCollections.observableArrayList(newItemsForComboBox));

        // 3. Attempt to set the selected value in the ComboBox.
        if (objectToSelect != null && newItemsForComboBox.contains(objectToSelect)) {
            pointSelectionComboBox.setValue(objectToSelect);
        } else if (path != null && !path.isEmpty()) {
            pointSelectionComboBox.setValue(ALL_POINTS_MARKER);
        } else {
            if (!pointSelectionComboBox.isDisabled()) {
                pointSelectionComboBox.setValue(ALL_POINTS_MARKER);
            } else {
                pointSelectionComboBox.setValue(null);
            }
        }
    }

//    public void updatePointSelectionComboBox(List<CurvePoint> path, Object objectToSelect) {
//        Object previouslySelected = pointSelectionComboBox.getValue();
//        pointSelectionComboBox.getItems().clear();
//        pointSelectionComboBox.getItems().add(ALL_POINTS_MARKER);
//
//        if (path != null && !path.isEmpty()) {
//            for (int i = 0; i < path.size(); i++) {
//                pointSelectionComboBox.getItems().add(path.get(i)); // Add the CurvePoint object
//            }
//        }
//
//        if (objectToSelect != null && pointSelectionComboBox.getItems().contains(objectToSelect)) {
//            pointSelectionComboBox.setValue(objectToSelect);
//        } else if (path != null && !path.isEmpty()) {
//            pointSelectionComboBox.setValue(ALL_POINTS_MARKER); // Default to "ALL" if path exists and no specific selection
//        } else if (previouslySelected != null && pointSelectionComboBox.getItems().contains(previouslySelected)){
//            pointSelectionComboBox.setValue(previouslySelected); // try to keep selection if valid
//        } else {
//            pointSelectionComboBox.setValue(null); // Or ALL_POINTS_MARKER if preferred when empty and controls are enabled
//            if(!pointSelectionComboBox.isDisabled()) pointSelectionComboBox.setValue(ALL_POINTS_MARKER);
//        }
//    }

    public void setOnPointSelectionAction(ChangeListener<Object> listener) {
        pointSelectionComboBox.valueProperty().addListener(listener);
    }

    public Object getSelectedPointFromComboBox() {
        return pointSelectionComboBox.getValue();
    }

    /**
     * Loads the parameters from the given CurvePoint into the TextFields.
     * Converts radians to degrees for display.
     */
    public void loadParametersForPoint(CurvePoint point) {
        if (point == null) {
            clearParameterFields(); // Or load global defaults
            return;
        }
        moveSpeedField.setText(String.format(Locale.US, "%.2f", point.moveSpeed));
        turnSpeedField.setText(String.format(Locale.US, "%.2f", point.turnSpeed));
        followDistanceField.setText(String.format(Locale.US, "%.2f", point.followDistance));
        pointLengthField.setText(String.format(Locale.US, "%.2f", point.pointLength));
        slowDownTurnDegreesField.setText(String.format(Locale.US, "%.1f", Math.toDegrees(point.slowDownTurnRadians)));
        slowDownTurnAmountField.setText(String.format(Locale.US, "%.2f", point.slowDownTurnAmount));
    }

    /**
     * Loads the application's global default values into the parameter TextFields.
     */
    public void loadGlobalDefaultsIntoParameterFields() {
        moveSpeedField.setText(DEFAULT_MOVE_SPEED);
        turnSpeedField.setText(DEFAULT_TURN_SPEED);
        followDistanceField.setText(DEFAULT_FOLLOW_DISTANCE);
        pointLengthField.setText(DEFAULT_POINT_LENGTH);
        slowDownTurnDegreesField.setText(DEFAULT_SLOW_DOWN_TURN_DEGREES);
        slowDownTurnAmountField.setText(DEFAULT_SLOW_DOWN_TURN_AMOUNT);
    }

    /**
     * Sets all parameter TextFields to display "-- Varies --".
     */
    public void showVariesInParameterFields() {
        for (TextField tf : paramTextFieldsList) {
            tf.setText(TEXTFIELD_VARIES_TEXT);
        }
    }

    /**
     * Clears the text in all parameter TextFields.
     */
    public void clearParameterFields() {
        for (TextField tf : paramTextFieldsList) {
            tf.setText("");
        }
    }

    // --- Getters for parameter TextFields (for FtcFieldSimulatorApp to add listeners) ---
    public TextField getMoveSpeedField() { return moveSpeedField; }
    public TextField getTurnSpeedField() { return turnSpeedField; }
    public TextField getFollowDistanceField() { return followDistanceField; }
    public TextField getPointLengthField() { return pointLengthField; }
    public TextField getSlowDownTurnDegreesField() { return slowDownTurnDegreesField; }
    public TextField getSlowDownTurnAmountField() { return slowDownTurnAmountField; }
    public List<TextField> getAllParamTextFields() { return paramTextFieldsList; }


    // --- Getters for CurvePoint Parameters from TextFields (used by FtcFieldSimulatorApp) ---
    // These remain the same, used when creating NEW points or when "ALL" is modified.
    public double getMoveSpeedParam() throws NumberFormatException { return Double.parseDouble(moveSpeedField.getText()); }
    public double getTurnSpeedParam() throws NumberFormatException { return Double.parseDouble(turnSpeedField.getText()); }
    public double getFollowDistanceParam() throws NumberFormatException { return Double.parseDouble(followDistanceField.getText()); }
    public double getPointLengthParam() throws NumberFormatException { return Double.parseDouble(pointLengthField.getText()); }
    public double getSlowDownTurnDegreesParam() throws NumberFormatException { return Double.parseDouble(slowDownTurnDegreesField.getText()); }
    public double getSlowDownTurnAmountParam() throws NumberFormatException { return Double.parseDouble(slowDownTurnAmountField.getText()); }
}

