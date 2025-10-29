package com.example.ftcfieldsimulator;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.geometry.Insets;
import javafx.geometry.Point2D;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox; // NEW IMPORT
import javafx.scene.paint.Color;
import javafx.stage.FileChooser;
import javafx.stage.Stage;

import com.example.ftcfieldsimulator.UdpPositionListener.CircleData;
import com.example.ftcfieldsimulator.UdpPositionListener.KeyValueData;
import com.example.ftcfieldsimulator.UdpPositionListener.LineData;
import com.example.ftcfieldsimulator.UdpPositionListener.PositionData;
import com.example.ftcfieldsimulator.UdpPositionListener.TextData;
import com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

public class FtcFieldSimulatorApp extends Application {

    // --- UI and State Components ---
    private FieldDisplay fieldDisplay;
    private ControlPanel controlPanel;
    private FieldKeyValueTable keyValueTable;
    private FieldStatusDisplay fieldStatusDisplay; // NEW
    private Robot robot;
    // ... (rest of member variables are the same)
    private RecordingManager recordingManager;
    private UdpPositionListener udpListener;
    private Thread udpListenerThread;
    private Label instructionLabel;
    private Stage primaryStage;
    private PlotDisplayWindow plotDisplayWindow;
    private UdpPlotListener udpPlotListener;
    private Thread udpPlotListenerThread;
    private List<CurvePoint> currentPath = new ArrayList<>();
    private boolean isCreatingPath = false;
    private Map<String, LineData> namedLinesToDraw = new HashMap<>();
    private final Object namedLinesLock = new Object();
    private Map<TextField, String> textFieldPreviousValues = new HashMap<>();

    // --- Configuration Constants (omitted for brevity, no changes here) ---
    public static final double FIELD_WIDTH_INCHES = 144.0;
    public static final double FIELD_HEIGHT_INCHES = 144.0;
    private static final int FIELD_DISPLAY_WIDTH_PIXELS = 800;
    private static final int FIELD_DISPLAY_HEIGHT_PIXELS = 800;
    private static final String FIELD_IMAGE_PATH = "/decode_field.png";
    private static final String ROBOT_IMAGE_PATH = "/robot.png";
    public static final double FIELD_IMAGE_ALPHA = 0.3;
    public static final double BACKGROUND_ALPHA = 0.1;
    public static final double ROBOT_START_FIELD_X = 0.0;
    public static final double ROBOT_START_FIELD_Y = 0.0;
    public static final double ROBOT_START_HEADING_DEGREES = 0.0;
    private static final int UDP_LISTENER_PORT = 7777;
    private static final int ROBOT_LISTENER_PORT = 6666;
    private static final String ROBOT_IP_ADDRESS = "192.168.43.1";
    private static final double ROBOT_MOVE_INCREMENT_INCHES = 2.0;
    private static final double ROBOT_TURN_INCREMENT_DEGREES = 5.0;
    private static final double MASTER_DEFAULT_MOVE_SPEED = 0.4;
    private static final double MASTER_DEFAULT_TURN_SPEED = 0.4;
    private static final double MASTER_DEFAULT_FOLLOW_DISTANCE = 10.0;
    private static final double MASTER_DEFAULT_POINT_LENGTH = 10.0;
    private static final double MASTER_DEFAULT_SLOW_DOWN_TURN_RADIANS = Math.toRadians(60);
    private static final double MASTER_DEFAULT_SLOW_DOWN_TURN_AMOUNT = 0.6;


    public static void main(String[] args) {
        Application.launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        this.primaryStage = primaryStage;
        primaryStage.setTitle("FTC Field Simulator");

        double controlPanelWidth = FIELD_DISPLAY_WIDTH_PIXELS * ControlPanel.PREFERRED_WIDTH_RATIO_TO_FIELD;
        double rightPanelWidth = controlPanelWidth * 0.9; // Make right panel slightly smaller
        double totalAppWidth = FIELD_DISPLAY_WIDTH_PIXELS + controlPanelWidth + rightPanelWidth;
        double totalAppHeight = FIELD_DISPLAY_HEIGHT_PIXELS;

        // --- Init core components ---
        this.recordingManager = new RecordingManager(this::handleUdpMessage, this::onPlaybackFinished);
        recordingManager.setOnProgressUpdate(index -> {
            if (controlPanel != null && controlPanel.getTimelineSlider() != null && !controlPanel.getTimelineSlider().isValueChanging()) {
                controlPanel.getTimelineSlider().setValue(index);
            }
            updateTimeLapsedDisplay();
        });

        this.robot = new Robot(ROBOT_START_FIELD_X, ROBOT_START_FIELD_Y, ROBOT_START_HEADING_DEGREES, ROBOT_IMAGE_PATH);
        fieldDisplay = new FieldDisplay(FIELD_DISPLAY_WIDTH_PIXELS, FIELD_DISPLAY_HEIGHT_PIXELS, FIELD_WIDTH_INCHES, FIELD_HEIGHT_INCHES, FIELD_IMAGE_PATH, robot, BACKGROUND_ALPHA, FIELD_IMAGE_ALPHA);
        fieldDisplay.setNamedLinesMap(namedLinesToDraw);
        fieldDisplay.setRobotTextMessage(null);

        // --- Init UI Panels ---
        controlPanel = new ControlPanel(controlPanelWidth);
        keyValueTable = new FieldKeyValueTable(rightPanelWidth);
        fieldStatusDisplay = new FieldStatusDisplay(); // NEW
        instructionLabel = new Label("Create a new path or select a point to edit its parameters.");
        // ... (instructionLabel setup remains the same) ...
        instructionLabel.setPadding(new Insets(5));
        instructionLabel.setMaxWidth(Double.MAX_VALUE);
        instructionLabel.setAlignment(Pos.CENTER);
        HBox instructionPane = new HBox(instructionLabel);
        instructionPane.setAlignment(Pos.CENTER);
        instructionPane.setStyle("-fx-background-color: #CFD8DC;");


        // --- NEW: Assemble the right-side panel ---
        VBox rightPanel = new VBox();
        // The keyValueTable will grow to fill available vertical space. The status display will be its natural height.
        VBox.setVgrow(keyValueTable, Priority.ALWAYS);
        rightPanel.getChildren().addAll(keyValueTable, fieldStatusDisplay);


        // --- Assemble the main layout ---
        BorderPane mainLayout = new BorderPane();
        mainLayout.setLeft(controlPanel);
        mainLayout.setCenter(fieldDisplay);
        mainLayout.setRight(rightPanel); // MODIFIED
        mainLayout.setBottom(instructionPane);

        double instructionPaneHeight = 30;
        Scene scene = new Scene(mainLayout, totalAppWidth, totalAppHeight + instructionPaneHeight);
        scene.addEventFilter(KeyEvent.KEY_PRESSED, this::handleSceneKeyPress);

        // --- Wire up everything ---
        setupControlPanelActions(primaryStage);
        setupRecordingControlActions();
        setupParameterFieldListeners();

        startUdpPositionListener();
        startUdpPlotListener();
        primaryStage.setScene(scene);
        primaryStage.setResizable(true);
        primaryStage.show();
        primaryStage.setOnCloseRequest(event -> stopApp());

        updateUIFromRobotState();
        updateControlPanelForPathState();
        updateTimeLapsedDisplay();
    }

    // ... (handleUdpMessage and other methods are unchanged, but updateUIFromRobotState is modified below) ...

    private void updateUIFromRobotState() {
        if (robot != null && fieldDisplay != null) {
            double displayHeading = robot.getHeadingDegrees() % 360;
            if (displayHeading < 0) displayHeading += 360; // Normalize to 0-359

            // --- MODIFIED: Send status update to the new display ---
            if (fieldStatusDisplay != null) {
                fieldStatusDisplay.updateRobotStatus(robot.getXInches(), robot.getYInches(), displayHeading);
            }

            // The old call to controlPanel.updateRobotStatus is now removed from ControlPanel itself.

            fieldDisplay.drawCurrentState();
        }
    }

    // ... all other methods from FtcFieldSimulatorApp remain the same ...
    private void showPlotDisplay() {
        if (plotDisplayWindow == null) {
            plotDisplayWindow = new PlotDisplayWindow(primaryStage); // Pass primary stage as owner
        }
        plotDisplayWindow.show();
    }
    private void handleUdpPlotData(PlotDataEvent dataEvent) {
        if (dataEvent == null) return;

        Platform.runLater(() -> {
            // Log all received plot data for now
            // System.out.println("App received PlotDataEvent: " + dataEvent);

            // If PlotDisplayWindow is not yet created, create it.
            // This ensures if data comes before user clicks button, window can still be prepared.
            // However, it won't show until user clicks.
            if (plotDisplayWindow == null) {
                // plotDisplayWindow = new PlotDisplayWindow(primaryStage); // Create but don't show
                // Let's only interact if it's already created and showing by user action
            }

            if (plotDisplayWindow != null && plotDisplayWindow.isShowing()) {
                PlotDisplay display = plotDisplayWindow.getPlotDisplay();
                if (display != null) {
                    // Pass the generic event. PlotDisplay will decide what to do.
                    display.addPlotEvent(dataEvent);
                }
            } else if (dataEvent instanceof PlotYLimitsEvent || dataEvent instanceof PlotYUnitsEvent) {
                // Optional: If plot window isn't visible, maybe still store these global settings
                // so when it becomes visible, it uses the latest known ones.
                // For now, we only update if visible.
                System.out.println("Plot window not visible. Discarding: " + dataEvent);
            }
        });
    }

    private void setupControlPanelActions(Stage ownerStage) {
        controlPanel.setOnNewPathAction(event -> startNewPathCreation());
        controlPanel.setOnDeletePathAction(event -> deleteCurrentPath());
        controlPanel.setOnExportPathAction(event -> exportPathToCSV(ownerStage));
        controlPanel.setOnSendPathAction(event -> handleSendPathToRobot());
        controlPanel.setOnClearTrailAction(event -> {
            fieldDisplay.clearTrail();
            fieldDisplay.drawCurrentState();
            instructionLabel.setText("Robot trail cleared.");
        });
        controlPanel.setOnClearNamedLinesAction(event -> {
            clearAllNamedLines();
            instructionLabel.setText("All custom lines cleared.");
        });
        controlPanel.setOnPointSelectionAction(this::handlePointSelectionChanged);
        controlPanel.setOnShowPlotAction(event -> showPlotDisplay());
    }

    private void setupParameterFieldListeners() {
        if (controlPanel == null) return;
        for (TextField tf : controlPanel.getAllParamTextFields()) {
            tf.focusedProperty().addListener((obs, oldVal, newVal) -> {
                if (newVal) {
                    textFieldPreviousValues.put(tf, tf.getText());
                } else {
                    handleParameterFieldFocusLost(tf);
                }
            });
            tf.setOnAction(event -> handleParameterFieldFocusLost(tf));
        }
    }

    private void handleParameterFieldFocusLost(TextField textField) {
        if (controlPanel == null || currentPath == null) return;

        String previousText = textFieldPreviousValues.getOrDefault(textField, "");
        String currentText = textField.getText().trim(); // Trim whitespace

        if (Objects.equals(previousText, currentText) && !previousText.equals(ControlPanel.TEXTFIELD_VARIES_TEXT)) {
            return; // No actual change, or was "-- Varies --" and still is (though user can't type that directly)
        }
        // If currentText becomes the "-- Varies --" text, it's likely a programmatic change, not user input for commit.
        if (currentText.equals(ControlPanel.TEXTFIELD_VARIES_TEXT)) {
            textFieldPreviousValues.put(textField, currentText); // Update stored value if it was set programmatically
            return;
        }

        Object selectedItem = controlPanel.getSelectedPointFromComboBox();
        double parsedValue;

        try {
            if (currentText.isEmpty()) {
                // If "ALL" was selected and field showed "-- Varies --" and user cleared it
                if (Objects.equals(selectedItem, ControlPanel.ALL_POINTS_MARKER) && previousText.equals(ControlPanel.TEXTFIELD_VARIES_TEXT)) {
                    System.out.println("Field cleared for 'ALL' that showed '-- Varies --'. No update to points for this field.");
                    // The field will remain empty. If another selection happens, it will be re-populated.
                    // Or, we could force re-evaluation of "--Varies--" here, but it might be cleared by user.
                    // For now, let it be empty. Selection change will refresh.
                    textFieldPreviousValues.put(textField, currentText); // Store the empty state
                    return;
                }
                throw new NumberFormatException("Parameter cannot be empty."); // Empty is invalid for a specific point or for "ALL" when setting a value
            }
            parsedValue = Double.parseDouble(currentText);

            // Add specific range validation if desired
            if (textField == controlPanel.getMoveSpeedField() && parsedValue <= 0) throw new NumberFormatException("Move speed must be > 0");
            if (textField == controlPanel.getTurnSpeedField() && parsedValue <= 0) throw new NumberFormatException("Turn speed must be > 0");
            if (textField == controlPanel.getFollowDistanceField() && parsedValue < 0) throw new NumberFormatException("Follow distance must be >= 0");
            if (textField == controlPanel.getPointLengthField() && parsedValue <= 0) throw new NumberFormatException("Point length must be > 0");
            if (textField == controlPanel.getSlowDownTurnAmountField() && (parsedValue < 0 || parsedValue > 1)) throw new NumberFormatException("Slow down amount must be 0.0-1.0");
            // No specific range for slowDownTurnDegreesField here, but it's converted to radians.

        } catch (NumberFormatException e) {
            instructionLabel.setText("Invalid input: " + e.getMessage() + ". Reverting.");
            textField.setText(previousText); // Revert
            textFieldPreviousValues.put(textField, previousText); // Ensure map is correct
            return;
        }

        boolean updateOccurred = false;
        if (Objects.equals(selectedItem, ControlPanel.ALL_POINTS_MARKER)) {
            for (CurvePoint point : currentPath) {
                updateCurvePointParameter(point, textField, parsedValue);
            }
            instructionLabel.setText("Applied '" + getFieldName(textField) + " = " + currentText + "' to all points.");
            updateOccurred = true;
            refreshParameterFieldsForAllSelected(); // Refresh all fields for "ALL" view, as one change might make others uniform or varied
        } else if (selectedItem instanceof CurvePoint) {
            CurvePoint point = (CurvePoint) selectedItem;
            updateCurvePointParameter(point, textField, parsedValue);
            int pointIndex = currentPath.indexOf(point) + 1;
            instructionLabel.setText("Updated Point " + pointIndex + " (" + getFieldName(textField) + " = " + currentText + ").");
            updateOccurred = true;
        }

        if (updateOccurred) {
            textFieldPreviousValues.put(textField, currentText); // Update stored value after successful application
            fieldDisplay.drawCurrentState(); // Redraw path if parameters affect appearance (though unlikely for these)
        }
    }

    private void updateCurvePointParameter(CurvePoint point, TextField changedField, double value) {
        if (changedField == controlPanel.getMoveSpeedField()) point.moveSpeed = value;
        else if (changedField == controlPanel.getTurnSpeedField()) point.turnSpeed = value;
        else if (changedField == controlPanel.getFollowDistanceField()) point.followDistance = value;
        else if (changedField == controlPanel.getPointLengthField()) point.pointLength = value;
        else if (changedField == controlPanel.getSlowDownTurnDegreesField()) point.slowDownTurnRadians = Math.toRadians(value);
        else if (changedField == controlPanel.getSlowDownTurnAmountField()) point.slowDownTurnAmount = value;
    }

    private String getFieldName(TextField textField) {
        if (textField == controlPanel.getMoveSpeedField()) return "Move Speed";
        if (textField == controlPanel.getTurnSpeedField()) return "Turn Speed";
        if (textField == controlPanel.getFollowDistanceField()) return "Follow Distance";
        if (textField == controlPanel.getPointLengthField()) return "Point Length";
        if (textField == controlPanel.getSlowDownTurnDegreesField()) return "Slow Turn Deg";
        if (textField == controlPanel.getSlowDownTurnAmountField()) return "Slow Turn Amt";
        return "Parameter";
    }

    private void handlePointSelectionChanged(ObservableValue<? extends Object> obs, Object oldVal, Object newVal) {
        if (controlPanel == null || isCreatingPath) { // Do not change display if actively drawing path
            if (isCreatingPath && newVal != null && controlPanel.getSelectedPointFromComboBox() != null) {
                // If drawing path and user somehow clicks ComboBox, try to revert to original selection if possible
                // This is a bit defensive, ideally ComboBox is disabled during path creation
                Platform.runLater(() -> controlPanel.updatePointSelectionComboBox(currentPath, oldVal != null ? oldVal : ControlPanel.ALL_POINTS_MARKER));
            }
            return;
        }

        CurvePoint pointToHighlight = null; // Initialize to null (no highlight)

        if (newVal == null) {
            if (currentPath.isEmpty()) {
                controlPanel.loadGlobalDefaultsIntoParameterFields();
                controlPanel.setPointEditingControlsDisabled(true);
            } else {
                // Should default to "ALL" if path exists
                controlPanel.updatePointSelectionComboBox(currentPath, ControlPanel.ALL_POINTS_MARKER);
                // This will re-trigger the listener with "ALL"
            }
            return;
        } else if (Objects.equals(newVal, ControlPanel.ALL_POINTS_MARKER)) {
            refreshParameterFieldsForAllSelected();
            // When "ALL" is selected, no specific point is highlighted
        } else if (newVal instanceof CurvePoint) {
            CurvePoint selectedCurvePoint = (CurvePoint) newVal;
            controlPanel.loadParametersForPoint(selectedCurvePoint);
            pointToHighlight = selectedCurvePoint; // This is the point to highlight
        }

        System.out.println("ComboBox selection changed to: " + newVal);
        if (Objects.equals(newVal, ControlPanel.ALL_POINTS_MARKER)) {
            refreshParameterFieldsForAllSelected();
        } else if (newVal instanceof CurvePoint) {
            controlPanel.loadParametersForPoint((CurvePoint) newVal);
        }

        // Update the highlighted point in FieldDisplay and redraw
        if (fieldDisplay != null) {
            fieldDisplay.setHighlightedPoint(pointToHighlight);
            fieldDisplay.drawCurrentState(); // Redraw the field to show highlight
        }
    }

    private void refreshParameterFieldsForAllSelected() {
        if (controlPanel == null) return;
        if (currentPath.isEmpty()) {
            controlPanel.loadGlobalDefaultsIntoParameterFields();
            // Point editing controls should be disabled by updateControlPanelForPathState
            return;
        }

        System.out.println("Refreshing for ALL points. Path size: " + currentPath.size());
        checkAndSetField(currentPath, cp -> cp.moveSpeed, controlPanel.getMoveSpeedField(), "%.2f");
        checkAndSetField(currentPath, cp -> cp.turnSpeed, controlPanel.getTurnSpeedField(), "%.2f");
        checkAndSetField(currentPath, cp -> cp.followDistance, controlPanel.getFollowDistanceField(), "%.1f");
        checkAndSetField(currentPath, cp -> cp.pointLength, controlPanel.getPointLengthField(), "%.1f");
        checkAndSetField(currentPath, cp -> Math.toDegrees(cp.slowDownTurnRadians), controlPanel.getSlowDownTurnDegreesField(), "%.1f");
        checkAndSetField(currentPath, cp -> cp.slowDownTurnAmount, controlPanel.getSlowDownTurnAmountField(), "%.2f");
    }

    private <T> void checkAndSetField(List<CurvePoint> path, Function<CurvePoint, T> getter, TextField field, String format) {
        if (path.isEmpty()) { // Should be handled by calling context
            loadSpecificGlobalDefault(field); // Fallback
            return;
        }

        T firstValue = getter.apply(path.get(0));
        boolean allSame = true;
        for (int i = 1; i < path.size(); i++) {
            T currentValue = getter.apply(path.get(i));
            if (currentValue instanceof Double && firstValue instanceof Double) {
                if (Math.abs((Double) currentValue - (Double) firstValue) > 0.0001) { // Tolerance for double comparison
                    allSame = false;
                    break;
                }
            } else if (!Objects.equals(currentValue, firstValue)) {
                allSame = false;
                break;
            }
        }

        if (allSame) {
            if (firstValue instanceof Double) {
                field.setText(String.format(Locale.US, format, (Double) firstValue));
            } else { // Should not happen with current parameters but good for generic
                field.setText(firstValue.toString());
            }
        } else {
            field.setText(ControlPanel.TEXTFIELD_VARIES_TEXT);
        }
        textFieldPreviousValues.put(field, field.getText()); // Update stored value for focus lost checks
    }

    private void loadSpecificGlobalDefault(TextField field) {
        // Accessing static defaults from ControlPanel directly
        if (field == controlPanel.getMoveSpeedField()) field.setText(ControlPanel.DEFAULT_MOVE_SPEED);
        else if (field == controlPanel.getTurnSpeedField()) field.setText(ControlPanel.DEFAULT_TURN_SPEED);
        else if (field == controlPanel.getFollowDistanceField()) field.setText(ControlPanel.DEFAULT_FOLLOW_DISTANCE);
        else if (field == controlPanel.getPointLengthField()) field.setText(ControlPanel.DEFAULT_POINT_LENGTH);
        else if (field == controlPanel.getSlowDownTurnDegreesField()) field.setText(ControlPanel.DEFAULT_SLOW_DOWN_TURN_DEGREES);
        else if (field == controlPanel.getSlowDownTurnAmountField()) field.setText(ControlPanel.DEFAULT_SLOW_DOWN_TURN_AMOUNT);
        else field.setText("");
        textFieldPreviousValues.put(field, field.getText());
    }

    private void updateControlPanelForPathState() {
        if (controlPanel == null) return;

        boolean pathExistsAndNotEmpty = !currentPath.isEmpty();
        controlPanel.setPointEditingControlsDisabled(!pathExistsAndNotEmpty);
        controlPanel.enablePathControls(pathExistsAndNotEmpty); // For Delete, Export, Send buttons

        Object selectionToRestore = controlPanel.getSelectedPointFromComboBox();
        if (!pathExistsAndNotEmpty) {
            selectionToRestore = ControlPanel.ALL_POINTS_MARKER; // Show ALL (which will load defaults) if no path
        } else {
            // If current selection is a CurvePoint but no longer in the path (e.g., path was cleared and a new short one made)
            if (selectionToRestore instanceof CurvePoint && !currentPath.contains(selectionToRestore)) {
                selectionToRestore = ControlPanel.ALL_POINTS_MARKER;
            } else if (selectionToRestore == null) { // No selection yet, default to ALL if path exists
                selectionToRestore = ControlPanel.ALL_POINTS_MARKER;
            }
        }
        controlPanel.updatePointSelectionComboBox(currentPath, selectionToRestore);

        // After ComboBox is updated, its value change listener (handlePointSelectionChanged)
        // should take care of populating the TextFields correctly.
        // So, explicitly calling refreshParameterFieldsForAllSelected or loadParametersForPoint here
        // might be redundant if the ComboBox listener is robust.
        // Let's ensure the listener is triggered or manually trigger if needed.
        Object currentSelectionAfterUpdate = controlPanel.getSelectedPointFromComboBox();
        if (currentSelectionAfterUpdate == null && pathExistsAndNotEmpty) {
            // This case should ideally not happen if updatePointSelectionComboBox defaults to "ALL"
            controlPanel.updatePointSelectionComboBox(currentPath, ControlPanel.ALL_POINTS_MARKER);
        } else {
            // Manually refresh based on current combo box state just in case listener didn't cover all edge cases on init
            if (Objects.equals(currentSelectionAfterUpdate, ControlPanel.ALL_POINTS_MARKER)) {
                refreshParameterFieldsForAllSelected();
            } else if (currentSelectionAfterUpdate instanceof CurvePoint) {
                controlPanel.loadParametersForPoint((CurvePoint) currentSelectionAfterUpdate);
            } else { // No path or null selection
                controlPanel.loadGlobalDefaultsIntoParameterFields();
            }
        }
        // If path creation is active, disable point editing controls again
        if (isCreatingPath) {
            controlPanel.setPointEditingControlsDisabled(true);
        }
    }

    private void handleSendPathToRobot() {
        if (currentPath.isEmpty()) {
            instructionLabel.setText("No path to send.");
            System.out.println("Attempted to send path, but currentPath is empty.");
            return;
        }
        instructionLabel.setText("Sending path to robot...");
        System.out.println("Preparing to send " + currentPath.size() + " points to robot at " + ROBOT_IP_ADDRESS + ":" + ROBOT_LISTENER_PORT);
        try (DatagramSocket socket = new DatagramSocket()) {
            InetAddress address = InetAddress.getByName(ROBOT_IP_ADDRESS);
            for (CurvePoint point : currentPath) {
                String message = String.format(Locale.US, "curve_point:%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f",
                        point.x, point.y, point.moveSpeed, point.turnSpeed,
                        point.followDistance, point.pointLength,
                        point.slowDownTurnRadians, point.slowDownTurnAmount);
                byte[] buffer = message.getBytes(StandardCharsets.UTF_8);
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, ROBOT_LISTENER_PORT);
                socket.send(packet);
                System.out.println("Sent: " + message);
            }
            String endMessage = "end";
            byte[] endBuffer = endMessage.getBytes(StandardCharsets.UTF_8);
            DatagramPacket endPacket = new DatagramPacket(endBuffer, endBuffer.length, address, ROBOT_LISTENER_PORT);
            socket.send(endPacket);
            System.out.println("Sent: " + endMessage);
            instructionLabel.setText("Path sent successfully to " + ROBOT_IP_ADDRESS);
            System.out.println("Path sending complete.");
        } catch (IOException e) {
            instructionLabel.setText("Error sending path: " + e.getMessage());
            System.err.println("Error sending path to robot: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void exportPathToCSV(Stage ownerStage) {
        if (currentPath.isEmpty()) {
            instructionLabel.setText("No path to export.");
            return;
        }
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Save Path to CSV");
        fileChooser.setInitialFileName("ftc_path.csv");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("CSV Files (*.csv)", "*.csv"));
        File file = fileChooser.showSaveDialog(ownerStage);
        if (file != null) {
            try (PrintWriter writer = new PrintWriter(file)) {
                writer.println("x_inches,y_inches"); // Header
                for (CurvePoint point : currentPath) {
                    writer.printf(Locale.US, "%.3f,%.3f\n", point.x, point.y);
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
        isCreatingPath = false;
        fieldDisplay.setPathCreationMode(false, null, () -> finishPathCreation(false));
        fieldDisplay.setPathToDraw(currentPath);
        fieldDisplay.drawCurrentState();
        fieldDisplay.setHighlightedPoint(null);
        instructionLabel.setText("Path deleted. Click 'New Path' to start drawing.");
        updateControlPanelForPathState();
    }

    private void finishPathCreation(boolean cancelled) {
        if (!isCreatingPath) return; // Guard against multiple calls

        isCreatingPath = false; // Path creation is officially over
        fieldDisplay.setPathCreationMode(false, null, null); // Stop field listening

        if (cancelled && currentPath != null) { // Null check for currentPath
            currentPath.clear();
            instructionLabel.setText("Path creation cancelled. Click 'New Path' to start again.");
        } else {
            if (currentPath == null || currentPath.isEmpty()) { // Null check
                instructionLabel.setText("Path finished with no points. Click 'New Path' to start again.");
                if (currentPath != null) currentPath.clear(); // Ensure it's empty if it was just null
                else currentPath = new ArrayList<>(); // Initialize if null
            } else {
                instructionLabel.setText("Path finished with " + currentPath.size() + " points. Select points to edit parameters.");
            }
        }

        // Explicitly re-enable the "New Path" button and allow others to be controlled by path existence.
        if (controlPanel != null) {
            controlPanel.setPathEditingActive(false);
        }

        // This will now correctly update the state of "Delete", "Export", "Send"
        // because newPathButton will be seen as enabled by enablePathControls.
        // It will also update the ComboBox and parameter fields.
        updateControlPanelForPathState();

        // Update the visual path on the field
        if (fieldDisplay != null && currentPath != null) { // Null checks
            fieldDisplay.setPathToDraw(currentPath);
            fieldDisplay.drawCurrentState();
        }
    }

    private void handleFieldClickForPath(Point2D pixelCoords) {
        if (!isCreatingPath || controlPanel == null) return;
        Point2D inchesCoordsFieldCenter = fieldDisplay.pixelToInches(pixelCoords.getX(), pixelCoords.getY());
        double moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnDeg, slowDownTurnAmount, slowDownTurnRad;
        try {
            moveSpeed = controlPanel.getMoveSpeedParam();
            turnSpeed = controlPanel.getTurnSpeedParam();
            followDistance = controlPanel.getFollowDistanceParam();
            pointLength = controlPanel.getPointLengthParam();
            slowDownTurnDeg = controlPanel.getSlowDownTurnDegreesParam();
            slowDownTurnAmount = controlPanel.getSlowDownTurnAmountParam();
            if (moveSpeed <= 0 || turnSpeed <= 0 || followDistance < 0 || pointLength <= 0 || slowDownTurnAmount < 0 || slowDownTurnAmount > 1) {
                throw new NumberFormatException("Default parameter out of typical range.");
            }
            slowDownTurnRad = Math.toRadians(slowDownTurnDeg);
        } catch (NumberFormatException e) {
            instructionLabel.setText("Invalid global default parameter. Using master defaults for new point.");
            System.err.println("Error parsing global default parameters: " + e.getMessage() + ". Using master defaults.");
            moveSpeed = MASTER_DEFAULT_MOVE_SPEED;
            turnSpeed = MASTER_DEFAULT_TURN_SPEED;
            followDistance = MASTER_DEFAULT_FOLLOW_DISTANCE;
            pointLength = MASTER_DEFAULT_POINT_LENGTH;
            slowDownTurnRad = MASTER_DEFAULT_SLOW_DOWN_TURN_RADIANS;
            slowDownTurnAmount = MASTER_DEFAULT_SLOW_DOWN_TURN_AMOUNT;
        }
        CurvePoint newPoint = new CurvePoint(
                inchesCoordsFieldCenter.getX(), inchesCoordsFieldCenter.getY(),
                moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRad, slowDownTurnAmount
        );
        currentPath.add(newPoint);
        fieldDisplay.setPathToDraw(currentPath);
        fieldDisplay.drawCurrentState();
        if (currentPath.size() == 1) {
            instructionLabel.setText("Point 1 added. Click next waypoint. ESC to cancel.");
        } else {
            instructionLabel.setText("Point " + currentPath.size() + " added. Click next, Double-click last, or ESC to cancel.");
        }
    }

    private void updateTimeLapsedDisplay() {
        if (controlPanel == null || recordingManager == null) return;

        long timeLapsedMs;
        if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
            // If recording, show time since recording started
            timeLapsedMs = recordingManager.getCurrentRecordingDuration(); // Assumes you add such a method to RecordingManager
            // or calculate it based on System.time - recordingStartTime
        } else {
            timeLapsedMs = recordingManager.getCurrentEventTimeLapsed();
        }
        controlPanel.updateTimeLapsed(timeLapsedMs);
    }

    private void startNewPathCreation() {
        if (isCreatingPath) return; // Already in this mode

        if (currentPath == null) { // Defensive initialization
            currentPath = new ArrayList<>();
        }
        currentPath.clear();
        isCreatingPath = true;
        fieldDisplay.setHighlightedPoint(null);

        // Mode change: Disable "New Path", "Delete", "Export", "Send"
        // and enable "Finish Path" / "Cancel Path" implicitly by UI context.
        if (controlPanel != null) {
            controlPanel.setPathEditingActive(true);
        }

        // Update other UI elements:
        // - ComboBox for points should be cleared/disabled.
        // - Parameter fields should show global defaults but be disabled.
        // - "Delete", "Export", "Send" will be further confirmed as disabled by enablePathControls(false)
        updateControlPanelForPathState();

        instructionLabel.setText("Click the first waypoint. Parameters from global defaults will be used.");
        if (fieldDisplay != null) {
            fieldDisplay.setPathCreationMode(true, this::handleFieldClickForPath, () -> finishPathCreation(false));
            fieldDisplay.setPathToDraw(currentPath); // Show path as it's built
            fieldDisplay.drawCurrentState();
        }
    }

    // --- Recording, UDP, and Other Utility Methods (ensure these are complete from your original) ---
    private void setupRecordingControlActions() {
        controlPanel.setOnOpenAction(e -> handleOpenRecording());
        controlPanel.setOnSaveAction(e -> handleSaveRecording());
        controlPanel.setOnRecordAction(() -> {
            RecordingManager.PlaybackState recordingState = recordingManager.getCurrentState();
            if (recordingState == RecordingManager.PlaybackState.RECORDING) {
                recordingManager.stopRecording();
                controlPanel.toggleRecordButtonIcon(false);
                if (recordingManager.hasRecording()) {
                    controlPanel.setPlaybackControlsDisabled(false);
                    controlPanel.updateTimelineSlider(recordingManager.getPlaybackIndex(), recordingManager.getTotalEvents());
                    controlPanel.setSaveButtonDisabled(false);
                } else {
                    controlPanel.setPlaybackControlsDisabled(true);
                    controlPanel.setSaveButtonDisabled(true);
                }
            } else {
                controlPanel.togglePlayPauseButtonIcon(false);
                recordingManager.startRecording();
                controlPanel.toggleRecordButtonIcon(true);
                controlPanel.setPlaybackControlsDisabled(true);
                controlPanel.setSaveButtonDisabled(true);
            }
            updateTimeLapsedDisplay();
        });
        controlPanel.setOnPlayPauseAction(() -> {
            RecordingManager.PlaybackState playbackState = recordingManager.getCurrentState();
            if (playbackState == RecordingManager.PlaybackState.PLAYING) {
                recordingManager.pause();
                controlPanel.togglePlayPauseButtonIcon(false);
            } else if (playbackState == RecordingManager.PlaybackState.IDLE || playbackState == RecordingManager.PlaybackState.PAUSED) {
                if (recordingManager.hasRecording()) {
                    recordingManager.play();
                    controlPanel.togglePlayPauseButtonIcon(true);
                }
            }
            updateTimeLapsedDisplay();
        });
        controlPanel.setOnForwardAction(() -> {
            if (recordingManager.hasRecording() && recordingManager.getCurrentState() != RecordingManager.PlaybackState.RECORDING) {
                recordingManager.stepForward();
                controlPanel.togglePlayPauseButtonIcon(false);
            }
            updateTimeLapsedDisplay();
        });
        controlPanel.setOnReverseAction(() -> {
            if (recordingManager.hasRecording() && recordingManager.getCurrentState() != RecordingManager.PlaybackState.RECORDING) {
                recordingManager.stepBackward();
                controlPanel.togglePlayPauseButtonIcon(false);
            }
            updateTimeLapsedDisplay();
        });
        controlPanel.setOnTimelineSliderChanged((observable, oldValue, newValue) -> {
            if (controlPanel.getTimelineSlider().isValueChanging() && recordingManager.getCurrentState() != RecordingManager.PlaybackState.PLAYING) {
                recordingManager.seekTo(newValue.intValue());
                controlPanel.togglePlayPauseButtonIcon(false);
                updateTimeLapsedDisplay();
            }
        });
        controlPanel.setOnSliderMouseReleased(event -> {
            if (recordingManager.getCurrentState() != RecordingManager.PlaybackState.PLAYING &&
                    recordingManager.getCurrentState() != RecordingManager.PlaybackState.RECORDING) {

                int sliderRawValue = (int) controlPanel.getTimelineSlider().getValue();

                // The FieldDisplay might need to be cleared before a potentially large jump
                // if (fieldDisplay != null) {
                // fieldDisplay.clearDrawingSurface(); // Or similar method
                // }

                recordingManager.seekTo(sliderRawValue); // seekTo will now handle snapping and dispatching

                // After seekTo, playbackIndex in RecordingManager is at the snapped position.
                // The dispatchCurrentEvent calls within seekTo would have updated the
                // onProgressUpdateCallback, which in turn should update the slider's visual position
                // to the new playbackIndex and the time label.
                // So, an explicit controlPanel.getTimelineSlider().setValue() here might cause a flicker
                // if onProgressUpdate also does it. Let onProgressUpdate be the source of truth for UI sync.

                controlPanel.togglePlayPauseButtonIcon(false); // Ensure UI consistency
            }
            // updateTimeLapsedDisplay(); // Should be handled by onProgressUpdate from seekTo
        });


    }

    private void handleSaveRecording() {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Save Recording");
        fileChooser.setInitialFileName("recording.rec");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
        File file = fileChooser.showSaveDialog(primaryStage);
        if (file == null) { instructionLabel.setText("Save cancelled."); return; }
        ArrayList<RecordingManager.RecordedEvent> events = recordingManager.getRecordedSession();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            for (RecordingManager.RecordedEvent event : events) {
                String line = formatEventToString(event);
                if (line != null) {
                    writer.write(line);
                    writer.newLine();
                }
            }
            instructionLabel.setText("Recording saved: " + file.getName());
        } catch (IOException e) {
            instructionLabel.setText("Error saving recording: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private String formatEventToString(RecordingManager.RecordedEvent event) {
        String payload; UdpMessageData data = event.messageData;
        if (data instanceof PositionData) { PositionData d = (PositionData) data; payload = String.format(Locale.US,"pos:%.3f,%.3f,%.3f", d.x, d.y, d.heading); }
        else if (data instanceof CircleData) { CircleData d = (CircleData) data; payload = String.format(Locale.US,"cir:%.3f,%.3f", d.radiusInches, d.heading); }
        else if (data instanceof LineData) { LineData d = (LineData) data; payload = String.format(Locale.US,"line:%s,%.3f,%.3f,%.3f,%.3f,%d", d.name, d.x1, d.y1, d.x2, d.y2, d.styleCode); }
        else if (data instanceof TextData) { TextData d = (TextData) data; payload = "txt:" + d.text; }
        else if (data instanceof KeyValueData) {
            KeyValueData kv = (KeyValueData) data;
            // Simple escaping for now to handle potential issues in the value string
            payload = String.format("kv:%s,%s", kv.key, kv.value);
        }
        else { return null; }
        return event.timestamp + "|" + payload;
    }

    private void handleOpenRecording() {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Open Recording");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
        File file = fileChooser.showOpenDialog(primaryStage);
        if (file == null) { instructionLabel.setText("Open cancelled."); return; }
        ArrayList<RecordingManager.RecordedEvent> loadedEvents = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] lineParts = line.split("\\|", 2);
                if (lineParts.length != 2) continue;
                long timestamp = Long.parseLong(lineParts[0]); String payload = lineParts[1];
                UdpMessageData parsedData = null;
                if (payload.startsWith("pos:")) { String c = payload.substring(4); String[] p = c.split(","); if (p.length == 3) parsedData = new PositionData(Double.parseDouble(p[0]), Double.parseDouble(p[1]), Double.parseDouble(p[2])); }
                else if (payload.startsWith("cir:")) { String c = payload.substring(4); String[] p = c.split(","); if (p.length == 2) parsedData = new CircleData(Double.parseDouble(p[0]), Double.parseDouble(p[1])); }
                else if (payload.startsWith("line:")) { String c = payload.substring(5); String[] p = c.split(",", 6); if (p.length == 6) parsedData = new LineData(p[0], Double.parseDouble(p[1]), Double.parseDouble(p[2]), Double.parseDouble(p[3]), Double.parseDouble(p[4]), Integer.parseInt(p[5]));}
                else if (payload.startsWith("txt:")) { parsedData = new TextData(payload.substring(4));}
                else if (payload.startsWith("kv:")) {
                    String c = payload.substring(3);
                    String[] p = c.split(",", 2);
                    if (p.length == 2) parsedData = new KeyValueData(p[0], p[1]);
                }
                if (parsedData != null) loadedEvents.add(new RecordingManager.RecordedEvent(timestamp, parsedData));
            }
            recordingManager.loadRecording(loadedEvents);
            controlPanel.setPlaybackControlsDisabled(loadedEvents.isEmpty());
            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
            controlPanel.setSaveButtonDisabled(loadedEvents.isEmpty());
            controlPanel.togglePlayPauseButtonIcon(false);
            instructionLabel.setText("Opened: " + file.getName());
            updateTimeLapsedDisplay();
        } catch (Exception e) {
            instructionLabel.setText("Error opening recording: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void handleUdpMessage(UdpMessageData messageData) {
        if (messageData == null) return;
        Platform.runLater(() -> {
            if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) recordingManager.addEvent(messageData);
            if (messageData instanceof PositionData) { PositionData p = (PositionData)messageData; if(robot!=null){ fieldDisplay.addTrailDot(robot.getXInches(),robot.getYInches()); robot.setPosition(p.x,p.y); robot.setHeading(p.heading);}}
            else if (messageData instanceof CircleData) { CircleData c = (CircleData)messageData; if(fieldDisplay!=null) fieldDisplay.addDebugCircle(robot.getXInches(),robot.getYInches(),c.radiusInches,c.heading,Color.rgb(255,165,0,0.7));}
            else if (messageData instanceof LineData) { LineData l = (LineData)messageData; synchronized(namedLinesLock){namedLinesToDraw.put(l.name,l);}}
            else if (messageData instanceof TextData) { TextData t = (TextData)messageData; if(fieldDisplay!=null) fieldDisplay.setRobotTextMessage(t.text);}
            else if (messageData instanceof KeyValueData) {
                KeyValueData kv = (KeyValueData) messageData;
                if (keyValueTable != null) {
                    keyValueTable.updateValue(kv.key, kv.value);
                }
            }
            if(messageData instanceof PositionData) updateUIFromRobotState(); else fieldDisplay.drawCurrentState();
        });
    }

    private void onPlaybackFinished() {
        Platform.runLater(() -> {
            if (controlPanel != null) {
                controlPanel.togglePlayPauseButtonIcon(false);
            }
            updateTimeLapsedDisplay();
        });
    }

    private void startUdpPositionListener() {
        try {
            udpListener = new UdpPositionListener(UDP_LISTENER_PORT, this::handleUdpMessage);
            udpListenerThread = new Thread(udpListener, "UdpListenerThread");
            udpListenerThread.setDaemon(true);
            udpListenerThread.start();
            System.out.println("UDP Listener started on port " + UDP_LISTENER_PORT);
        } catch (Exception e) {
            instructionLabel.setText("ERROR: UDP Listener start failed on " + UDP_LISTENER_PORT);
            e.printStackTrace();
        }
    }

    private void startUdpPlotListener() {
        try {
            udpPlotListener = new UdpPlotListener(this::handleUdpPlotData); // Uses default port from UdpPlotListener
            udpPlotListenerThread = new Thread(udpPlotListener, "UdpPlotListenerThread");
            udpPlotListenerThread.setDaemon(true);
            udpPlotListenerThread.start();
            System.out.println("UDP Plot Listener started on port " + UdpPlotListener.DEFAULT_PLOT_LISTENER_PORT);
        } catch (Exception e) {
            // You might want a different label or way to show this error if instructionLabel is for field sim
            System.err.println("ERROR: UDP Plot Listener start failed on " + UdpPlotListener.DEFAULT_PLOT_LISTENER_PORT + " - " + e.getMessage());
            if (instructionLabel != null) { // Be cautious if this runs before instructionLabel is ready
                instructionLabel.setText("ERROR: Plot Listener failed");
            }
            e.printStackTrace();
        }
    }

    private void stopApp() {
        if (udpListener != null) udpListener.stopListener();
        if (udpPlotListener != null) udpPlotListener.stopListener();

        if (udpListenerThread != null && udpListenerThread.isAlive()) {
            try {
                udpListenerThread.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (udpPlotListenerThread != null && udpPlotListenerThread.isAlive()) {
            try {
                udpPlotListenerThread.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (plotDisplayWindow != null && plotDisplayWindow.isShowing()){ // Close plot window if open
            plotDisplayWindow.hide();
        }
        System.out.println("Exiting application.");
        Platform.exit();
        System.exit(0);
    }

    private void clearAllNamedLines() {
        synchronized (namedLinesLock) {
            namedLinesToDraw.clear();
        }
        if (fieldDisplay != null) {
            fieldDisplay.drawCurrentState();
        }
    }

    private void handleSceneKeyPress(KeyEvent event) {
        if (isCreatingPath) {
            if (event.getCode() == KeyCode.ESCAPE) {
                finishPathCreation(true); // true for cancelled
                event.consume();
            }
        } else {
            handleRobotMovementKeyPress(event);
        }
    }

    private void handleRobotMovementKeyPress(KeyEvent event) {
        if (robot == null || isCreatingPath) { // Do not move robot if path creation is active
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
            case A: // Strafe Left
                double strafeLeftAngleRad_CCW = Math.toRadians(currentHeading_CCW + 90.0);
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeLeftAngleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeLeftAngleRad_CCW);
                moved = true;
                break;
            case D: // Strafe Right
                double strafeRightAngleRad_CCW = Math.toRadians(currentHeading_CCW - 90.0);
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeRightAngleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeRightAngleRad_CCW);
                moved = true;
                break;
            default:
                // Not a movement key we handle here
                break;
        }

        if (moved) {
            if (event.getCode() == KeyCode.UP || event.getCode() == KeyCode.DOWN ||
                    event.getCode() == KeyCode.A || event.getCode() == KeyCode.D) {
                robot.setPosition(newFieldX, newFieldY);
            }
            updateUIFromRobotState();
            event.consume(); // Consume the event so it's not processed further (e.g., by focus traversal)
        }
    }
}



//package com.example.ftcfieldsimulator;
//
//import javafx.application.Application;
//import javafx.application.Platform;
//import javafx.beans.value.ChangeListener;
//import javafx.beans.value.ObservableValue;
//import javafx.geometry.Insets;
//import javafx.geometry.Point2D;
//import javafx.geometry.Pos;
//import javafx.scene.Scene;
//import javafx.scene.control.CheckBox;
//import javafx.scene.control.Label;
//import javafx.scene.control.TextField;
//import javafx.scene.input.KeyCode;
//import javafx.scene.input.KeyEvent;
//import javafx.scene.layout.BorderPane;
//import javafx.scene.layout.HBox;
//import javafx.scene.paint.Color;
//import javafx.stage.FileChooser;
//import javafx.stage.Stage;
//
//import com.example.ftcfieldsimulator.UdpPositionListener.CircleData;
//import com.example.ftcfieldsimulator.UdpPositionListener.LineData;
//import com.example.ftcfieldsimulator.UdpPositionListener.PositionData;
//import com.example.ftcfieldsimulator.UdpPositionListener.TextData;
//import com.example.ftcfieldsimulator.UdpPositionListener.UdpMessageData;
//import com.example.ftcfieldsimulator.UdpPositionListener.KeyValueData;
//
//import java.io.BufferedReader;
//import java.io.BufferedWriter;
//import java.io.File;
//import java.io.FileReader;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.io.PrintWriter;
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.InetAddress;
//import java.nio.charset.StandardCharsets;
//import java.util.ArrayList;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Locale;
//import java.util.Map;
//import java.util.Objects;
//import java.util.function.Function;
//
//public class FtcFieldSimulatorApp extends Application {
//
//    // --- UI and State Components ---
//    private FieldDisplay fieldDisplay;
//    private ControlPanel controlPanel;
//    private FieldKeyValueTable keyValueTable;
//    private Robot robot;
//    private RecordingManager recordingManager;
//    private UdpPositionListener udpListener;
//    private Thread udpListenerThread;
//    private Label instructionLabel;
//    private Stage primaryStage;
//    private PlotDisplayWindow plotDisplayWindow;
//    private UdpPlotListener udpPlotListener;
//    private Thread udpPlotListenerThread;
//
//    // --- Path and Line Management ---
//    private List<CurvePoint> currentPath = new ArrayList<>();
//    private boolean isCreatingPath = false;
//    private Map<String, LineData> namedLinesToDraw = new HashMap<>();
//    private final Object namedLinesLock = new Object();
//    private Map<TextField, String> textFieldPreviousValues = new HashMap<>();
//
//    // --- Configuration Constants ---
//    public static final double FIELD_WIDTH_INCHES = 144.0;
//    public static final double FIELD_HEIGHT_INCHES = 144.0;
//    private static final int FIELD_DISPLAY_WIDTH_PIXELS = 800;
//    private static final int FIELD_DISPLAY_HEIGHT_PIXELS = 800;
//    private static final String FIELD_IMAGE_PATH = "/decode_field.png";
//    private static final String ROBOT_IMAGE_PATH = "/robot.png";
//    public static final double FIELD_IMAGE_ALPHA = 0.3;
//    public static final double BACKGROUND_ALPHA = 0.1;
//    public static final double ROBOT_START_FIELD_X = 0.0;
//    public static final double ROBOT_START_FIELD_Y = 0.0;
//    public static final double ROBOT_START_HEADING_DEGREES = 0.0;
//    private static final int UDP_LISTENER_PORT = 7777;
//    private static final int ROBOT_LISTENER_PORT = 6666;
//    private static final String ROBOT_IP_ADDRESS = "192.168.43.1";
//    private static final double ROBOT_MOVE_INCREMENT_INCHES = 2.0;
//    private static final double ROBOT_TURN_INCREMENT_DEGREES = 5.0;
//
//    private static final double MASTER_DEFAULT_MOVE_SPEED = 0.4;
//    private static final double MASTER_DEFAULT_TURN_SPEED = 0.4;
//    private static final double MASTER_DEFAULT_FOLLOW_DISTANCE = 10.0;
//    private static final double MASTER_DEFAULT_POINT_LENGTH = 10.0;
//    private static final double MASTER_DEFAULT_SLOW_DOWN_TURN_RADIANS = Math.toRadians(60);
//    private static final double MASTER_DEFAULT_SLOW_DOWN_TURN_AMOUNT = 0.6;
//
//    public static void main(String[] args) {
//        Application.launch(args);
//    }
//
//    @Override
//    public void start(Stage primaryStage) {
//        this.primaryStage = primaryStage;
//        primaryStage.setTitle("FTC Field Simulator");
//
//        double controlPanelWidth = FIELD_DISPLAY_WIDTH_PIXELS * ControlPanel.PREFERRED_WIDTH_RATIO_TO_FIELD;
//        double keyValueTableWidth = controlPanelWidth * 0.8; // Make it slightly smaller than control panel
//        double totalAppWidth = FIELD_DISPLAY_WIDTH_PIXELS + controlPanelWidth + keyValueTableWidth;
//        double totalAppHeight = FIELD_DISPLAY_HEIGHT_PIXELS;
//
//        this.recordingManager = new RecordingManager(this::handleUdpMessage, this::onPlaybackFinished);
//        recordingManager.setOnProgressUpdate(index -> {
//            if (controlPanel != null && controlPanel.getTimelineSlider() != null && !controlPanel.getTimelineSlider().isValueChanging()) {
//                controlPanel.getTimelineSlider().setValue(index);
//            }
//            updateTimeLapsedDisplay();
//        });
//
//        this.robot = new Robot(ROBOT_START_FIELD_X, ROBOT_START_FIELD_Y, ROBOT_START_HEADING_DEGREES, ROBOT_IMAGE_PATH);
//        fieldDisplay = new FieldDisplay(FIELD_DISPLAY_WIDTH_PIXELS, FIELD_DISPLAY_HEIGHT_PIXELS, FIELD_WIDTH_INCHES, FIELD_HEIGHT_INCHES, FIELD_IMAGE_PATH, robot, BACKGROUND_ALPHA, FIELD_IMAGE_ALPHA);
//        fieldDisplay.setNamedLinesMap(namedLinesToDraw);
//        fieldDisplay.setRobotTextMessage(null);
//
//        controlPanel = new ControlPanel(controlPanelWidth);
//        keyValueTable = new FieldKeyValueTable(keyValueTableWidth);
//        instructionLabel = new Label("Create a new path or select a point to edit its parameters.");
//        instructionLabel.setPadding(new Insets(5));
//        instructionLabel.setMaxWidth(Double.MAX_VALUE);
//        instructionLabel.setAlignment(Pos.CENTER);
//        HBox instructionPane = new HBox(instructionLabel);
//        instructionPane.setAlignment(Pos.CENTER);
//        instructionPane.setStyle("-fx-background-color: #CFD8DC;");
//
//        BorderPane mainLayout = new BorderPane();
//        mainLayout.setLeft(controlPanel);
//        mainLayout.setCenter(fieldDisplay);
//        mainLayout.setRight(keyValueTable);
//        mainLayout.setBottom(instructionPane);
//
//        double instructionPaneHeight = 30;
//        Scene scene = new Scene(mainLayout, totalAppWidth, totalAppHeight + instructionPaneHeight);
//        scene.addEventFilter(KeyEvent.KEY_PRESSED, this::handleSceneKeyPress);
//
//        setupControlPanelActions(primaryStage);
//        setupRecordingControlActions();
//        setupParameterFieldListeners();
//
//        startUdpPositionListener();
//        startUdpPlotListener();
//        primaryStage.setScene(scene);
//        primaryStage.setResizable(true);
//        primaryStage.show();
//        primaryStage.setOnCloseRequest(event -> stopApp());
//
//        updateUIFromRobotState();
//        updateControlPanelForPathState();
//        updateTimeLapsedDisplay();
//    }
//
//    private void showPlotDisplay() {
//        if (plotDisplayWindow == null) {
//            plotDisplayWindow = new PlotDisplayWindow(primaryStage); // Pass primary stage as owner
//        }
//        plotDisplayWindow.show();
//    }
//    private void handleUdpPlotData(PlotDataEvent dataEvent) {
//        if (dataEvent == null) return;
//
//        Platform.runLater(() -> {
//            // Log all received plot data for now
//            // System.out.println("App received PlotDataEvent: " + dataEvent);
//
//            // If PlotDisplayWindow is not yet created, create it.
//            // This ensures if data comes before user clicks button, window can still be prepared.
//            // However, it won't show until user clicks.
//            if (plotDisplayWindow == null) {
//                // plotDisplayWindow = new PlotDisplayWindow(primaryStage); // Create but don't show
//                // Let's only interact if it's already created and showing by user action
//            }
//
//            if (plotDisplayWindow != null && plotDisplayWindow.isShowing()) {
//                PlotDisplay display = plotDisplayWindow.getPlotDisplay();
//                if (display != null) {
//                    // Pass the generic event. PlotDisplay will decide what to do.
//                    display.addPlotEvent(dataEvent);
//                }
//            } else if (dataEvent instanceof PlotYLimitsEvent || dataEvent instanceof PlotYUnitsEvent) {
//                // Optional: If plot window isn't visible, maybe still store these global settings
//                // so when it becomes visible, it uses the latest known ones.
//                // For now, we only update if visible.
//                System.out.println("Plot window not visible. Discarding: " + dataEvent);
//            }
//        });
//    }
//
//    private void setupControlPanelActions(Stage ownerStage) {
//        controlPanel.setOnNewPathAction(event -> startNewPathCreation());
//        controlPanel.setOnDeletePathAction(event -> deleteCurrentPath());
//        controlPanel.setOnExportPathAction(event -> exportPathToCSV(ownerStage));
//        controlPanel.setOnSendPathAction(event -> handleSendPathToRobot());
//        controlPanel.setOnClearTrailAction(event -> {
//            fieldDisplay.clearTrail();
//            fieldDisplay.drawCurrentState();
//            instructionLabel.setText("Robot trail cleared.");
//        });
//        controlPanel.setOnClearNamedLinesAction(event -> {
//            clearAllNamedLines();
//            instructionLabel.setText("All custom lines cleared.");
//        });
//        controlPanel.setOnPointSelectionAction(this::handlePointSelectionChanged);
//        controlPanel.setOnShowPlotAction(event -> showPlotDisplay());
//    }
//
//    private void setupParameterFieldListeners() {
//        if (controlPanel == null) return;
//        for (TextField tf : controlPanel.getAllParamTextFields()) {
//            tf.focusedProperty().addListener((obs, oldVal, newVal) -> {
//                if (newVal) {
//                    textFieldPreviousValues.put(tf, tf.getText());
//                } else {
//                    handleParameterFieldFocusLost(tf);
//                }
//            });
//            tf.setOnAction(event -> handleParameterFieldFocusLost(tf));
//        }
//    }
//
//    private void handleParameterFieldFocusLost(TextField textField) {
//        if (controlPanel == null || currentPath == null) return;
//
//        String previousText = textFieldPreviousValues.getOrDefault(textField, "");
//        String currentText = textField.getText().trim(); // Trim whitespace
//
//        if (Objects.equals(previousText, currentText) && !previousText.equals(ControlPanel.TEXTFIELD_VARIES_TEXT)) {
//            return; // No actual change, or was "-- Varies --" and still is (though user can't type that directly)
//        }
//        // If currentText becomes the "-- Varies --" text, it's likely a programmatic change, not user input for commit.
//        if (currentText.equals(ControlPanel.TEXTFIELD_VARIES_TEXT)) {
//            textFieldPreviousValues.put(textField, currentText); // Update stored value if it was set programmatically
//            return;
//        }
//
//        Object selectedItem = controlPanel.getSelectedPointFromComboBox();
//        double parsedValue;
//
//        try {
//            if (currentText.isEmpty()) {
//                // If "ALL" was selected and field showed "-- Varies --" and user cleared it
//                if (Objects.equals(selectedItem, ControlPanel.ALL_POINTS_MARKER) && previousText.equals(ControlPanel.TEXTFIELD_VARIES_TEXT)) {
//                    System.out.println("Field cleared for 'ALL' that showed '-- Varies --'. No update to points for this field.");
//                    // The field will remain empty. If another selection happens, it will be re-populated.
//                    // Or, we could force re-evaluation of "--Varies--" here, but it might be cleared by user.
//                    // For now, let it be empty. Selection change will refresh.
//                    textFieldPreviousValues.put(textField, currentText); // Store the empty state
//                    return;
//                }
//                throw new NumberFormatException("Parameter cannot be empty."); // Empty is invalid for a specific point or for "ALL" when setting a value
//            }
//            parsedValue = Double.parseDouble(currentText);
//
//            // Add specific range validation if desired
//            if (textField == controlPanel.getMoveSpeedField() && parsedValue <= 0) throw new NumberFormatException("Move speed must be > 0");
//            if (textField == controlPanel.getTurnSpeedField() && parsedValue <= 0) throw new NumberFormatException("Turn speed must be > 0");
//            if (textField == controlPanel.getFollowDistanceField() && parsedValue < 0) throw new NumberFormatException("Follow distance must be >= 0");
//            if (textField == controlPanel.getPointLengthField() && parsedValue <= 0) throw new NumberFormatException("Point length must be > 0");
//            if (textField == controlPanel.getSlowDownTurnAmountField() && (parsedValue < 0 || parsedValue > 1)) throw new NumberFormatException("Slow down amount must be 0.0-1.0");
//            // No specific range for slowDownTurnDegreesField here, but it's converted to radians.
//
//        } catch (NumberFormatException e) {
//            instructionLabel.setText("Invalid input: " + e.getMessage() + ". Reverting.");
//            textField.setText(previousText); // Revert
//            textFieldPreviousValues.put(textField, previousText); // Ensure map is correct
//            return;
//        }
//
//        boolean updateOccurred = false;
//        if (Objects.equals(selectedItem, ControlPanel.ALL_POINTS_MARKER)) {
//            for (CurvePoint point : currentPath) {
//                updateCurvePointParameter(point, textField, parsedValue);
//            }
//            instructionLabel.setText("Applied '" + getFieldName(textField) + " = " + currentText + "' to all points.");
//            updateOccurred = true;
//            refreshParameterFieldsForAllSelected(); // Refresh all fields for "ALL" view, as one change might make others uniform or varied
//        } else if (selectedItem instanceof CurvePoint) {
//            CurvePoint point = (CurvePoint) selectedItem;
//            updateCurvePointParameter(point, textField, parsedValue);
//            int pointIndex = currentPath.indexOf(point) + 1;
//            instructionLabel.setText("Updated Point " + pointIndex + " (" + getFieldName(textField) + " = " + currentText + ").");
//            updateOccurred = true;
//        }
//
//        if (updateOccurred) {
//            textFieldPreviousValues.put(textField, currentText); // Update stored value after successful application
//            fieldDisplay.drawCurrentState(); // Redraw path if parameters affect appearance (though unlikely for these)
//        }
//    }
//
//    private void updateCurvePointParameter(CurvePoint point, TextField changedField, double value) {
//        if (changedField == controlPanel.getMoveSpeedField()) point.moveSpeed = value;
//        else if (changedField == controlPanel.getTurnSpeedField()) point.turnSpeed = value;
//        else if (changedField == controlPanel.getFollowDistanceField()) point.followDistance = value;
//        else if (changedField == controlPanel.getPointLengthField()) point.pointLength = value;
//        else if (changedField == controlPanel.getSlowDownTurnDegreesField()) point.slowDownTurnRadians = Math.toRadians(value);
//        else if (changedField == controlPanel.getSlowDownTurnAmountField()) point.slowDownTurnAmount = value;
//    }
//
//    private String getFieldName(TextField textField) {
//        if (textField == controlPanel.getMoveSpeedField()) return "Move Speed";
//        if (textField == controlPanel.getTurnSpeedField()) return "Turn Speed";
//        if (textField == controlPanel.getFollowDistanceField()) return "Follow Distance";
//        if (textField == controlPanel.getPointLengthField()) return "Point Length";
//        if (textField == controlPanel.getSlowDownTurnDegreesField()) return "Slow Turn Deg";
//        if (textField == controlPanel.getSlowDownTurnAmountField()) return "Slow Turn Amt";
//        return "Parameter";
//    }
//
//    private void handlePointSelectionChanged(ObservableValue<? extends Object> obs, Object oldVal, Object newVal) {
//        if (controlPanel == null || isCreatingPath) { // Do not change display if actively drawing path
//            if (isCreatingPath && newVal != null && controlPanel.getSelectedPointFromComboBox() != null) {
//                // If drawing path and user somehow clicks ComboBox, try to revert to original selection if possible
//                // This is a bit defensive, ideally ComboBox is disabled during path creation
//                Platform.runLater(() -> controlPanel.updatePointSelectionComboBox(currentPath, oldVal != null ? oldVal : ControlPanel.ALL_POINTS_MARKER));
//            }
//            return;
//        }
//
//        CurvePoint pointToHighlight = null; // Initialize to null (no highlight)
//
//        if (newVal == null) {
//            if (currentPath.isEmpty()) {
//                controlPanel.loadGlobalDefaultsIntoParameterFields();
//                controlPanel.setPointEditingControlsDisabled(true);
//            } else {
//                // Should default to "ALL" if path exists
//                controlPanel.updatePointSelectionComboBox(currentPath, ControlPanel.ALL_POINTS_MARKER);
//                // This will re-trigger the listener with "ALL"
//            }
//            return;
//        } else if (Objects.equals(newVal, ControlPanel.ALL_POINTS_MARKER)) {
//            refreshParameterFieldsForAllSelected();
//            // When "ALL" is selected, no specific point is highlighted
//        } else if (newVal instanceof CurvePoint) {
//            CurvePoint selectedCurvePoint = (CurvePoint) newVal;
//            controlPanel.loadParametersForPoint(selectedCurvePoint);
//            pointToHighlight = selectedCurvePoint; // This is the point to highlight
//        }
//
//        System.out.println("ComboBox selection changed to: " + newVal);
//        if (Objects.equals(newVal, ControlPanel.ALL_POINTS_MARKER)) {
//            refreshParameterFieldsForAllSelected();
//        } else if (newVal instanceof CurvePoint) {
//            controlPanel.loadParametersForPoint((CurvePoint) newVal);
//        }
//
//        // Update the highlighted point in FieldDisplay and redraw
//        if (fieldDisplay != null) {
//            fieldDisplay.setHighlightedPoint(pointToHighlight);
//            fieldDisplay.drawCurrentState(); // Redraw the field to show highlight
//        }
//    }
//
//    private void refreshParameterFieldsForAllSelected() {
//        if (controlPanel == null) return;
//        if (currentPath.isEmpty()) {
//            controlPanel.loadGlobalDefaultsIntoParameterFields();
//            // Point editing controls should be disabled by updateControlPanelForPathState
//            return;
//        }
//
//        System.out.println("Refreshing for ALL points. Path size: " + currentPath.size());
//        checkAndSetField(currentPath, cp -> cp.moveSpeed, controlPanel.getMoveSpeedField(), "%.2f");
//        checkAndSetField(currentPath, cp -> cp.turnSpeed, controlPanel.getTurnSpeedField(), "%.2f");
//        checkAndSetField(currentPath, cp -> cp.followDistance, controlPanel.getFollowDistanceField(), "%.1f");
//        checkAndSetField(currentPath, cp -> cp.pointLength, controlPanel.getPointLengthField(), "%.1f");
//        checkAndSetField(currentPath, cp -> Math.toDegrees(cp.slowDownTurnRadians), controlPanel.getSlowDownTurnDegreesField(), "%.1f");
//        checkAndSetField(currentPath, cp -> cp.slowDownTurnAmount, controlPanel.getSlowDownTurnAmountField(), "%.2f");
//    }
//
//    private <T> void checkAndSetField(List<CurvePoint> path, Function<CurvePoint, T> getter, TextField field, String format) {
//        if (path.isEmpty()) { // Should be handled by calling context
//            loadSpecificGlobalDefault(field); // Fallback
//            return;
//        }
//
//        T firstValue = getter.apply(path.get(0));
//        boolean allSame = true;
//        for (int i = 1; i < path.size(); i++) {
//            T currentValue = getter.apply(path.get(i));
//            if (currentValue instanceof Double && firstValue instanceof Double) {
//                if (Math.abs((Double) currentValue - (Double) firstValue) > 0.0001) { // Tolerance for double comparison
//                    allSame = false;
//                    break;
//                }
//            } else if (!Objects.equals(currentValue, firstValue)) {
//                allSame = false;
//                break;
//            }
//        }
//
//        if (allSame) {
//            if (firstValue instanceof Double) {
//                field.setText(String.format(Locale.US, format, (Double) firstValue));
//            } else { // Should not happen with current parameters but good for generic
//                field.setText(firstValue.toString());
//            }
//        } else {
//            field.setText(ControlPanel.TEXTFIELD_VARIES_TEXT);
//        }
//        textFieldPreviousValues.put(field, field.getText()); // Update stored value for focus lost checks
//    }
//
//    private void loadSpecificGlobalDefault(TextField field) {
//        // Accessing static defaults from ControlPanel directly
//        if (field == controlPanel.getMoveSpeedField()) field.setText(ControlPanel.DEFAULT_MOVE_SPEED);
//        else if (field == controlPanel.getTurnSpeedField()) field.setText(ControlPanel.DEFAULT_TURN_SPEED);
//        else if (field == controlPanel.getFollowDistanceField()) field.setText(ControlPanel.DEFAULT_FOLLOW_DISTANCE);
//        else if (field == controlPanel.getPointLengthField()) field.setText(ControlPanel.DEFAULT_POINT_LENGTH);
//        else if (field == controlPanel.getSlowDownTurnDegreesField()) field.setText(ControlPanel.DEFAULT_SLOW_DOWN_TURN_DEGREES);
//        else if (field == controlPanel.getSlowDownTurnAmountField()) field.setText(ControlPanel.DEFAULT_SLOW_DOWN_TURN_AMOUNT);
//        else field.setText("");
//        textFieldPreviousValues.put(field, field.getText());
//    }
//
//    private void updateControlPanelForPathState() {
//        if (controlPanel == null) return;
//
//        boolean pathExistsAndNotEmpty = !currentPath.isEmpty();
//        controlPanel.setPointEditingControlsDisabled(!pathExistsAndNotEmpty);
//        controlPanel.enablePathControls(pathExistsAndNotEmpty); // For Delete, Export, Send buttons
//
//        Object selectionToRestore = controlPanel.getSelectedPointFromComboBox();
//        if (!pathExistsAndNotEmpty) {
//            selectionToRestore = ControlPanel.ALL_POINTS_MARKER; // Show ALL (which will load defaults) if no path
//        } else {
//            // If current selection is a CurvePoint but no longer in the path (e.g., path was cleared and a new short one made)
//            if (selectionToRestore instanceof CurvePoint && !currentPath.contains(selectionToRestore)) {
//                selectionToRestore = ControlPanel.ALL_POINTS_MARKER;
//            } else if (selectionToRestore == null) { // No selection yet, default to ALL if path exists
//                selectionToRestore = ControlPanel.ALL_POINTS_MARKER;
//            }
//        }
//        controlPanel.updatePointSelectionComboBox(currentPath, selectionToRestore);
//
//        // After ComboBox is updated, its value change listener (handlePointSelectionChanged)
//        // should take care of populating the TextFields correctly.
//        // So, explicitly calling refreshParameterFieldsForAllSelected or loadParametersForPoint here
//        // might be redundant if the ComboBox listener is robust.
//        // Let's ensure the listener is triggered or manually trigger if needed.
//        Object currentSelectionAfterUpdate = controlPanel.getSelectedPointFromComboBox();
//        if (currentSelectionAfterUpdate == null && pathExistsAndNotEmpty) {
//            // This case should ideally not happen if updatePointSelectionComboBox defaults to "ALL"
//            controlPanel.updatePointSelectionComboBox(currentPath, ControlPanel.ALL_POINTS_MARKER);
//        } else {
//            // Manually refresh based on current combo box state just in case listener didn't cover all edge cases on init
//            if (Objects.equals(currentSelectionAfterUpdate, ControlPanel.ALL_POINTS_MARKER)) {
//                refreshParameterFieldsForAllSelected();
//            } else if (currentSelectionAfterUpdate instanceof CurvePoint) {
//                controlPanel.loadParametersForPoint((CurvePoint) currentSelectionAfterUpdate);
//            } else { // No path or null selection
//                controlPanel.loadGlobalDefaultsIntoParameterFields();
//            }
//        }
//        // If path creation is active, disable point editing controls again
//        if (isCreatingPath) {
//            controlPanel.setPointEditingControlsDisabled(true);
//        }
//    }
//
//    private void handleSendPathToRobot() {
//        if (currentPath.isEmpty()) {
//            instructionLabel.setText("No path to send.");
//            System.out.println("Attempted to send path, but currentPath is empty.");
//            return;
//        }
//        instructionLabel.setText("Sending path to robot...");
//        System.out.println("Preparing to send " + currentPath.size() + " points to robot at " + ROBOT_IP_ADDRESS + ":" + ROBOT_LISTENER_PORT);
//        try (DatagramSocket socket = new DatagramSocket()) {
//            InetAddress address = InetAddress.getByName(ROBOT_IP_ADDRESS);
//            for (CurvePoint point : currentPath) {
//                String message = String.format(Locale.US, "curve_point:%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f",
//                        point.x, point.y, point.moveSpeed, point.turnSpeed,
//                        point.followDistance, point.pointLength,
//                        point.slowDownTurnRadians, point.slowDownTurnAmount);
//                byte[] buffer = message.getBytes(StandardCharsets.UTF_8);
//                DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, ROBOT_LISTENER_PORT);
//                socket.send(packet);
//                System.out.println("Sent: " + message);
//            }
//            String endMessage = "end";
//            byte[] endBuffer = endMessage.getBytes(StandardCharsets.UTF_8);
//            DatagramPacket endPacket = new DatagramPacket(endBuffer, endBuffer.length, address, ROBOT_LISTENER_PORT);
//            socket.send(endPacket);
//            System.out.println("Sent: " + endMessage);
//            instructionLabel.setText("Path sent successfully to " + ROBOT_IP_ADDRESS);
//            System.out.println("Path sending complete.");
//        } catch (IOException e) {
//            instructionLabel.setText("Error sending path: " + e.getMessage());
//            System.err.println("Error sending path to robot: " + e.getMessage());
//            e.printStackTrace();
//        }
//    }
//
//    private void exportPathToCSV(Stage ownerStage) {
//        if (currentPath.isEmpty()) {
//            instructionLabel.setText("No path to export.");
//            return;
//        }
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Save Path to CSV");
//        fileChooser.setInitialFileName("ftc_path.csv");
//        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("CSV Files (*.csv)", "*.csv"));
//        File file = fileChooser.showSaveDialog(ownerStage);
//        if (file != null) {
//            try (PrintWriter writer = new PrintWriter(file)) {
//                writer.println("x_inches,y_inches"); // Header
//                for (CurvePoint point : currentPath) {
//                    writer.printf(Locale.US, "%.3f,%.3f\n", point.x, point.y);
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
//    private void deleteCurrentPath() {
//        currentPath.clear();
//        isCreatingPath = false;
//        fieldDisplay.setPathCreationMode(false, null, () -> finishPathCreation(false));
//        fieldDisplay.setPathToDraw(currentPath);
//        fieldDisplay.drawCurrentState();
//        fieldDisplay.setHighlightedPoint(null);
//        instructionLabel.setText("Path deleted. Click 'New Path' to start drawing.");
//        updateControlPanelForPathState();
//    }
//
//    private void finishPathCreation(boolean cancelled) {
//        if (!isCreatingPath) return; // Guard against multiple calls
//
//        isCreatingPath = false; // Path creation is officially over
//        fieldDisplay.setPathCreationMode(false, null, null); // Stop field listening
//
//        if (cancelled && currentPath != null) { // Null check for currentPath
//            currentPath.clear();
//            instructionLabel.setText("Path creation cancelled. Click 'New Path' to start again.");
//        } else {
//            if (currentPath == null || currentPath.isEmpty()) { // Null check
//                instructionLabel.setText("Path finished with no points. Click 'New Path' to start again.");
//                if (currentPath != null) currentPath.clear(); // Ensure it's empty if it was just null
//                else currentPath = new ArrayList<>(); // Initialize if null
//            } else {
//                instructionLabel.setText("Path finished with " + currentPath.size() + " points. Select points to edit parameters.");
//            }
//        }
//
//        // Explicitly re-enable the "New Path" button and allow others to be controlled by path existence.
//        if (controlPanel != null) {
//            controlPanel.setPathEditingActive(false);
//        }
//
//        // This will now correctly update the state of "Delete", "Export", "Send"
//        // because newPathButton will be seen as enabled by enablePathControls.
//        // It will also update the ComboBox and parameter fields.
//        updateControlPanelForPathState();
//
//        // Update the visual path on the field
//        if (fieldDisplay != null && currentPath != null) { // Null checks
//            fieldDisplay.setPathToDraw(currentPath);
//            fieldDisplay.drawCurrentState();
//        }
//    }
//
//    private void handleFieldClickForPath(Point2D pixelCoords) {
//        if (!isCreatingPath || controlPanel == null) return;
//        Point2D inchesCoordsFieldCenter = fieldDisplay.pixelToInches(pixelCoords.getX(), pixelCoords.getY());
//        double moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnDeg, slowDownTurnAmount, slowDownTurnRad;
//        try {
//            moveSpeed = controlPanel.getMoveSpeedParam();
//            turnSpeed = controlPanel.getTurnSpeedParam();
//            followDistance = controlPanel.getFollowDistanceParam();
//            pointLength = controlPanel.getPointLengthParam();
//            slowDownTurnDeg = controlPanel.getSlowDownTurnDegreesParam();
//            slowDownTurnAmount = controlPanel.getSlowDownTurnAmountParam();
//            if (moveSpeed <= 0 || turnSpeed <= 0 || followDistance < 0 || pointLength <= 0 || slowDownTurnAmount < 0 || slowDownTurnAmount > 1) {
//                throw new NumberFormatException("Default parameter out of typical range.");
//            }
//            slowDownTurnRad = Math.toRadians(slowDownTurnDeg);
//        } catch (NumberFormatException e) {
//            instructionLabel.setText("Invalid global default parameter. Using master defaults for new point.");
//            System.err.println("Error parsing global default parameters: " + e.getMessage() + ". Using master defaults.");
//            moveSpeed = MASTER_DEFAULT_MOVE_SPEED;
//            turnSpeed = MASTER_DEFAULT_TURN_SPEED;
//            followDistance = MASTER_DEFAULT_FOLLOW_DISTANCE;
//            pointLength = MASTER_DEFAULT_POINT_LENGTH;
//            slowDownTurnRad = MASTER_DEFAULT_SLOW_DOWN_TURN_RADIANS;
//            slowDownTurnAmount = MASTER_DEFAULT_SLOW_DOWN_TURN_AMOUNT;
//        }
//        CurvePoint newPoint = new CurvePoint(
//                inchesCoordsFieldCenter.getX(), inchesCoordsFieldCenter.getY(),
//                moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRad, slowDownTurnAmount
//        );
//        currentPath.add(newPoint);
//        fieldDisplay.setPathToDraw(currentPath);
//        fieldDisplay.drawCurrentState();
//        if (currentPath.size() == 1) {
//            instructionLabel.setText("Point 1 added. Click next waypoint. ESC to cancel.");
//        } else {
//            instructionLabel.setText("Point " + currentPath.size() + " added. Click next, Double-click last, or ESC to cancel.");
//        }
//    }
//
//    private void updateTimeLapsedDisplay() {
//        if (controlPanel == null || recordingManager == null) return;
//
//        long timeLapsedMs;
//        if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) {
//            // If recording, show time since recording started
//            timeLapsedMs = recordingManager.getCurrentRecordingDuration(); // Assumes you add such a method to RecordingManager
//            // or calculate it based on System.time - recordingStartTime
//        } else {
//            timeLapsedMs = recordingManager.getCurrentEventTimeLapsed();
//        }
//        controlPanel.updateTimeLapsed(timeLapsedMs);
//    }
//
//    private void startNewPathCreation() {
//        if (isCreatingPath) return; // Already in this mode
//
//        if (currentPath == null) { // Defensive initialization
//            currentPath = new ArrayList<>();
//        }
//        currentPath.clear();
//        isCreatingPath = true;
//        fieldDisplay.setHighlightedPoint(null);
//
//        // Mode change: Disable "New Path", "Delete", "Export", "Send"
//        // and enable "Finish Path" / "Cancel Path" implicitly by UI context.
//        if (controlPanel != null) {
//            controlPanel.setPathEditingActive(true);
//        }
//
//        // Update other UI elements:
//        // - ComboBox for points should be cleared/disabled.
//        // - Parameter fields should show global defaults but be disabled.
//        // - "Delete", "Export", "Send" will be further confirmed as disabled by enablePathControls(false)
//        updateControlPanelForPathState();
//
//        instructionLabel.setText("Click the first waypoint. Parameters from global defaults will be used.");
//        if (fieldDisplay != null) {
//            fieldDisplay.setPathCreationMode(true, this::handleFieldClickForPath, () -> finishPathCreation(false));
//            fieldDisplay.setPathToDraw(currentPath); // Show path as it's built
//            fieldDisplay.drawCurrentState();
//        }
//    }
//
//    // --- Recording, UDP, and Other Utility Methods (ensure these are complete from your original) ---
//    private void setupRecordingControlActions() {
//        controlPanel.setOnOpenAction(e -> handleOpenRecording());
//        controlPanel.setOnSaveAction(e -> handleSaveRecording());
//        controlPanel.setOnRecordAction(() -> {
//            RecordingManager.PlaybackState recordingState = recordingManager.getCurrentState();
//            if (recordingState == RecordingManager.PlaybackState.RECORDING) {
//                recordingManager.stopRecording();
//                controlPanel.toggleRecordButtonIcon(false);
//                if (recordingManager.hasRecording()) {
//                    controlPanel.setPlaybackControlsDisabled(false);
//                    controlPanel.updateTimelineSlider(recordingManager.getPlaybackIndex(), recordingManager.getTotalEvents());
//                    controlPanel.setSaveButtonDisabled(false);
//                } else {
//                    controlPanel.setPlaybackControlsDisabled(true);
//                    controlPanel.setSaveButtonDisabled(true);
//                }
//            } else {
//                controlPanel.togglePlayPauseButtonIcon(false);
//                recordingManager.startRecording();
//                controlPanel.toggleRecordButtonIcon(true);
//                controlPanel.setPlaybackControlsDisabled(true);
//                controlPanel.setSaveButtonDisabled(true);
//            }
//            updateTimeLapsedDisplay();
//        });
//        controlPanel.setOnPlayPauseAction(() -> {
//            RecordingManager.PlaybackState playbackState = recordingManager.getCurrentState();
//            if (playbackState == RecordingManager.PlaybackState.PLAYING) {
//                recordingManager.pause();
//                controlPanel.togglePlayPauseButtonIcon(false);
//            } else if (playbackState == RecordingManager.PlaybackState.IDLE || playbackState == RecordingManager.PlaybackState.PAUSED) {
//                if (recordingManager.hasRecording()) {
//                    recordingManager.play();
//                    controlPanel.togglePlayPauseButtonIcon(true);
//                }
//            }
//            updateTimeLapsedDisplay();
//        });
//        controlPanel.setOnForwardAction(() -> {
//            if (recordingManager.hasRecording() && recordingManager.getCurrentState() != RecordingManager.PlaybackState.RECORDING) {
//                recordingManager.stepForward();
//                controlPanel.togglePlayPauseButtonIcon(false);
//            }
//            updateTimeLapsedDisplay();
//        });
//        controlPanel.setOnReverseAction(() -> {
//            if (recordingManager.hasRecording() && recordingManager.getCurrentState() != RecordingManager.PlaybackState.RECORDING) {
//                recordingManager.stepBackward();
//                controlPanel.togglePlayPauseButtonIcon(false);
//            }
//            updateTimeLapsedDisplay();
//        });
//        controlPanel.setOnTimelineSliderChanged((observable, oldValue, newValue) -> {
//            if (controlPanel.getTimelineSlider().isValueChanging() && recordingManager.getCurrentState() != RecordingManager.PlaybackState.PLAYING) {
//                recordingManager.seekTo(newValue.intValue());
//                controlPanel.togglePlayPauseButtonIcon(false);
//                updateTimeLapsedDisplay();
//            }
//        });
//        controlPanel.setOnSliderMouseReleased(event -> {
//            if (recordingManager.getCurrentState() != RecordingManager.PlaybackState.PLAYING &&
//                    recordingManager.getCurrentState() != RecordingManager.PlaybackState.RECORDING) {
//
//                int sliderRawValue = (int) controlPanel.getTimelineSlider().getValue();
//
//                // The FieldDisplay might need to be cleared before a potentially large jump
//                // if (fieldDisplay != null) {
//                // fieldDisplay.clearDrawingSurface(); // Or similar method
//                // }
//
//                recordingManager.seekTo(sliderRawValue); // seekTo will now handle snapping and dispatching
//
//                // After seekTo, playbackIndex in RecordingManager is at the snapped position.
//                // The dispatchCurrentEvent calls within seekTo would have updated the
//                // onProgressUpdateCallback, which in turn should update the slider's visual position
//                // to the new playbackIndex and the time label.
//                // So, an explicit controlPanel.getTimelineSlider().setValue() here might cause a flicker
//                // if onProgressUpdate also does it. Let onProgressUpdate be the source of truth for UI sync.
//
//                controlPanel.togglePlayPauseButtonIcon(false); // Ensure UI consistency
//            }
//            // updateTimeLapsedDisplay(); // Should be handled by onProgressUpdate from seekTo
//        });
//
//
//    }
//
//    private void handleSaveRecording() {
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Save Recording");
//        fileChooser.setInitialFileName("recording.rec");
//        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
//        File file = fileChooser.showSaveDialog(primaryStage);
//        if (file == null) { instructionLabel.setText("Save cancelled."); return; }
//        ArrayList<RecordingManager.RecordedEvent> events = recordingManager.getRecordedSession();
//        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
//            for (RecordingManager.RecordedEvent event : events) {
//                String line = formatEventToString(event);
//                if (line != null) {
//                    writer.write(line);
//                    writer.newLine();
//                }
//            }
//            instructionLabel.setText("Recording saved: " + file.getName());
//        } catch (IOException e) {
//            instructionLabel.setText("Error saving recording: " + e.getMessage());
//            e.printStackTrace();
//        }
//    }
//
//    private String formatEventToString(RecordingManager.RecordedEvent event) {
//        String payload; UdpMessageData data = event.messageData;
//        if (data instanceof PositionData) { PositionData d = (PositionData) data; payload = String.format(Locale.US,"pos:%.3f,%.3f,%.3f", d.x, d.y, d.heading); }
//        else if (data instanceof CircleData) { CircleData d = (CircleData) data; payload = String.format(Locale.US,"cir:%.3f,%.3f", d.radiusInches, d.heading); }
//        else if (data instanceof LineData) { LineData d = (LineData) data; payload = String.format(Locale.US,"line:%s,%.3f,%.3f,%.3f,%.3f,%d", d.name, d.x1, d.y1, d.x2, d.y2, d.styleCode); }
//        else if (data instanceof TextData) { TextData d = (TextData) data; payload = "txt:" + d.text; }
//        else if (data instanceof KeyValueData) {
//            KeyValueData kv = (KeyValueData) data;
//            // Simple escaping for now to handle potential issues in the value string
//            payload = String.format("kv:%s,%s", kv.key, kv.value);
//        }
//        else { return null; }
//        return event.timestamp + "|" + payload;
//    }
//
//    private void handleOpenRecording() {
//        FileChooser fileChooser = new FileChooser();
//        fileChooser.setTitle("Open Recording");
//        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Recording Files", "*.rec"));
//        File file = fileChooser.showOpenDialog(primaryStage);
//        if (file == null) { instructionLabel.setText("Open cancelled."); return; }
//        ArrayList<RecordingManager.RecordedEvent> loadedEvents = new ArrayList<>();
//        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
//            String line;
//            while ((line = reader.readLine()) != null) {
//                String[] lineParts = line.split("\\|", 2);
//                if (lineParts.length != 2) continue;
//                long timestamp = Long.parseLong(lineParts[0]); String payload = lineParts[1];
//                UdpMessageData parsedData = null;
//                if (payload.startsWith("pos:")) { String c = payload.substring(4); String[] p = c.split(","); if (p.length == 3) parsedData = new PositionData(Double.parseDouble(p[0]), Double.parseDouble(p[1]), Double.parseDouble(p[2])); }
//                else if (payload.startsWith("cir:")) { String c = payload.substring(4); String[] p = c.split(","); if (p.length == 2) parsedData = new CircleData(Double.parseDouble(p[0]), Double.parseDouble(p[1])); }
//                else if (payload.startsWith("line:")) { String c = payload.substring(5); String[] p = c.split(",", 6); if (p.length == 6) parsedData = new LineData(p[0], Double.parseDouble(p[1]), Double.parseDouble(p[2]), Double.parseDouble(p[3]), Double.parseDouble(p[4]), Integer.parseInt(p[5]));}
//                else if (payload.startsWith("txt:")) { parsedData = new TextData(payload.substring(4));}
//                else if (payload.startsWith("kv:")) {
//                    String c = payload.substring(3);
//                    String[] p = c.split(",", 2);
//                    if (p.length == 2) parsedData = new KeyValueData(p[0], p[1]);
//                }
//                if (parsedData != null) loadedEvents.add(new RecordingManager.RecordedEvent(timestamp, parsedData));
//            }
//            recordingManager.loadRecording(loadedEvents);
//            controlPanel.setPlaybackControlsDisabled(loadedEvents.isEmpty());
//            controlPanel.updateTimelineSlider(0, recordingManager.getTotalEvents());
//            controlPanel.setSaveButtonDisabled(loadedEvents.isEmpty());
//            controlPanel.togglePlayPauseButtonIcon(false);
//            instructionLabel.setText("Opened: " + file.getName());
//            updateTimeLapsedDisplay();
//        } catch (Exception e) {
//            instructionLabel.setText("Error opening recording: " + e.getMessage());
//            e.printStackTrace();
//        }
//    }
//
//    private void handleUdpMessage(UdpMessageData messageData) {
//        if (messageData == null) return;
//        Platform.runLater(() -> {
//            if (recordingManager.getCurrentState() == RecordingManager.PlaybackState.RECORDING) recordingManager.addEvent(messageData);
//            if (messageData instanceof PositionData) { PositionData p = (PositionData)messageData; if(robot!=null){ fieldDisplay.addTrailDot(robot.getXInches(),robot.getYInches()); robot.setPosition(p.x,p.y); robot.setHeading(p.heading);}}
//            else if (messageData instanceof CircleData) { CircleData c = (CircleData)messageData; if(fieldDisplay!=null) fieldDisplay.addDebugCircle(robot.getXInches(),robot.getYInches(),c.radiusInches,c.heading,Color.rgb(255,165,0,0.7));}
//            else if (messageData instanceof LineData) { LineData l = (LineData)messageData; synchronized(namedLinesLock){namedLinesToDraw.put(l.name,l);}}
//            else if (messageData instanceof TextData) { TextData t = (TextData)messageData; if(fieldDisplay!=null) fieldDisplay.setRobotTextMessage(t.text);}
//            else if (messageData instanceof KeyValueData) {
//                KeyValueData kv = (KeyValueData) messageData;
//                if (keyValueTable != null) {
//                    keyValueTable.updateValue(kv.key, kv.value);
//                }
//            }
//            if(messageData instanceof PositionData) updateUIFromRobotState(); else fieldDisplay.drawCurrentState();
//        });
//    }
//
//    private void onPlaybackFinished() {
//        Platform.runLater(() -> {
//            if (controlPanel != null) {
//                controlPanel.togglePlayPauseButtonIcon(false);
//            }
//            updateTimeLapsedDisplay();
//        });
//    }
//
//    private void startUdpPositionListener() {
//        try {
//            udpListener = new UdpPositionListener(UDP_LISTENER_PORT, this::handleUdpMessage);
//            udpListenerThread = new Thread(udpListener, "UdpListenerThread");
//            udpListenerThread.setDaemon(true);
//            udpListenerThread.start();
//            System.out.println("UDP Listener started on port " + UDP_LISTENER_PORT);
//        } catch (Exception e) {
//            instructionLabel.setText("ERROR: UDP Listener start failed on " + UDP_LISTENER_PORT);
//            e.printStackTrace();
//        }
//    }
//
//    private void startUdpPlotListener() {
//        try {
//            udpPlotListener = new UdpPlotListener(this::handleUdpPlotData); // Uses default port from UdpPlotListener
//            udpPlotListenerThread = new Thread(udpPlotListener, "UdpPlotListenerThread");
//            udpPlotListenerThread.setDaemon(true);
//            udpPlotListenerThread.start();
//            System.out.println("UDP Plot Listener started on port " + UdpPlotListener.DEFAULT_PLOT_LISTENER_PORT);
//        } catch (Exception e) {
//            // You might want a different label or way to show this error if instructionLabel is for field sim
//            System.err.println("ERROR: UDP Plot Listener start failed on " + UdpPlotListener.DEFAULT_PLOT_LISTENER_PORT + " - " + e.getMessage());
//            if (instructionLabel != null) { // Be cautious if this runs before instructionLabel is ready
//                instructionLabel.setText("ERROR: Plot Listener failed");
//            }
//            e.printStackTrace();
//        }
//    }
//
//    private void updateUIFromRobotState() {
//        if (robot != null && controlPanel != null && fieldDisplay != null) {
//            double displayHeading = robot.getHeadingDegrees() % 360;
//            if (displayHeading < 0) displayHeading += 360; // Normalize to 0-359
//            controlPanel.updateRobotStatus(robot.getXInches(), robot.getYInches(), displayHeading);
//            fieldDisplay.drawCurrentState();
//        }
//    }
//
//    private void stopApp() {
//        if (udpListener != null) udpListener.stopListener();
//        if (udpPlotListener != null) udpPlotListener.stopListener();
//
//        if (udpListenerThread != null && udpListenerThread.isAlive()) {
//            try {
//                udpListenerThread.join(500);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
//        }
//        if (udpPlotListenerThread != null && udpPlotListenerThread.isAlive()) {
//            try {
//                udpPlotListenerThread.join(500);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
//        }
//        if (plotDisplayWindow != null && plotDisplayWindow.isShowing()){ // Close plot window if open
//            plotDisplayWindow.hide();
//        }
//        System.out.println("Exiting application.");
//        Platform.exit();
//        System.exit(0);
//    }
//
//    private void clearAllNamedLines() {
//        synchronized (namedLinesLock) {
//            namedLinesToDraw.clear();
//        }
//        if (fieldDisplay != null) {
//            fieldDisplay.drawCurrentState();
//        }
//    }
//
//    private void handleSceneKeyPress(KeyEvent event) {
//        if (isCreatingPath) {
//            if (event.getCode() == KeyCode.ESCAPE) {
//                finishPathCreation(true); // true for cancelled
//                event.consume();
//            }
//        } else {
//            handleRobotMovementKeyPress(event);
//        }
//    }
//
//    private void handleRobotMovementKeyPress(KeyEvent event) {
//        if (robot == null || isCreatingPath) { // Do not move robot if path creation is active
//            return;
//        }
//        double currentX = robot.getXInches();
//        double currentY = robot.getYInches();
//        double currentHeading_CCW = robot.getHeadingDegrees();
//        boolean moved = false;
//
//        double newFieldX = currentX;
//        double newFieldY = currentY;
//        double angleRad_CCW = Math.toRadians(currentHeading_CCW);
//
//        switch (event.getCode()) {
//            case UP:
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
//                moved = true;
//                break;
//            case DOWN:
//                newFieldX = currentX - ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
//                newFieldY = currentY - ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
//                moved = true;
//                break;
//            case LEFT:
//                robot.setHeading(currentHeading_CCW + ROBOT_TURN_INCREMENT_DEGREES);
//                moved = true;
//                break;
//            case RIGHT:
//                robot.setHeading(currentHeading_CCW - ROBOT_TURN_INCREMENT_DEGREES);
//                moved = true;
//                break;
//            case A: // Strafe Left
//                double strafeLeftAngleRad_CCW = Math.toRadians(currentHeading_CCW + 90.0);
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeLeftAngleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeLeftAngleRad_CCW);
//                moved = true;
//                break;
//            case D: // Strafe Right
//                double strafeRightAngleRad_CCW = Math.toRadians(currentHeading_CCW - 90.0);
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeRightAngleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeRightAngleRad_CCW);
//                moved = true;
//                break;
//            default:
//                // Not a movement key we handle here
//                break;
//        }
//
//        if (moved) {
//            if (event.getCode() == KeyCode.UP || event.getCode() == KeyCode.DOWN ||
//                    event.getCode() == KeyCode.A || event.getCode() == KeyCode.D) {
//                robot.setPosition(newFieldX, newFieldY);
//            }
//            updateUIFromRobotState();
//            event.consume(); // Consume the event so it's not processed further (e.g., by focus traversal)
//        }
//    }
//}
