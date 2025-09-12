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
import javafx.scene.input.MouseButton; // For double click
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox; // For instruction panel
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.scene.paint.Color;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;

public class FtcFieldSimulatorApp extends Application {

    // --- Configuration Constants ---
    public static final double FIELD_WIDTH_INCHES = 144.0; // 12 feet
    public static final double FIELD_HEIGHT_INCHES = 144.0; // 12 feet

    // Desired window size (can be adjusted, scaling will handle it)
    private static final int FIELD_DISPLAY_WIDTH_PIXELS = 720; // This is now for the FieldDisplay itself
    private static final int FIELD_DISPLAY_HEIGHT_PIXELS = 720;

    // Path to your field image (REPLACE WITH YOUR ACTUAL PATH or put in resources)
    // For simplicity now, let's assume it's next to the compiled classes or in a known relative path.
    // A better way is to load from resources folder if using a build system like Maven/Gradle.
//    private static final String FIELD_IMAGE_PATH = "/field.png"; // e.g., src/main/resources/field.png
    private static final String FIELD_IMAGE_PATH = "/decode_field.png"; // e.g., src/main/resources/field.png
    private static final String ROBOT_IMAGE_PATH = "/robot.png"; // Add if Robot class doesn't define it itself
    public static final double FIELD_IMAGE_ALPHA = 0.3;
    public static final double BACKGROUND_ALPHA = 0.1;
    private FieldDisplay fieldDisplay;
    private ControlPanel controlPanel;
    private Robot robot; // Add robot instance

    // --- Robot Starting Configuration ---
    public static final double ROBOT_START_FIELD_X = 0.0;  // Initial position on Field X-axis (vertical, 0=center)
    public static final double ROBOT_START_FIELD_Y = 0.0;  // Initial position on Field Y-axis (horizontal, 0=center)
    public static final double ROBOT_START_HEADING_DEGREES = 90.0; // Initial heading (0=+FieldX/Down, 90=+FieldY/Right, CCW)

    private static final double ROBOT_MOVE_INCREMENT_INCHES = 2.0; // Move 2 inches per key press
    private static final double ROBOT_TURN_INCREMENT_DEGREES = 5.0; // Turn 5 degrees per key press

    private List<CurvePoint> initialPath; // We'll populate this later

    private UdpPositionListener udpListener;
    private Thread udpListenerThread;
    private static final int UDP_LISTENER_PORT = 7777; // Example port

    // --- Path Management ---
    private List<CurvePoint> currentPath = new ArrayList<>();
    private boolean isCreatingPath = false;
    private Label instructionLabel; // For user instructions

    // +++ Storage for Named Lines +++
    // Using ConcurrentHashMap if FieldDisplay might iterate it from a non-FX thread,
    // otherwise HashMap is fine if all access (add/iterate for drawing) is on FX thread.
    // Since FieldDisplay.drawCurrentState() is called from Platform.runLater, HashMap is okay.
    private Map<String, UdpPositionListener.LineData> namedLinesToDraw = new HashMap<>();
    private final Object namedLinesLock = new Object(); // Lock for synchronizing access to namedLinesToDraw

    public static void main(String[] args) {
        Application.launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        primaryStage.setTitle("FTC Field Simulator");

        // --- Calculate dimensions ---
        double controlPanelWidth = FIELD_DISPLAY_WIDTH_PIXELS * ControlPanel.PREFERRED_WIDTH_RATIO_TO_FIELD;
        double totalAppWidth = FIELD_DISPLAY_WIDTH_PIXELS + controlPanelWidth;
        double totalAppHeight = FIELD_DISPLAY_HEIGHT_PIXELS; // Adjusted for instruction label later

        // --- Define the 80% bounding box limits ---
        double margin = FIELD_WIDTH_INCHES * 0.10; // 10% margin on each side for 80% coverage
        double pathMinX = margin;
        double pathMaxX = FIELD_WIDTH_INCHES - margin;
        double pathMinY = margin;
        double pathMaxY = FIELD_HEIGHT_INCHES - margin;

        double pathWidth = pathMaxX - pathMinX;
        double pathHeight = pathMaxY - pathMinY;

//        // --- Populate initialPath with S-shape points ---
//        initialPath = createDefaultSPath();
//        robot = createInitialRobot(initialPath);
//        robot = new Robot(
//                0.0, // Initial X at field center
//                0.0, // Initial Y at field center
//                0.0, // Initial Heading
//                ROBOT_IMAGE_PATH);

        this.robot = new Robot(
                ROBOT_START_FIELD_X,
                ROBOT_START_FIELD_Y,
                ROBOT_START_HEADING_DEGREES,
                ROBOT_IMAGE_PATH
        );

        // Create the field display
        fieldDisplay = new FieldDisplay(
                FIELD_DISPLAY_WIDTH_PIXELS,
                FIELD_DISPLAY_HEIGHT_PIXELS,
                FIELD_WIDTH_INCHES,
                FIELD_HEIGHT_INCHES,
                FIELD_IMAGE_PATH,
                robot,
                BACKGROUND_ALPHA,
                FIELD_IMAGE_ALPHA
        );
//        fieldDisplay.setPath(initialPath); // Set path (even if empty)

        // Pass the map of named lines to FieldDisplay
        fieldDisplay.setNamedLinesMap(namedLinesToDraw);

        // Example: Initial text or clear it
        fieldDisplay.setRobotTextMessage(null); // Or some default like "Ready"

        controlPanel = new ControlPanel(controlPanelWidth);

        // --- Instruction Label ---
        instructionLabel = new Label("Click 'New Path' to start drawing.");
        instructionLabel.setPadding(new Insets(5));
        instructionLabel.setMaxWidth(Double.MAX_VALUE);
        instructionLabel.setAlignment(Pos.CENTER);
        HBox instructionPane = new HBox(instructionLabel); // Put label in HBox for potential styling/centering
        instructionPane.setAlignment(Pos.CENTER);
        instructionPane.setStyle("-fx-background-color: #CFD8DC;"); // Light blue-gray background

        // --- Setup Main Layout (BorderPane) ---
        BorderPane mainLayout = new BorderPane();
        mainLayout.setLeft(controlPanel);
        mainLayout.setCenter(fieldDisplay);
        mainLayout.setBottom(instructionPane); // Add instruction panel at the bottom

//         Make the mainLayout (or fieldDisplay if you make it focusable) able to receive focus
//        mainLayout.setFocusTraversable(true); // Allow it to be part of focus traversal
//         fieldDisplay.setFocusTraversable(true);

        // --- Setup Scene ---
        // Adjust totalAppHeight if instructionPane has significant height
        double instructionPaneHeight = 30; // Estimate or calculate dynamically
        Scene scene = new Scene(mainLayout, totalAppWidth, totalAppHeight + instructionPaneHeight);
//        scene.setOnKeyPressed(this::handleRobotMovementKeyPress);
//        primaryStage.setScene(scene);
//        primaryStage.setResizable(true); // Allow resizing, though layout might need more work for robustness
//        primaryStage.show();

        // --- Event Handlers for ControlPanel Buttons ---
        controlPanel.setOnNewPathAction(event -> startNewPathCreation());
        controlPanel.setOnDeletePathAction(event -> deleteCurrentPath());
        controlPanel.setOnExportPathAction(event -> exportPathToCSV(primaryStage));
        controlPanel.setOnClearTrailAction(event -> {
            fieldDisplay.clearTrail(); // Call a method on FieldDisplay to clear its trail
            fieldDisplay.drawCurrentState(); // Redraw the field to show the cleared trail
            instructionLabel.setText("Robot trail cleared."); // Update instruction label
            System.out.println("Clear Trail button action: Trail cleared.");
        });
        controlPanel.setOnClearNamedLinesAction(event -> { // Assuming you add this button to ControlPanel
            clearAllNamedLines();
            instructionLabel.setText("All custom lines cleared.");
            System.out.println("Clear All Lines button action: Named lines cleared.");
        });
        controlPanel.enablePathControls(!currentPath.isEmpty()); // Initial state of buttons

        // --- Event Handlers for FieldDisplay and Scene (Path Creation) ---
        scene.setOnKeyPressed(this::handleSceneKeyPress); // For ESC to cancel path

        // --- Start UDP Listener ---
        udpListener = new UdpPositionListener(UDP_LISTENER_PORT, this::handleUdpMessage);
        udpListenerThread = new Thread(udpListener, "UdpListenerThread");
        udpListenerThread.setDaemon(true); // Allow JVM to exit if only this thread is running
        udpListenerThread.start();

        primaryStage.setScene(scene);
        primaryStage.setResizable(true);
        primaryStage.show();
        primaryStage.setOnCloseRequest(event -> stopApp());

        primaryStage.setOnCloseRequest(event -> { // Ensure listener stops on window close
            System.out.println("Window close requested. Stopping UDP listener.");
            if (udpListener != null) {
                udpListener.stopListener();
            }
            if (udpListenerThread != null && udpListenerThread.isAlive()) {
                try {
                    udpListenerThread.join(1000); // Wait for thread to die
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    System.err.println("Interrupted while waiting for UDP listener thread to stop.");
                }
            }
            System.out.println("Exiting application.");
        });

        // --- Initial UI Updates ---
        updateUIFromRobotState(); // Call method to update status and draw
    }

    private void clearAllNamedLines() {
        synchronized (namedLinesLock) {
            namedLinesToDraw.clear();
        }
        if (fieldDisplay != null) {
            fieldDisplay.drawCurrentState(); // Redraw to reflect the cleared lines
        }
    }

    private void handleSceneKeyPress(KeyEvent event) {
        if (isCreatingPath) {
            // --- Keys active ONLY during path creation ---
            if (event.getCode() == KeyCode.ESCAPE) {
                System.out.println("Path Creation: ESC pressed.");
                finishPathCreation(true); // true = cancel by ESC
                event.consume(); // Consume the event if handled
            }
            // Add other path-creation specific key actions here if needed
        } else {
            System.out.println("Key Press - Attempting Robot Movement for: " + event.getCode());
            // --- Keys active when NOT creating a path ---
            if (event.getCode() == KeyCode.DELETE && !currentPath.isEmpty()) {
                System.out.println("Path Management: DELETE pressed.");
                deleteCurrentPath();
                event.consume(); // Consume the event if handled
            } else {
                // If no specific non-path-creation key was handled above,
                // try to handle it as robot movement.
                // handleRobotMovementKeyPress itself checks if robot is null.
                System.out.println("Key Press - Attempting Robot Movement for: " + event.getCode());
                handleRobotMovementKeyPress(event);
                // Don't consume here unless handleRobotMovementKeyPress explicitly handles and consumes it.
                // If arrow keys are meant for other things too (like navigating controls),
                // consumption needs careful thought. For now, assume it's for robot movement.
            }
        }
//        if (isCreatingPath && event.getCode() == KeyCode.ESCAPE) {
//            finishPathCreation(true); // true = cancel by ESC
//        } else if (event.getCode() == KeyCode.DELETE && !currentPath.isEmpty() && !isCreatingPath) {
//            deleteCurrentPath(); // Allow deleting path with DELETE key if not editing
//        } else {
//            handleRobotMovementKeyPress(event); // Existing robot movement
//        }
    }

    private void startNewPathCreation() {
        if (isCreatingPath) return; // Already creating

        currentPath.clear();
        isCreatingPath = true;
        instructionLabel.setText("Click the first waypoint on the field. ESC to cancel, Double-click last point to finish.");
//        fieldDisplay.setPathCreationMode(true, this::handleFieldClickForPath);
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
//        currentPath.add(new CurvePoint(inchesCoords.getX(), inchesCoords.getY()));
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

    // Call this when path creation is done (e.g., ESC, Double Click, or a "Finish" button)
    private void finishPathCreation(boolean cancelled) {
        if (!isCreatingPath) return;

        System.out.println("Finishing path creation. Cancelled: " + cancelled + ". Current isCreatingPath: " + isCreatingPath);
        isCreatingPath = false;
        System.out.println("Set isCreatingPath to: " + isCreatingPath);
//        fieldDisplay.setPathCreationMode(false, null);
        fieldDisplay.setPathCreationMode(
                false,
                null,  // For single clicks to add points
                () -> finishPathCreation(false) // For double-clicks to finish (not cancelled)
        );
        controlPanel.setPathEditingActive(false);

        if (cancelled) {
            currentPath.clear();
            instructionLabel.setText("Path creation cancelled. Click 'New Path' to start again.");
        } else if (currentPath.isEmpty()) {
            instructionLabel.setText("Path is empty. Click 'New Path' to start again.");
        } else {
            instructionLabel.setText("Path finished with " + currentPath.size() + " points. You can now Export or Delete it.");
        }
        fieldDisplay.setPathToDraw(currentPath);
        fieldDisplay.drawCurrentState();
        controlPanel.enablePathControls(!currentPath.isEmpty());
    }

    private void deleteCurrentPath() {
        currentPath.clear();
        isCreatingPath = false; // Ensure path creation mode is exited
//        fieldDisplay.setPathCreationMode(false, null);
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

    // --- Callback method for UDP Pose/Circle/Line Updates ---
    private void handleUdpMessage(UdpMessageData messageData) {
        if (messageData == null) return;

        Platform.runLater(() -> { // Ensure all UI updates are on the JavaFX Application Thread
            if (messageData instanceof PositionData) {
                PositionData pose = (PositionData) messageData;
                // Existing logic to handle robot position update
                if (robot != null && fieldDisplay != null) {
                    fieldDisplay.addTrailDot(robot.getXInches(), robot.getYInches());
                    robot.setPosition(pose.x, pose.y);
                    robot.setHeading(pose.heading);
//                    updateUIFromRobotState();
                }
                System.out.println("Handled Position Update: " + pose);

            } else if (messageData instanceof CircleData) {
                CircleData circle = (CircleData) messageData;
                if (robot != null && fieldDisplay != null) {
                    // Draw circle centered at the current robot position
                    fieldDisplay.addDebugCircle(
                            robot.getXInches(),    // Center X at current robot X
                            robot.getYInches(),    // Center Y at current robot Y
                            circle.radiusInches,
                            circle.heading,        // <<< PASS THE HEADING
                            Color.rgb(255, 165, 0, 0.7) // Orange with some transparency
                    );
//                    fieldDisplay.drawCurrentState(); // Redraw to show the new debug shape
                    System.out.println("Drawing Debug Circle: Center (" + robot.getXInches() + ", " + robot.getYInches() +
                            "), Radius=" + circle.radiusInches + ", Heading=" + circle.heading);                }
            } else if (messageData instanceof LineData) {
                LineData line = (LineData) messageData; // Now the consolidated LineData
                synchronized (namedLinesLock) {
                    namedLinesToDraw.put(line.name, line); // Add or replace line by its name
                }
                // fieldDisplay.drawCurrentState(); // Called at the end
                System.out.println("Handled Line Update/Add: Name='" + line.name + "', Style=" + line.style +
                        ", Coords=(" + line.x0 + "," + line.y0 + ")-(" + line.x1 + "," + line.y1 + ")");
            } else if (messageData instanceof TextData) {
                TextData text = (TextData) messageData;
                if (fieldDisplay != null) {
                    fieldDisplay.setRobotTextMessage(text.text);
                    System.out.println("Displaying robot text: " + text.text); // For console logging
                }
            } else {
                System.err.println("Received unknown UdpMessageData type: " + messageData.getClass().getName());
            }

            // Always redraw after processing any message that might change the display
            if (fieldDisplay != null) {
                // If robot position was updated, status panel also needs update
                if (messageData instanceof PositionData) {
                    updateUIFromRobotState(); // Updates status and redraws field
                } else {
                    fieldDisplay.drawCurrentState(); // Just redraw field for other shapes
                }
            }
        });
    }

//    // --- Callback method for UDP Pose Updates ---
//    private void handlePoseUpdateFromUdp(UdpPositionListener.RobotPose newPose) { // newPose.x, newPose.y are field-center
//        if (robot != null && fieldDisplay != null) {
//            // Add a trail dot at the robot's *current* position BEFORE moving it
//            fieldDisplay.addTrailDot(robot.getXInches(), robot.getYInches());
//
//            // Update the robot's position and heading
//            robot.setPosition(newPose.x, newPose.y);
//            robot.setHeading(newPose.heading);
//
//            // Update the UI (status panel and redraw field display)
//            updateUIFromRobotState();
//        }
//    }
//


    private void handleRobotMovementKeyPress(KeyEvent event) {
        // System.out.println("handleRobotMovementKeyPress checking for: " + event.getCode()); // DEBUG
        if (robot == null || isCreatingPath) {
            if (isCreatingPath) System.out.println("Robot movement skipped: isCreatingPath is true.");
            if (robot == null) System.out.println("Robot movement skipped: robot is null.");
            return;
        }

        double currentX = robot.getXInches(); // Current Field X (vertical)
        double currentY = robot.getYInches(); // Current Field Y (horizontal)
        double currentHeading_CCW = robot.getHeadingDegrees(); // 0=+FieldX (Down), 90=+FieldY (Right), CCW positive
        boolean moved = false;

        // Declare newFieldX and newFieldY here, to be assigned in the switch cases
        double newFieldX = currentX; // Initialize with current position
        double newFieldY = currentY; // Initialize with current position

        double angleRad_CCW = Math.toRadians(currentHeading_CCW);

        switch (event.getCode()) {
            case UP: // Move "forward" along the robot's current heading
                // FieldX component = cos(angle), FieldY component = sin(angle)
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
                // robot.setPosition(newFieldX, newFieldY); // Set position after switch
                moved = true;
                break;
            case DOWN: // Move "backward"
                newFieldX = currentX - ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
                newFieldY = currentY - ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
                // robot.setPosition(newFieldX, newFieldY);
                moved = true;
                break;
            case LEFT: // Turn left (increase heading in CCW system)
                robot.setHeading(currentHeading_CCW + ROBOT_TURN_INCREMENT_DEGREES);
                // newFieldX and newFieldY remain unchanged for turns if setPosition is outside switch for movement
                moved = true; // Still counts as a move for UI update
                break;
            case RIGHT: // Turn right (decrease heading in CCW system)
                robot.setHeading(currentHeading_CCW - ROBOT_TURN_INCREMENT_DEGREES);
                moved = true; // Still counts as a move for UI update
                break;
            case A: // Strafe Left (90 degrees CCW from current heading)
                double strafeLeftAngleRad_CCW = Math.toRadians(currentHeading_CCW + 90.0);
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeLeftAngleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeLeftAngleRad_CCW);
                // robot.setPosition(newFieldX, newFieldY);
                moved = true;
                break;
            case D: // Strafe Right (90 degrees CW from current heading, or -90 CCW)
                double strafeRightAngleRad_CCW = Math.toRadians(currentHeading_CCW - 90.0);
                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeRightAngleRad_CCW);
                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeRightAngleRad_CCW);
                // robot.setPosition(newFieldX, newFieldY);
                moved = true;
                break;
            default:
                // Do nothing for other keys
                break;
        }

        if (moved) {
            // Set the robot's position only if it's a translational move (UP, DOWN, A, D)
            // For turns (LEFT, RIGHT), only the heading was changed.
            // We could also call setPosition for turns with currentX/Y, but it's redundant.
            if (event.getCode() == KeyCode.UP || event.getCode() == KeyCode.DOWN ||
                    event.getCode() == KeyCode.A || event.getCode() == KeyCode.D) {
                robot.setPosition(newFieldX, newFieldY);
            }

            // Ensure robot stays within field boundaries (optional, but good practice)
            // robot.setPosition(
            //    Math.max(-FIELD_HEIGHT_INCHES_VERTICAL_X / 2.0, Math.min(FIELD_HEIGHT_INCHES_VERTICAL_X / 2.0, robot.getXInches())),
            //    Math.max(-FIELD_WIDTH_INCHES_HORIZONTAL_Y / 2.0, Math.min(FIELD_WIDTH_INCHES_HORIZONTAL_Y / 2.0, robot.getYInches()))
            // );
            updateUIFromRobotState(); // This will typically redraw the field display
            event.consume();      // Consume the event if it resulted in robot movement
        }
    }



//    private void handleRobotMovementKeyPress(KeyEvent event) {
//        System.out.println("handleRobotMovementKeyPress checking for: " + event.getCode()); // DEBUG
//        if (robot == null || isCreatingPath) {
//            if(isCreatingPath) System.out.println("Robot movement skipped: isCreatingPath is true.");
//            if(robot == null) System.out.println("Robot movement skipped: robot is null.");
//            return;
//        }
//
//        double currentX = robot.getXInches();
//        double currentY = robot.getYInches();
//        double currentHeading_CCW = robot.getHeadingDegrees();
//        boolean moved = false;
//
//        // Calculate movement based on current heading for UP/DOWN
//        double moveX = 0;
//        double moveY = 0;
//        if (event.getCode() == KeyCode.UP || event.getCode() == KeyCode.DOWN) {
//            double angleRad = Math.toRadians(currentHeading_CCW);
//            double delta = (event.getCode() == KeyCode.UP) ? ROBOT_MOVE_INCREMENT_INCHES : -ROBOT_MOVE_INCREMENT_INCHES;
//            moveX = delta * Math.sin(angleRad); // Assuming 0 degrees is positive Y, 90 is positive X
//            moveY = delta * Math.cos(angleRad); // Adjust if your heading definition is different
//            // For typical math angles (0=East):
//            // moveX = delta * Math.cos(angleRad);
//            // moveY = delta * Math.sin(angleRad);
//            // Let's assume 0 deg is North (positive Y) for now
//            // and heading increases clockwise.
//            // So, X component is sin(heading), Y is cos(heading)
//            // but our canvas Y is inverted.
//            // Let's use a simpler field-centric forward/backward for now
//            // relative to screen, not robot heading, then refine.
//
//            // Simpler: Move along field axes (not robot-relative yet for simplicity)
//            // For robot-relative, we'd use trigonometry with the robot's heading.
//            // Let's do robot-relative forward/backward and left/right turns for now.
//        }
//
//        // currentHeading_CCW is robot.getHeadingDegrees() (0 = +FieldX/Down, CCW positive)
//        double angleRad_CCW = Math.toRadians(currentHeading_CCW);
//
//        switch (event.getCode()) {
//            case UP: // Move "forward" along the robot's current heading
//                // FieldX component = cos(angle), FieldY component = sin(angle)
//                // This is correct for a system where 0 degrees is along X and angles are CCW.
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
//                robot.setPosition(newFieldX, newFieldY);
//                moved = true;
//                break;
//            case DOWN: // Move "backward"
//                newFieldX = currentX - ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRad_CCW);
//                newFieldY = currentY - ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRad_CCW);
//                robot.setPosition(newFieldX, newFieldY);
//                moved = true;
//                break;
//            case LEFT: // Turn left (increase heading in CCW system)
//                robot.setHeading(currentHeading_CCW + ROBOT_TURN_INCREMENT_DEGREES); // CCW turn left = positive angle change
//                moved = true;
//                break;
//            case RIGHT: // Turn right (decrease heading in CCW system)
//                robot.setHeading(currentHeading_CCW - ROBOT_TURN_INCREMENT_DEGREES); // CCW turn right = negative angle change
//                moved = true;
//                break;
//            case A: // Strafe Left (90 degrees CCW from current heading)
//                // For a CCW system, strafing left means adding 90 degrees to the current heading
//                double strafeLeftAngleRad_CCW = Math.toRadians(currentHeading_CCW + 90.0);
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeLeftAngleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeLeftAngleRad_CCW);
//                robot.setPosition(newFieldX, newFieldY);
//                moved = true;
//                break;
//            case D: // Strafe Right (90 degrees CW from current heading, or -90 CCW)
//                double strafeRightAngleRad_CCW = Math.toRadians(currentHeading_CCW - 90.0);
//                newFieldX = currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(strafeRightAngleRad_CCW);
//                newFieldY = currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(strafeRightAngleRad_CCW);
//                robot.setPosition(newFieldX, newFieldY);
//                moved = true;
//                break;
////            case UP:
////                System.out.println("Robot Movement: UP");
//////                // Move forward relative to robot's heading
//////                double angleRadUp = Math.toRadians(currentHeading);
//////                // Standard geometry: 0 deg = +X axis. In our case, let's assume 0 deg = points "up" on screen (negative Y in math, positive Y in our field coords if 0,0 is top-left)
//////                // If 0 deg is "North" (along positive Y field axis):
//////                // X component = sin(heading), Y component = cos(heading)
//////                // However, our `inchesToPixelY` inverts Y. Let's adjust for robot's orientation.
//////                // Assuming 0 degrees is "up" on the screen (positive Y direction in INCHES)
////                robot.setPosition(
////                        currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(Math.toRadians(currentHeading)),
////                        currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(Math.toRadians(currentHeading))
////                );
////                moved = true;
////                break;
////            case DOWN:
////                System.out.println("Robot Movement: DOWN");
////                // Move backward relative to robot's heading
////                robot.setPosition(
////                        currentX - ROBOT_MOVE_INCREMENT_INCHES * Math.sin(Math.toRadians(currentHeading)),
////                        currentY - ROBOT_MOVE_INCREMENT_INCHES * Math.cos(Math.toRadians(currentHeading))
////                );
////                moved = true;
////                break;
////            case LEFT:
////                System.out.println("Robot Movement: LEFT");
////                robot.setHeading(currentHeading + ROBOT_TURN_INCREMENT_DEGREES);
////                moved = true;
////                break;
////            case RIGHT:
////                System.out.println("Robot Movement: RIGHT");
////                robot.setHeading(currentHeading - ROBOT_TURN_INCREMENT_DEGREES);
////                moved = true;
////                break;
////            case A: // Strafe Left
////                System.out.println("Robot Movement: STRAFE LEFT (A)");
////                double angleRadStrafeLeft = Math.toRadians(currentHeading + 90); // Perpendicular to heading
////                robot.setPosition(
////                        currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRadStrafeLeft),
////                        currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRadStrafeLeft)
////                );
////                moved = true;
////                break;
////            case D: // Strafe Right
////                System.out.println("Robot Movement: STRAFE RIGHT (D)");
////                double angleRadStrafeRight = Math.toRadians(currentHeading - 90); // Perpendicular to heading
////                robot.setPosition(
////                        currentX + ROBOT_MOVE_INCREMENT_INCHES * Math.sin(angleRadStrafeRight),
////                        currentY + ROBOT_MOVE_INCREMENT_INCHES * Math.cos(angleRadStrafeRight)
////                );
////                moved = true;
////                break;
//            default:
//                // Do nothing for other keys
//                break;
//        }
//
//        if (moved) {
//            // Ensure robot stays within field boundaries (optional, but good practice)
//            // robot.setPosition(
//            //    Math.max(0, Math.min(FIELD_WIDTH_INCHES, robot.getXInches())),
//            //    Math.max(0, Math.min(FIELD_HEIGHT_INCHES, robot.getYInches()))
//            // );
//            updateUIFromRobotState();
//            // Consume the event if it resulted in robot movement
//            event.consume();
//        }
//    }

    private void updateUIFromRobotState() {
        if (robot != null && controlPanel != null && fieldDisplay != null) {
            // Normalize heading to 0-359.9 degrees for display if desired
            double displayHeading = robot.getHeadingDegrees() % 360;
            if (displayHeading < 0) displayHeading += 360;

            controlPanel.updateRobotStatus(robot.getXInches(), robot.getYInches(), displayHeading);
            fieldDisplay.drawCurrentState();
        }
    }

    private void stopApp() {
        System.out.println("Window close requested / App stopping. Stopping UDP listener.");
        if (udpListener != null) {
            udpListener.stopListener();
        }
        if (udpListenerThread != null && udpListenerThread.isAlive()) {
            try {
                udpListenerThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                System.err.println("Interrupted while waiting for UDP listener thread to stop.");
            }
        }
        System.out.println("Exiting application.");
        Platform.exit(); // Ensure JavaFX platform exits cleanly
        System.exit(0); // Force exit if needed
    }

    // Make sure to stop the listener when the application exits
    @Override
    public void stop() throws Exception {
        stopApp(); // Call our common stop logic
        super.stop(); // Call superclass method
//        System.out.println("Application stop method called. Stopping UDP listener.");
//        if (udpListener != null) {
//            udpListener.stopListener();
//        }
//        if (udpListenerThread != null && udpListenerThread.isAlive()) {
//            try {
//                udpListenerThread.join(1000); // Wait a bit for the thread to finish
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt(); // Preserve interrupt status
//                System.err.println("UDP listener thread interrupted during stop.");
//            }
//        }
//        super.stop(); // Important to call super.stop()
//        System.out.println("Application stopped.");
    }

    // --- Call updateUIFromRobotState() whenever the robot's state changes ---
    // Example method that changes robot state - this would be called by actual controls
    public void moveRobotProgrammatically(double newX, double newY, double newHeading) {
        if (robot != null) {
            robot.setPosition(newX, newY);
            robot.setHeading(newHeading);
            updateUIFromRobotState();
        }
    }
    private List<CurvePoint> createDefaultSPath() {
        List<CurvePoint> path = new ArrayList<>();
        double margin = FIELD_WIDTH_INCHES * 0.10;
        double pathMinX = margin;
        double pathMaxX = FIELD_WIDTH_INCHES - margin;
        double pathMinY = margin;
        double pathMaxY = FIELD_HEIGHT_INCHES - margin;
        double pathWidth = pathMaxX - pathMinX;
        double pathHeight = pathMaxY - pathMinY;

        path.add(new CurvePoint(pathMinX, pathMinY));
        path.add(new CurvePoint(pathMaxX, pathMinY + pathHeight * 0.25));
        path.add(new CurvePoint(pathMinX, pathMinY + pathHeight * 0.50));
        path.add(new CurvePoint(pathMaxX, pathMinY + pathHeight * 0.75));
        path.add(new CurvePoint(pathMinX, pathMaxY));
        path.add(new CurvePoint(pathMinX + pathWidth * 0.25, pathMaxY));
        return path;
    }
    private Robot createInitialRobot(List<CurvePoint> path) {
        if (path != null && !path.isEmpty()) {
            return new Robot(path.get(0).x, path.get(0).y, 0.0, ROBOT_IMAGE_PATH);
        } else {
            return new Robot(
                    FIELD_WIDTH_INCHES / 2.0,
                    FIELD_HEIGHT_INCHES / 2.0,
                    0.0,
                    ROBOT_IMAGE_PATH
            );
        }
    }
}
