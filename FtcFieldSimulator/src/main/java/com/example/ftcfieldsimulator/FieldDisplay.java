package com.example.ftcfieldsimulator; // Adjust package as needed

import javafx.geometry.Point2D; // Using Point2D for trail dots (pixel coordinates)
import javafx.scene.Cursor; // For changing cursor
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label; // For coordinate display
import javafx.scene.image.Image;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent; // For mouse events
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.TextAlignment; // For centering text
import javafx.scene.transform.Rotate;

// import javafx.geometry.Point2D; // If using this instead of CurvePoint for path

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.function.Consumer; // For click callback

import com.example.ftcfieldsimulator.UdpPositionListener.LineStyle;

public class FieldDisplay extends Pane {
    private final Canvas canvas; // The drawing surface
    private final GraphicsContext gc; // GraphicsContext of the canvas

    private final Label coordinateLabel; // Label to display mouse coordinates

    private final double fieldWidthInches;
    private final double fieldHeightInches;

    private Image fieldImage;
    private double fieldImageAlpha; // To store the alpha for the field image
    private double backgroundAlpha;
    private Robot robot;
    private List<CurvePoint> currentPath = new ArrayList<>();

    // Scaling factors
    private double scaleX; // pixels per inch
    private double scaleY; // pixels per inch

    private List<Point2D> robotTrailDots; // Stores pixel coordinates of trail dots
    private static final int MAX_TRAIL_DOTS = 1000; // Limit the number of dots
    private static final double TRAIL_DOT_RADIUS_PIXELS = 3.0;

    // Define simple inner classes or records for these shapes (if not already defined elsewhere)
// These will store coordinates in INCHES (field-relative, center origin)
    private static class DebugCircle {
        final double centerXInches, centerYInches, radiusInches;
        final double headingDegrees;
        final Color color; // Optional: allow color specification

        DebugCircle(double centerXInches, double centerYInches, double radiusInches, double headingDegrees, Color color) {            this.centerXInches = centerXInches;
            this.centerYInches = centerYInches;
            this.radiusInches = radiusInches;
            this.headingDegrees = headingDegrees;
            this.color = color;
        }
    }
    private DebugCircle currentDebugCircle = null; // Stores only the latest circle
    private final Object debugCircleLock = new Object(); // For synchronizing access if needed

    private Map<String, UdpPositionListener.LineData> namedLinesMap = new HashMap<>();
    private final Object namedLinesMapLock = new Object(); // For synchronizing access if setNamedLinesMap can be called from non-FX thread.

    private List<CurvePoint> currentPathToDraw = new ArrayList<>(); // Path to be drawn
    // Callback for when the field is clicked during path creation
    private Consumer<Point2D> onFieldPointClickListener;
    private Runnable onPathFinishListener; // New callback for finishing path
    private boolean isPathCreationMode = false;

    // +++ For Robot Text Message +++
    private String currentRobotTextMessage = null;
    private final Object robotTextLock = new Object();
    private static final Font ROBOT_TEXT_FONT = Font.font("Arial", 12);
    private static final Color ROBOT_TEXT_COLOR = Color.LIGHTGRAY;
    private static final Color ROBOT_TEXT_BACKGROUND_COLOR = Color.rgb(0, 0, 0, 0.5); // Similar to coordinateLabel
    private static final double ROBOT_TEXT_PADDING = 3.0;
    private static final double ROBOT_TEXT_Y_OFFSET_PIXELS = 20; // Offset below the robot center

    public FieldDisplay(int canvasWidthPixels, int canvasHeightPixels,
                        double fieldWidthInches, double fieldHeightInches,
                        String fieldImagePath, Robot robot, double backgroundAlpha,double fieldImageAlphaFromApp) {
//        super(canvasWidthPixels, canvasHeightPixels);
        this.fieldWidthInches = fieldWidthInches;
        this.fieldHeightInches = fieldHeightInches;
        this.robot = robot;

        this.scaleX = canvasWidthPixels / fieldWidthInches;
        this.scaleY = canvasHeightPixels / fieldHeightInches;
        this.fieldImageAlpha = fieldImageAlphaFromApp;
        this.backgroundAlpha = backgroundAlpha;
        this.fieldImage = ImageLoader.loadImage(
                fieldImagePath, // Ensure leading slash for classpath root
                canvasWidthPixels,
                canvasHeightPixels,
//                (int) getWidth(),     // Use canvas width for placeholder
//                (int) getHeight(),    // Use canvas height for placeholder
                "Field Image",
                Color.LIGHTSLATEGRAY
        );

        this.robotTrailDots = Collections.synchronizedList(new ArrayList<>()); // Thread-safe list

        // --- Setup Canvas ---
        this.canvas = new Canvas(canvasWidthPixels, canvasHeightPixels);
        this.gc = this.canvas.getGraphicsContext2D();
        this.getChildren().add(this.canvas); // Add canvas to the Pane

        // --- Setup Coordinate Label ---
        this.coordinateLabel = new Label("");
        this.coordinateLabel.setFont(Font.font("Arial", 12));
        this.coordinateLabel.setTextFill(Color.LIGHTGRAY);
        this.coordinateLabel.setStyle("-fx-background-color: rgba(0, 0, 0, 0.5); -fx-padding: 3px; -fx-border-radius: 3px; -fx-background-radius: 3px;");
        this.coordinateLabel.setVisible(false); // Initially hidden
        this.coordinateLabel.setMouseTransparent(true); // So it doesn't interfere with canvas clicks
        this.getChildren().add(this.coordinateLabel); // Add label to the Pane (will float on top)


        // --- Make Canvas Focusable ---
        canvas.setFocusTraversable(true);

        // --- MOUSE CLICKED on Canvas ---
        this.canvas.addEventHandler(MouseEvent.MOUSE_CLICKED, event -> {
            // --- Request focus whenever the canvas is clicked ---
            // This ensures keyboard events are directed here after a click,
            // useful for robot movement when not in path creation mode.
            if (!canvas.isFocused()) { // Only request if not already focused
                canvas.requestFocus();
                System.out.println("FieldDisplay requested focus.");
            }

            if (isPathCreationMode) {
                if (event.getButton() == MouseButton.PRIMARY) { // Ensure left click
                    if (event.getClickCount() == 2) {
                        // Double-click detected
                        if (onPathFinishListener != null) {
                            System.out.println("FieldDisplay: Double click detected, invoking onPathFinishListener.");
                            onPathFinishListener.run(); // Signal to finish path
                        }
                    } else {
                        // Single-click
                        if (onFieldPointClickListener != null) {
                            onFieldPointClickListener.accept(new Point2D(event.getX(), event.getY()));
                        }
                    }
                }
            }
        });

        // --- MOUSE MOVED on Canvas (for coordinate display) ---
        this.canvas.addEventHandler(MouseEvent.MOUSE_MOVED, event -> {
//            if (isPathCreationMode) {
                Point2D fieldCoords = pixelToInches(event.getX(), event.getY());
                coordinateLabel.setText(String.format("X: %.1f, Y: %.1f", fieldCoords.getX(), fieldCoords.getY()));

                // Position the label slightly below and to the right of the mouse cursor
                // Adjust offsets as needed for best appearance with your crosshair
                double labelOffsetX = 15;
                double labelOffsetY = 15; // Offset below the cursor hotspot

                // Ensure label stays within canvas bounds (simple check)
                double newX = Math.min(event.getX() + labelOffsetX, canvas.getWidth() - coordinateLabel.prefWidth(-1) - 5);
                newX = Math.max(5, newX); // Keep it from going off the left
                double newY = Math.min(event.getY() + labelOffsetY, canvas.getHeight() - coordinateLabel.prefHeight(-1) - 5);
                newY = Math.max(5, newY); // Keep it from going off the top

                coordinateLabel.setLayoutX(newX);
                coordinateLabel.setLayoutY(newY);
                coordinateLabel.setVisible(true);
//            } else {
//                coordinateLabel.setVisible(false);
//            }
        });

        // --- MOUSE EXITED Canvas ---
        this.canvas.addEventHandler(MouseEvent.MOUSE_EXITED, event -> {
            if (isPathCreationMode) {
                coordinateLabel.setVisible(false);
            }
        });
    }

    public void addDebugCircle(double centerXInches, double centerYInches, double radiusInches, double headingDegrees, Color color) {
        synchronized (debugCircleLock) {
            currentDebugCircle = new DebugCircle(centerXInches, centerYInches, radiusInches, headingDegrees, color);
            // No need to call drawCurrentState() here; let the app manage update frequency.
        }
    }

    public void setNamedLinesMap(Map<String, UdpPositionListener.LineData> linesMap) {
        // This method should ideally be called from the JavaFX Application Thread.
        // If there's a possibility of it being called from another thread, synchronization is needed.
        // However, FtcFieldSimulatorApp typically calls this once during setup or updates the map
        // it holds, and FieldDisplay just reads from it during drawing on the FX thread.
        // For simplicity, direct assignment. If map itself is modified externally and concurrently,
        // then the map passed in should be thread-safe (like ConcurrentHashMap) or access here synchronized.
        synchronized(namedLinesMapLock) { // Synchronize if this method can be called from non-FX thread
            this.namedLinesMap = linesMap;
        }
    }

    public void clearDebugCircle() {
        synchronized (debugCircleLock) {
            currentDebugCircle = null;
        }
    }

    public void setRobotTextMessage(String text) {
        synchronized (robotTextLock) {
            this.currentRobotTextMessage = text;
        }
    }

    public void clearRobotTextMessage() {
        synchronized (robotTextLock) {
            currentRobotTextMessage = null;
        }
    }

    // Method to convert pixel coordinates to field inches
    public Point2D pixelToInches(double pixelX, double pixelY) {
//        double inchesX = pixelX / scaleX;
//        double inchesY = (getHeight() - pixelY) / scaleY; // Flipped Y
//        return new Point2D(inchesX, inchesY);
        return new Point2D(pixelToInchesX(pixelX), pixelToInchesY(pixelY));
    }

    public void setPathToDraw(List<CurvePoint> path) {
        this.currentPathToDraw = (path != null) ? new ArrayList<>(path) : new ArrayList<>();
    }

    public void setPathCreationMode(boolean isActive,
                                    Consumer<Point2D> pointClickListener,
                                    Runnable pathFinishListener) {
        this.isPathCreationMode = isActive;
        this.onFieldPointClickListener = isActive ? pointClickListener : null;
        this.onPathFinishListener = isActive ? pathFinishListener : null;
        canvas.setCursor(isActive ? Cursor.CROSSHAIR : Cursor.DEFAULT);
        if (!isActive) {
            coordinateLabel.setVisible(false); // Hide label when not in mode
        }
    }

    public void addTrailDot(double robotXInchesField, double robotYInchesField) { // Input is now field center relative
        if (robotTrailDots.size() >= MAX_TRAIL_DOTS) {
            if (!robotTrailDots.isEmpty()) robotTrailDots.remove(0);
        }
        // Convert robot's current field-center-relative inch position to pixel coordinates for the dot
        robotTrailDots.add(new Point2D(inchesToPixelX(robotXInchesField), inchesToPixelY(robotYInchesField)));
    }

    public void clearTrail() {
        robotTrailDots.clear();
        // The drawCurrentState() method, when next called, will see an empty list
        // and thus won't draw any trail.
    }

//    // --- Method to add a trail dot (called from FtcFieldSimulatorApp) ---
//    public void addTrailDot(double robotXInches, double robotYInches) {
//        if (robotTrailDots.size() >= MAX_TRAIL_DOTS) {
//            robotTrailDots.removeFirst(); // Remove the oldest dot if limit is reached
//        }
//        // Convert robot's current inch position to pixel coordinates for the dot
//        robotTrailDots.add(new Point2D(inchesToPixelX(robotXInches), inchesToPixelY(robotYInches)));
//    }

    public void clearTrailDots() {
        robotTrailDots.clear();
        // drawCurrentState(); // Optionally redraw immediately
    }

    public void setPath(List<CurvePoint> path) {
        this.currentPath = (path != null) ? new ArrayList<>(path) : new ArrayList<>();
        // drawCurrentState(); // Optionally redraw immediately
    }

    // TODO: Add updateRobotPosition(double xInches, double yInches, double headingDegrees)

    /**
     * Converts an X-coordinate from field inches (origin at center, +X right)
     * to canvas pixel X-coordinate (origin top-left, +X right).
     * @param xInchesField Robot's X in inches, relative to the center of the field.
     * @return Pixel X-coordinate on the canvas.
     */
    private double inchesToPixelX(double xInchesField) {
        // 1. Convert field X (center origin) to an X relative to left edge of field (still in inches)
        //    x_left_origin_inches = xInchesField + (fieldWidthInches / 2.0)
        // 2. Scale to pixels
        //    pixelX = (xInchesField + (fieldWidthInches / 2.0)) * scaleX;
        double canvasCenterX = canvas.getWidth() / 2.0;
        return canvasCenterX + (xInchesField * scaleX);
    }

    /**
     * Converts a Y-coordinate from field inches (origin at center, +Y up)
     * to canvas pixel Y-coordinate (origin top-left, +Y down).
     * @param yInchesField Robot's Y in inches, relative to the center of the field.
     * @return Pixel Y-coordinate on the canvas.
     */
    private double inchesToPixelY(double yInchesField) {
        // 1. Convert field Y (center origin, +Y up) to an Y relative to top edge of field (+Y down, still in inches)
        //    y_top_origin_inches_positive_down = (fieldHeightInches / 2.0) - yInchesField
        // 2. Scale to pixels
        //    pixelY = ((fieldHeightInches / 2.0) - yInchesField) * scaleY;
        double canvasCenterY = canvas.getHeight() / 2.0;
        return canvasCenterY - (yInchesField * scaleY); // Subtract because +Y field is up, +Y canvas is down
    }


    /**
     * Converts a canvas pixel X-coordinate (origin top-left, +X right)
     * to field X-coordinate in inches (origin at center, +X right).
     * @param pixelX Pixel X-coordinate on the canvas.
     * @return Field X-coordinate in inches.
     */
    public double pixelToInchesX(double pixelX) {
        // 1. Convert pixel X to inches from left edge
        //    x_left_origin_inches = pixelX / scaleX;
        // 2. Convert to inches from center
        //    x_center_origin_inches = x_left_origin_inches - (fieldWidthInches / 2.0);
        double canvasCenterX = canvas.getWidth() / 2.0;
        return (pixelX - canvasCenterX) / scaleX;
    }

    /**
     * Converts a canvas pixel Y-coordinate (origin top-left, +Y down)
     * to field Y-coordinate in inches (origin at center, +Y up).
     * @param pixelY Pixel Y-coordinate on the canvas.
     * @return Field Y-coordinate in inches.
     */
    public double pixelToInchesY(double pixelY) {
        // 1. Convert pixel Y to inches from top edge (+Y down)
        //    y_top_origin_inches_positive_down = pixelY / scaleY;
        // 2. Convert to inches from center (+Y up)
        //    y_center_origin_inches_positive_up = (fieldHeightInches / 2.0) - y_top_origin_inches_positive_down;
        double canvasCenterY = canvas.getHeight() / 2.0;
        return -(pixelY - canvasCenterY) / scaleY; // The negative sign handles the flip from canvas Y-down to field Y-up
    }

//    @Override
    public void drawCurrentState() {
//        GraphicsContext gc = getGraphicsContext2D();

        // Clear the entire canvas for a fresh draw on each frame/update
        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

        // Clear Canvas (or draw background color if image is transparent/smaller)
        Color bgColor = Color.rgb(50, 50, 50, this.backgroundAlpha);
        gc.setFill(bgColor);
        gc.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());

        // Draw Field Image
        if (fieldImage != null) {
            // Save the current global alpha
            double originalAlpha = gc.getGlobalAlpha();

            // Set the desired alpha for the field image
            gc.setGlobalAlpha(this.fieldImageAlpha); // Use the instance variable

            // Draw the image to fill the entire canvas.
            // Assumes fieldImage aspect ratio matches fieldWidthInches/fieldHeightInches.
            // If not, more sophisticated scaling/letterboxing might be needed.
            gc.drawImage(fieldImage, 0, 0, canvas.getWidth(), canvas.getHeight());

            // IMPORTANT: Restore the original global alpha
            gc.setGlobalAlpha(originalAlpha);
        }

        // Draw the CURRENT PATH being created or displayed
        if (currentPathToDraw != null && !currentPathToDraw.isEmpty()) {
            gc.setStroke(Color.ORANGERED); // Color for the interactive path
            gc.setLineWidth(3.0);
            gc.setLineDashes(isPathCreationMode ? 5 : 0); // Dashed line while creating

            gc.beginPath();
            CurvePoint firstP = currentPathToDraw.get(0);
            gc.moveTo(inchesToPixelX(firstP.x), inchesToPixelY(firstP.y));
            for (int i = 1; i < currentPathToDraw.size(); i++) {
                CurvePoint p = currentPathToDraw.get(i);
                gc.lineTo(inchesToPixelX(p.x), inchesToPixelY(p.y));
            }
            gc.stroke();
            gc.setLineDashes(0); // Reset dashes

            // Draw waypoints
            gc.setFill(Color.RED);
            for (CurvePoint p : currentPathToDraw) {
                gc.fillOval(inchesToPixelX(p.x) - 4, inchesToPixelY(p.y) - 4, 8, 8);
            }
        }

        // Draw Robot Image
        double robotCenterXPixels = 0; // Initialize to avoid issues if robot is null
        double robotCenterYPixels = 0;

        // Check if the robot object and its image are available
        if (robot != null) {
            javafx.scene.image.Image robotImg = robot.getRobotImage(); // Get the loaded robot image

            // Step 4a: Calculate the robot's dimensions in PIXELS
            // We use the robot's defined size in INCHES and our pixels-per-inch scaling factors
            double robotWidthPixels = Robot.ROBOT_WIDTH_INCHES * scaleX;
            double robotHeightPixels = Robot.ROBOT_HEIGHT_INCHES * scaleY;

            // Step 4b: Get the robot's desired center position in INCHES from the Robot object
            double robotXInches = robot.getXInches();
            double robotYInches = robot.getYInches();

            // Step 4c: Convert the robot's center position from INCHES to screen PIXELS
            // This uses the methods we defined earlier to handle field coordinate system
            robotCenterXPixels = inchesToPixelX(robotXInches);
            robotCenterYPixels = inchesToPixelY(robotYInches);

            // Step 4d: Get the robot's heading in DEGREES
            double robotHeadingDegrees = robot.getHeadingDegrees();

            // Step 4e: Perform the drawing with rotation
            gc.save(); // IMPORTANT: Save the current state of the graphics context (transformations, colors, etc.)

            // Create a rotation transformation:
            // - robotHeadingDegrees: The angle to rotate.
            // - robotCenterXPixels: The X pivot point for rotation on the canvas.
            // - robotCenterYPixels: The Y pivot point for rotation on the canvas.
            Rotate rotateTransform = new Rotate(robotHeadingDegrees, robotCenterXPixels, robotCenterYPixels);

            // Apply this rotation transform to the graphics context
            // This modifies how subsequent drawing operations are rendered
            gc.setTransform(
                    rotateTransform.getMxx(), rotateTransform.getMyx(),
                    rotateTransform.getMxy(), rotateTransform.getMyy(),
                    rotateTransform.getTx(), rotateTransform.getTy()
            );

            // Draw the robot image:
            // The image is drawn with its top-left corner at the specified coordinates.
            // Since we've translated and rotated the coordinate system such that the
            // (robotCenterXPixels, robotCenterYPixels) is now the effective (0,0) for rotation,
            // we need to draw the image offset by half its width and height
            // to make it appear centered around this pivot point.
            gc.drawImage(
                    robotImg, // The image to draw
                    robotCenterXPixels - robotWidthPixels / 2.0, // X position for top-left of image
                    robotCenterYPixels - robotHeightPixels / 2.0, // Y position for top-left of image
                    robotWidthPixels,  // Desired width of the drawn image on canvas (scaled)
                    robotHeightPixels  // Desired height of the drawn image on canvas (scaled)
            );

            gc.restore(); // IMPORTANT: Restore the graphics context to its state BEFORE gc.save()
            // This removes the rotation and translation we applied, so other
            // things (like text or other UI elements if any) are drawn normally.
        }

        // Draw Robot Trail Dots
        gc.setFill(Color.YELLOW);
        // Iterate safely if modifications can happen from another thread,
        // though addTrailDot should be called via Platform.runLater.
        // A simple loop is fine if addTrailDot is always on FX thread.
        synchronized (robotTrailDots) { // Synchronize access if addTrailDot might not always be on FX thread
            for (Point2D dot : robotTrailDots) {
                gc.fillOval(
                        dot.getX() - TRAIL_DOT_RADIUS_PIXELS,
                        dot.getY() - TRAIL_DOT_RADIUS_PIXELS,
                        TRAIL_DOT_RADIUS_PIXELS * 2,
                        TRAIL_DOT_RADIUS_PIXELS * 2
                );
            }
        }

        // +++ Draw Named, Styled Lines +++
        // Define default colors for styles (can be customized further)
        final Color style1Color = Color.CYAN.deriveColor(0, 1, 1, 0.9); // Semi-transparent Cyan
//        final Color style2Color = Color.LIGHTGREEN.deriveColor(0, 1, 1, 0.8); // Semi-transparent Light Green
        final Color style2Color = Color.CYAN.deriveColor(0, 1, 1, 0.9); // Semi-transparent Cyan
        final Color style3ColorDotted = Color.LIGHTPINK.deriveColor(0, 1, 1, 0.85); // Semi-transparent Light Pink

        // Iterate over a copy of the values if the map could be modified by another thread
        // during this iteration. However, if updates to namedLinesMap in FtcFieldSimulatorApp
        // and this drawing are both on the FX thread, direct iteration is fine.
        // For safety, iterating over a copy of values collected within a synchronized block is robust.
        List<UdpPositionListener.LineData> linesToRender;
        synchronized (namedLinesMapLock) { // Use the lock if namedLinesMap can be modified from non-FX thread.
            // Or if setNamedLinesMap can be called concurrently.
            linesToRender = new ArrayList<>(namedLinesMap.values());
        }

        if (linesToRender != null) { // Check if the map itself is null, though it's initialized in FtcFieldSimulatorApp
            for (UdpPositionListener.LineData lineData : linesToRender) {
                if (lineData == null) continue; // Should not happen if map doesn't store null values

                LineStyle style = lineData.style;
                double x0_px = inchesToPixelX(lineData.x0);
                double y0_px = inchesToPixelY(lineData.y0);
                double x1_px = inchesToPixelX(lineData.x1);
                double y1_px = inchesToPixelY(lineData.y1);

                switch (style) {
                    case SOLID_THICK: // Style 1
                        gc.setStroke(style1Color);
                        gc.setLineWidth(3.0);
                        gc.setLineDashes(0); // Ensure solid
                        gc.strokeLine(x0_px, y0_px, x1_px, y1_px);
                        break;
                    case SOLID_THIN: // Style 2
                        gc.setStroke(style2Color);
                        gc.setLineWidth(1.5);
                        gc.setLineDashes(0); // Ensure solid
                        gc.strokeLine(x0_px, y0_px, x1_px, y1_px);
                        break;
                    case DOTTED: // Style 3
                        gc.setStroke(style3ColorDotted);
                        gc.setLineWidth(1.5); // Dotted lines often look better slightly thinner
                        gc.setLineDashes(5, 5); // 5px line, 5px gap
                        gc.strokeLine(x0_px, y0_px, x1_px, y1_px);
                        gc.setLineDashes(0); // IMPORTANT: Reset for subsequent draws
                        break;
                    default:
                        // Default to a basic solid line if style is somehow null or unrecognized
                        gc.setStroke(Color.WHITE);
                        gc.setLineWidth(1.0);
                        gc.setLineDashes(0);
                        gc.strokeLine(x0_px, y0_px, x1_px, y1_px);
                        break;
                }
            }
        }

        // +++ Draw Robot Text Message +++
        String textToDraw;
        synchronized (robotTextLock) {
            textToDraw = currentRobotTextMessage;
        }
        if (textToDraw != null && !textToDraw.isEmpty() && robot != null) {
            // Recalculate robot center if not already done, or use stored from robot drawing
            // Ensure robotCenterXPixels and robotCenterYPixels are from the *un-rotated* context for text placement
            robotCenterXPixels = inchesToPixelX(robot.getXInches()); // Get current robot center in pixels
            robotCenterYPixels = inchesToPixelY(robot.getYInches());

            gc.setFont(ROBOT_TEXT_FONT);
            gc.setTextAlign(TextAlignment.CENTER); // Center the text horizontally

            // Estimate text width and height for background
            // JavaFX doesn't have a direct gc.measureText equivalent like some other frameworks.
            // A common workaround is to use a javafx.scene.text.Text node, but for simplicity here:
            // we'll make a rough estimate or use a fixed background size if text length varies too much.
            // For a more precise background, you'd create a Text node, apply font, get its layout bounds.
            double textWidthEstimate = textToDraw.length() * ROBOT_TEXT_FONT.getSize() * 0.6; // Very rough estimate
            double textHeight = ROBOT_TEXT_FONT.getSize();

            double backgroundWidth = textWidthEstimate + 2 * ROBOT_TEXT_PADDING;
            double backgroundHeight = textHeight + 2 * ROBOT_TEXT_PADDING;

            double textBackgroundX = robotCenterXPixels - backgroundWidth / 2;
            // Position background and text below the robot center
            double textBackgroundY = robotCenterYPixels + ROBOT_TEXT_Y_OFFSET_PIXELS;

            // Draw background
            gc.setFill(ROBOT_TEXT_BACKGROUND_COLOR);
            gc.fillRoundRect(textBackgroundX, textBackgroundY, backgroundWidth, backgroundHeight, 6, 6); // Rounded corners

            // Draw text
            gc.setFill(ROBOT_TEXT_COLOR);
            gc.fillText(
                    textToDraw,
                    robotCenterXPixels, // Centered horizontally
                    textBackgroundY + ROBOT_TEXT_PADDING + textHeight * 0.8 // Adjust Y for text baseline
            );
        }

        // --- Draw Debug Circle (Singular) with Heading Line ---
        DebugCircle circleToDraw;
        synchronized (debugCircleLock) {
            circleToDraw = this.currentDebugCircle; // Get the current circle safely
        }
        if (circleToDraw != null) {
            Color circleColor = circleToDraw.color != null ? circleToDraw.color : Color.MAGENTA;
            gc.setStroke(circleColor);
            gc.setLineWidth(3);

            double pixelCenterX = inchesToPixelX(circleToDraw.centerXInches);
            double pixelCenterY = inchesToPixelY(circleToDraw.centerYInches);
            double pixelRadius = circleToDraw.radiusInches * scaleX; // Assuming uniform scaling, use scaleX or average

            // Draw the circle
            gc.strokeOval(
                    pixelCenterX - pixelRadius,
                    pixelCenterY - pixelRadius,
                    pixelRadius * 2,
                    pixelRadius * 2
            );

            // Draw the heading line
            // Convert heading from degrees to radians.
            // Adjust for JavaFX canvas coordinate system if needed (0 degrees is typically along the positive X-axis).
            // If your heading means 0 degrees is "North" (positive Y field axis, which becomes negative Y math axis),
            // then: X component = sin(angle), Y component = -cos(angle) for math coordinates.
            // Let's assume heading is standard mathematical angle for simplicity first (0 degrees = +X)
            // If your simulator's "heading 0" is "up" (positive Y field / negative Y math), adjust angle calculation.
            // For 0 degrees = positive Y field axis (pointing "up" on your field display before canvas flip):
            // Math angle for calculation needs to be (90 - heading) because math 0 is right, field 0 is up.
            // Or, X = R*sin(heading_rad_field), Y = -R*cos(heading_rad_field) due to canvas Y inversion.

            double headingRad = Math.toRadians(circleToDraw.headingDegrees);

            // Assuming 0 degrees for heading points UP (positive Y in your field inch system)
            // then in standard math angle (where 0 is +X axis), this is 90 degrees.
            // So, effective math angle = Math.toRadians(90 - circleToDraw.headingDegrees)
            // Or, more directly for components:
            // For heading 0 = "up" (positive Y field):
            // endX = centerX + radius * sin(headingRad)
            // endY = centerY - radius * cos(headingRad) (minus because positive math Y is down on canvas)

            double lineEndXPixel = pixelCenterX + pixelRadius * Math.sin(headingRad);
            double lineEndYPixel = pixelCenterY - pixelRadius * Math.cos(headingRad); // Subtract because screen Y is inverted relative to standard math coordinates

            gc.beginPath();
            gc.moveTo(pixelCenterX, pixelCenterY);
            gc.lineTo(lineEndXPixel, lineEndYPixel);
            gc.stroke(); // Uses the same color and stroke width as the circle
        }

    }
}
