package com.example.ftcfieldsimulator; // Adjust package as needed

import javafx.geometry.Point2D;
import javafx.scene.Cursor;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color; // Keep Color import
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.text.TextAlignment;
import javafx.geometry.VPos;
import javafx.scene.transform.Rotate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.function.Consumer;

//// Import the LineStyle enum from UdpPositionListener
//import com.example.ftcfieldsimulator.UdpPositionListener.LineStyle;

public class FieldDisplay extends Pane {
    private final Canvas canvas;
    private final GraphicsContext gc;
    private final Label coordinateLabel;

    // These remain the logical dimensions of the field itself.
    // fieldWidthInches is the extent along the new HORIZONTAL Field Y-axis.
    // fieldHeightInches is the extent along the new VERTICAL Field X-axis.
    private final double fieldWidthInches_HorizontalY;
    private final double fieldHeightInches_VerticalX;

    private Image fieldImage;
    private double fieldImageAlpha;
    private double backgroundAlpha;
    private Robot robot;
    // currentPath stores points as (FieldX, FieldY) based on the new system
    private List<CurvePoint> currentPath = new ArrayList<>();


    // Renamed scaling factors for clarity with the new coordinate system
    private double scaleForFieldX_Vertical;  // Canvas pixels (vertical) per inch of Field X (vertical)
    private double scaleForFieldY_Horizontal; // Canvas pixels (horizontal) per inch of Field Y (horizontal)

    // robotTrailDots stores points in CANVAS PIXEL coordinates
    private List<Point2D> robotTrailDots;
    private static final int MAX_TRAIL_DOTS = 1000;
    private static final double TRAIL_DOT_RADIUS_PIXELS = 4.0;

    public enum LineStyle {
        SOLID_THICK(1),  // Style 1: Solid, 3px width (default color to be decided by renderer)
        SOLID_THIN(2),   // Style 2: Solid, 1.5px width (different default color by renderer)
        DOTTED(3);       // Style 3: Dotted

        private final int styleCode;
        LineStyle(int code) { this.styleCode = code; }
        public int getStyleCode() { return styleCode; }

        public static LineStyle fromCode(int code) {
            for (LineStyle style : values()) {
                if (style.styleCode == code) {
                    return style;
                }
            }
            return SOLID_THICK; // Default if code is invalid
        }
    }

    private static class DebugCircle {
        // centerXInches is new Field X (vertical), centerYInches is new Field Y (horizontal)
        final double fieldX, fieldY, radiusInches;
        final double headingDegrees; // Assumed to be relative to new Field X (0 deg = down screen)
        final Color color;

        DebugCircle(double fieldX, double fieldY, double radiusInches, double headingDegrees, Color color) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.radiusInches = radiusInches;
            this.headingDegrees = headingDegrees;
            this.color = color;
        }
    }
    private DebugCircle currentDebugCircle = null;
    private final Object debugCircleLock = new Object();

    private Map<String, UdpPositionListener.LineData> namedLinesMap = new HashMap<>();
    private final Object namedLinesMapLock = new Object();

    // currentPathToDraw stores points as (FieldX, FieldY) based on the new system
    private List<CurvePoint> currentPathToDraw = new ArrayList<>();
    private Consumer<Point2D> onFieldPointClickListener;
    private Runnable onPathFinishListener;
    private boolean isPathCreationMode = false;

    private String currentRobotTextMessage = null;
    private final Object robotTextLock = new Object();
    private static final Font ROBOT_TEXT_FONT = Font.font("Arial", 12);
    private static final Color ROBOT_TEXT_COLOR = Color.LIGHTGRAY;
    private static final Color ROBOT_TEXT_BACKGROUND_COLOR = Color.rgb(0, 0, 0, 0.5);
    private static final double ROBOT_TEXT_PADDING = 3.0;
    // This offset is in canvas pixels, its direction (e.g., "below") is relative to robot's canvas orientation
    private static final double ROBOT_TEXT_Y_CANVAS_OFFSET_PIXELS = 20;

    // Font for axis labels and numbers
    private static final Font AXIS_LABEL_FONT = Font.font("Arial", FontWeight.BOLD, 14);
    private static final Font AXIS_NUMBER_FONT = Font.font("Arial", 10);
//    private static final Color AXIS_COLOR = Color.rgb(200, 200, 200, 0.75); // Light gray, semi-transparent
    private static final Color AXIS_COLOR = Color.rgb(100, 100, 100, 0.75); // Medium-Dark Gray, still semi-transparent
//    private static final Color AXIS_COLOR = Color.rgb(80, 80, 80, 0.9); // Dark Gray, more opaque
    private static final double AXIS_LINE_WIDTH = 2;
    private static final double TICK_LENGTH_PIXELS = 5.0;
    private static final double LABEL_OFFSET_PIXELS = 15.0; // Offset for "X", "Y" labels from axis end
    private static final double NUMBER_OFFSET_PIXELS = 8.0; // Offset for numbers from ticks

    public FieldDisplay(int canvasWidthPixels, int canvasHeightPixels,
                        double fieldWidthInches,  // This is for the field's HORIZONTAL extent (new Y-axis)
                        double fieldHeightInches, // This is for the field's VERTICAL extent (new X-axis)
                        String fieldImagePath, Robot robot, double backgroundAlpha, double fieldImageAlphaFromApp) {

        this.fieldWidthInches_HorizontalY = fieldWidthInches;
        this.fieldHeightInches_VerticalX = fieldHeightInches;
        this.robot = robot;

        // Field X (vertical on field) scales with canvas height.
        this.scaleForFieldX_Vertical = (double) canvasHeightPixels / this.fieldHeightInches_VerticalX;
        // Field Y (horizontal on field) scales with canvas width.
        this.scaleForFieldY_Horizontal = (double) canvasWidthPixels / this.fieldWidthInches_HorizontalY;

        this.fieldImageAlpha = fieldImageAlphaFromApp;
        this.backgroundAlpha = backgroundAlpha;
        this.fieldImage = ImageLoader.loadImage(
                fieldImagePath,
                canvasWidthPixels,
                canvasHeightPixels,
                "Field Image",
                Color.LIGHTSLATEGRAY
        );

        this.robotTrailDots = Collections.synchronizedList(new ArrayList<>());

        this.canvas = new Canvas(canvasWidthPixels, canvasHeightPixels);
        this.gc = this.canvas.getGraphicsContext2D();
        this.getChildren().add(this.canvas);

        this.coordinateLabel = new Label("");
        this.coordinateLabel.setFont(Font.font("Arial", 12));
        this.coordinateLabel.setTextFill(Color.LIGHTGRAY);
        this.coordinateLabel.setStyle("-fx-background-color: rgba(0, 0, 0, 0.5); -fx-padding: 3px; -fx-border-radius: 3px; -fx-background-radius: 3px;");
        this.coordinateLabel.setVisible(false);
        this.coordinateLabel.setMouseTransparent(true);
        this.getChildren().add(this.coordinateLabel);

        canvas.setFocusTraversable(true);
        setupMouseHandlers();
    }

    private void setupMouseHandlers() {
        this.canvas.addEventHandler(MouseEvent.MOUSE_CLICKED, event -> {
            if (!canvas.isFocused()) {
                canvas.requestFocus();
                // System.out.println("FieldDisplay requested focus."); // Optional log
            }
            if (isPathCreationMode) {
                if (event.getButton() == MouseButton.PRIMARY) {
                    if (event.getClickCount() == 2) {
                        if (onPathFinishListener != null) onPathFinishListener.run();
                    } else {
                        if (onFieldPointClickListener != null)
                            onFieldPointClickListener.accept(new Point2D(event.getX(), event.getY()));
                    }
                }
            }
        });

        this.canvas.addEventHandler(MouseEvent.MOUSE_MOVED, event -> {
            Point2D fieldCoords = pixelToInches(event.getX(), event.getY()); // Returns (FieldX, FieldY)
            coordinateLabel.setText(String.format("Field X: %.1f, Field Y: %.1f", fieldCoords.getX(), fieldCoords.getY()));
            double labelOffsetX = 15;
            double labelOffsetY = 15;
            double newX = Math.min(event.getX() + labelOffsetX, canvas.getWidth() - coordinateLabel.prefWidth(-1) - 5);
            newX = Math.max(5, newX);
            double newY = Math.min(event.getY() + labelOffsetY, canvas.getHeight() - coordinateLabel.prefHeight(-1) - 5);
            newY = Math.max(5, newY);
            coordinateLabel.setLayoutX(newX);
            coordinateLabel.setLayoutY(newY);
            coordinateLabel.setVisible(true);
        });

        this.canvas.addEventHandler(MouseEvent.MOUSE_EXITED, event -> {
            if (isPathCreationMode) coordinateLabel.setVisible(false);
        });
    }

    public void addDebugCircle(double fieldX_inches, double fieldY_inches, double radiusInches, double headingDegrees, Color color) {
        synchronized (debugCircleLock) {
            currentDebugCircle = new DebugCircle(fieldX_inches, fieldY_inches, radiusInches, headingDegrees, color);
        }
    }

    public void setNamedLinesMap(Map<String, UdpPositionListener.LineData> linesMap) {
        synchronized(namedLinesMapLock) {
            this.namedLinesMap = linesMap;
        }
    }

    public void clearDebugCircle() {
        synchronized (debugCircleLock) { currentDebugCircle = null; }
    }

    public void setRobotTextMessage(String text) {
        synchronized (robotTextLock) { this.currentRobotTextMessage = text; }
    }

    public void clearRobotTextMessage() {
        synchronized (robotTextLock) { currentRobotTextMessage = null; }
    }

    /**
     * Converts a field X-coordinate (Vertical on field, +X UP, origin at field center)
     * to a canvas pixel Y-coordinate (Vertical on canvas, +Y DOWN, origin top-left).
     */
    private double fieldXtoCanvasY(double fieldXInches) {
        double canvasCenterY = canvas.getHeight() / 2.0;
        return canvasCenterY - (fieldXInches * this.scaleForFieldX_Vertical);
    }

    /**
     * Converts a field Y-coordinate (Horizontal on field, +Y LEFT, origin at field center)
     * to a canvas pixel X-coordinate (Horizontal on canvas, +X RIGHT, origin top-left).
     */
    private double fieldYtoCanvasX(double fieldYInches) {
        double canvasCenterX = canvas.getWidth() / 2.0;
        return canvasCenterX - (fieldYInches * this.scaleForFieldY_Horizontal);
    }

    /**
     * Converts a canvas pixel Y-coordinate (Vertical on canvas, +Y DOWN, origin top-left)
     * to a field X-coordinate (Vertical on field, +X UP, origin at field center).
     */
    public double canvasYtoFieldX(double canvasPixelY) {
        double canvasCenterY = canvas.getHeight() / 2.0;
        return -(canvasPixelY - canvasCenterY) / this.scaleForFieldX_Vertical;
    }

    /**
     * Converts a canvas pixel X-coordinate (Horizontal on canvas, +X RIGHT, origin top-left)
     * to a field Y-coordinate (Horizontal on field, +Y LEFT, origin at field center).
     */
    public double canvasXtoFieldY(double canvasPixelX) {
        double canvasCenterX = canvas.getWidth() / 2.0;
        return -(canvasPixelX - canvasCenterX) / this.scaleForFieldY_Horizontal;
    }

    /**
     * Converts canvas pixel coordinates (top-left origin) to NEW field coordinates (center origin).
     * @param canvasPixelX Horizontal pixel coordinate on canvas.
     * @param canvasPixelY Vertical pixel coordinate on canvas.
     * @return Point2D containing (FieldX, FieldY) in inches. FieldX is vertical, FieldY is horizontal.
     */
    public Point2D pixelToInches(double canvasPixelX, double canvasPixelY) {
        double fieldY_inches_horizontal = canvasXtoFieldY(canvasPixelX);
        double fieldX_inches_vertical = canvasYtoFieldX(canvasPixelY);
        return new Point2D(fieldX_inches_vertical, fieldY_inches_horizontal); // Return as (FieldX, FieldY)
    }

    private void drawFieldAxes(GraphicsContext gc) {
        gc.save(); // Save current state
        gc.setStroke(AXIS_COLOR);
        gc.setFill(AXIS_COLOR);
        gc.setLineWidth(AXIS_LINE_WIDTH);

        double canvasCenterX = canvas.getWidth() / 2.0;
        double canvasCenterY = canvas.getHeight() / 2.0;

        // --- Draw Field X-axis visual (Vertical line on canvas, representing FieldY = 0) ---
        // This line runs along where Field Y is zero.
        double fieldXAxis_canvasX = fieldYtoCanvasX(0); // Field Y = 0
        gc.strokeLine(fieldXAxis_canvasX, 0, fieldXAxis_canvasX, canvas.getHeight());

        // Label for Field X-axis (near positive end - TOP of screen)
        gc.setFont(AXIS_LABEL_FONT);
        gc.setTextAlign(TextAlignment.CENTER);
        gc.setTextBaseline(VPos.BOTTOM); // Align text bottom to be just above the axis start
        gc.fillText("X", fieldXAxis_canvasX + LABEL_OFFSET_PIXELS, LABEL_OFFSET_PIXELS * 2);


        // --- Draw Field Y-axis visual (Horizontal line on canvas, representing FieldX = 0) ---
        // This line runs along where Field X is zero.
        double fieldYAxis_canvasY = fieldXtoCanvasY(0); // Field X = 0
        gc.strokeLine(0, fieldYAxis_canvasY, canvas.getWidth(), fieldYAxis_canvasY);

        // Label for Field Y-axis (near positive end - LEFT of screen)
        gc.setFont(AXIS_LABEL_FONT);
        gc.setTextAlign(TextAlignment.RIGHT); // Align text right to be just left of axis start
        gc.setTextBaseline(VPos.CENTER);
        gc.fillText("Y", LABEL_OFFSET_PIXELS * 2, fieldYAxis_canvasY - LABEL_OFFSET_PIXELS);

        // --- Draw Ticks and Numbers ---
        gc.setFont(AXIS_NUMBER_FONT);
        int tickIntervalInches = 12; // Draw a tick every 12 inches (1 foot)

        // Ticks and numbers for Field X-axis (along the vertical line where FieldY=0)
        gc.setTextAlign(TextAlignment.LEFT);
        gc.setTextBaseline(VPos.CENTER);
        for (double fieldX_val = -fieldHeightInches_VerticalX / 2.0; fieldX_val <= fieldHeightInches_VerticalX / 2.0; fieldX_val += tickIntervalInches) {
            if (Math.abs(fieldX_val) < 0.1 && tickIntervalInches > 1) continue; // Skip 0 if ticks are dense
            double tickCanvasY = fieldXtoCanvasY(fieldX_val);
            // Draw tick mark
            gc.strokeLine(
                    fieldXAxis_canvasX - TICK_LENGTH_PIXELS, tickCanvasY,
                    fieldXAxis_canvasX + TICK_LENGTH_PIXELS, tickCanvasY
            );
            // Draw number (offset to the right of the Field X-axis line)
            gc.fillText(String.format("%.0f", fieldX_val),
                    fieldXAxis_canvasX + TICK_LENGTH_PIXELS + NUMBER_OFFSET_PIXELS,
                    tickCanvasY);
        }

        // Ticks and numbers for Field Y-axis (along the horizontal line where FieldX=0)
        gc.setTextAlign(TextAlignment.CENTER);
        gc.setTextBaseline(VPos.TOP); // Numbers below the ticks
        for (double fieldY_val = -fieldWidthInches_HorizontalY / 2.0; fieldY_val <= fieldWidthInches_HorizontalY / 2.0; fieldY_val += tickIntervalInches) {
            if (Math.abs(fieldY_val) < 0.1 && tickIntervalInches > 1) continue; // Skip 0
            double tickCanvasX = fieldYtoCanvasX(fieldY_val);
            // Draw tick mark
            gc.strokeLine(
                    tickCanvasX, fieldYAxis_canvasY - TICK_LENGTH_PIXELS,
                    tickCanvasX, fieldYAxis_canvasY + TICK_LENGTH_PIXELS
            );
            // Draw number (offset below the Field Y-axis line)
            gc.fillText(String.format("%.0f", fieldY_val),
                    tickCanvasX,
                    fieldYAxis_canvasY + TICK_LENGTH_PIXELS + NUMBER_OFFSET_PIXELS);
        }
        gc.restore(); // Restore previous state
    }

    public void setPathToDraw(List<CurvePoint> path) { // Path points are (FieldX, FieldY)
        this.currentPathToDraw = (path != null) ? new ArrayList<>(path) : new ArrayList<>();
    }

    public void setPathCreationMode(boolean isActive,
                                    Consumer<Point2D> pointClickListener, // Expects canvas pixel coords
                                    Runnable pathFinishListener) {
        this.isPathCreationMode = isActive;
        this.onFieldPointClickListener = pointClickListener;
        this.onPathFinishListener = pathFinishListener;
        canvas.setCursor(isActive ? Cursor.CROSSHAIR : Cursor.DEFAULT);
        if (!isActive) {
            coordinateLabel.setVisible(false);
        }
    }

    // robotFieldX is the new vertical field coord, robotFieldY is the new horizontal field coord
    public void addTrailDot(double robotFieldX, double robotFieldY) {
        if (robotTrailDots.size() >= MAX_TRAIL_DOTS) {
            if (!robotTrailDots.isEmpty()) robotTrailDots.remove(0);
        }
        // Convert to canvas pixel coordinates for storage
        robotTrailDots.add(new Point2D(fieldYtoCanvasX(robotFieldY), fieldXtoCanvasY(robotFieldX)));
    }

    public void clearTrail() { robotTrailDots.clear(); }
    public void clearTrailDots() { robotTrailDots.clear(); } // Alias
    public void setPath(List<CurvePoint> path) { this.currentPath = (path != null) ? new ArrayList<>(path) : new ArrayList<>();}

    // In FieldDisplay.java, REPLACE the entire method with this one.

    public void drawCurrentState() {
        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        gc.setFill(Color.rgb(50, 50, 50, this.backgroundAlpha));
        gc.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());

        // Draw the field image
        if (fieldImage != null) {
            gc.setGlobalAlpha(this.fieldImageAlpha);
            gc.drawImage(fieldImage, 0, 0, canvas.getWidth(), canvas.getHeight());
            gc.setGlobalAlpha(1.0); // Reset alpha
        }

        // Draw axes
        drawFieldAxes(gc);

        // Draw the path being created/displayed
        if (currentPathToDraw != null && !currentPathToDraw.isEmpty()) {
            gc.setStroke(isPathCreationMode ? Color.CYAN : Color.MAGENTA);
            gc.setLineWidth(2);
            for (int i = 0; i < currentPathToDraw.size() - 1; i++) {
                CurvePoint p1 = currentPathToDraw.get(i);
                CurvePoint p2 = currentPathToDraw.get(i + 1);
                gc.strokeLine(fieldYtoCanvasX(p1.y), fieldXtoCanvasY(p1.x), fieldYtoCanvasX(p2.y), fieldXtoCanvasY(p2.x));
            }
            gc.setFill(Color.RED);
            for (CurvePoint p : currentPathToDraw) {
                double canvasX = fieldYtoCanvasX(p.y);
                double canvasY = fieldXtoCanvasY(p.x);
                gc.fillOval(canvasX - 4, canvasY - 4, 8, 8);
            }
        }

        // Draw Named, Styled Lines
        final Color styleThickColor = Color.CYAN.deriveColor(0, 1, 1, 0.9);
        final Color styleThinColor = Color.LIMEGREEN.deriveColor(0, 1, 1, 0.8);
        final Color styleDottedColor = Color.LIGHTPINK.deriveColor(0, 1, 1, 0.85);

        // To prevent concurrent modification errors, copy the values to a temporary list before drawing.
        List<UdpPositionListener.LineData> linesToRender;
        synchronized (namedLinesMapLock) {
            linesToRender = new ArrayList<>(namedLinesMap.values());
        }

        if (linesToRender != null) {
            for (UdpPositionListener.LineData line : linesToRender) {
                if (line == null) continue;

                // Convert the integer style code from the data packet into your LineStyle enum
                LineStyle style = LineStyle.fromCode(line.styleCode);

                // Use the correct field names: x1, y1, x2, y2
                double canvasX1 = fieldYtoCanvasX(line.y1);
                double canvasY1 = fieldXtoCanvasY(line.x1);
                double canvasX2 = fieldYtoCanvasX(line.y2);
                double canvasY2 = fieldXtoCanvasY(line.x2);

                // Apply styles based on the enum
                switch (style) {
                    case SOLID_THICK:
                        gc.setStroke(styleThickColor);
                        gc.setLineWidth(3.0);
                        gc.setLineDashes(0);
                        gc.strokeLine(canvasX1, canvasY1, canvasX2, canvasY2);
                        break;
                    case SOLID_THIN:
                        gc.setStroke(styleThinColor);
                        gc.setLineWidth(3.5);
                        gc.setLineDashes(0);
                        gc.strokeLine(canvasX1, canvasY1, canvasX2, canvasY2);
                        break;
                    case DOTTED:
                        gc.setStroke(styleDottedColor);
                        gc.setLineWidth(1.5);
                        gc.setLineDashes(5, 5);
                        gc.strokeLine(canvasX1, canvasY1, canvasX2, canvasY2);
                        gc.setLineDashes(0); // Reset dashes for other drawing
                        break;
                }
            }
        }

//        // Draw the robot
//        if (robot != null) {
//            double robotCanvasX = fieldYtoCanvasX(robot.getYInches());
//            double robotCanvasY = fieldXtoCanvasY(robot.getXInches());
//            double robotCanvasAngle = robot.getHeadingDegrees() - 90;
//            drawRotatedImage(gc, robot.getImage(), robotCanvasAngle, robotCanvasX, robotCanvasY);
//        }

        // Draw Robot Image
        double robotCanvasPixelX = 0; // robot's center X on canvas (horizontal)
        double robotCanvasPixelY = 0; // robot's center Y on canvas (vertical)

        if (robot != null) {
            Image robotImg = robot.getRobotImage();
            double robotFieldX_vertical = robot.getXInches();    // Robot's Field X (vertical)
            double robotFieldY_horizontal = robot.getYInches();  // Robot's Field Y (horizontal)

            robotCanvasPixelX = fieldYtoCanvasX(robotFieldY_horizontal);
            robotCanvasPixelY = fieldXtoCanvasY(robotFieldX_vertical);

            // Robot.ROBOT_WIDTH_INCHES: robot's extent along its own local Y-axis (which aligns with Field Y when heading=0)
            // Robot.ROBOT_HEIGHT_INCHES: robot's extent along its own local X-axis (which aligns with Field X when heading=0)
            // These become the display width/height on canvas *before* rotation.
            double robotDisplayWidthOnCanvas = Robot.ROBOT_WIDTH_INCHES * scaleForFieldY_Horizontal;
            double robotDisplayHeightOnCanvas = Robot.ROBOT_HEIGHT_INCHES * scaleForFieldX_Vertical;

//            double robotHeadingDegrees = robot.getHeadingDegrees(); // 0 degrees = points +FieldX (down screen)
            double robotHeadingDegrees_CCW = robot.getHeadingDegrees(); // This is your CCW angle (0 deg = +FieldX/Down)

            gc.save();
            // JavaFX Rotate: positive angle is Clockwise. Pivot is robot's canvas center.
            Rotate rotateTransform = new Rotate(-robotHeadingDegrees_CCW, robotCanvasPixelX, robotCanvasPixelY);
            gc.setTransform(
                    rotateTransform.getMxx(), rotateTransform.getMyx(),
                    rotateTransform.getMxy(), rotateTransform.getMyy(),
                    rotateTransform.getTx(), rotateTransform.getTy()
            );
            gc.drawImage(
                    robotImg,
                    robotCanvasPixelX - robotDisplayWidthOnCanvas / 2.0,
                    robotCanvasPixelY - robotDisplayHeightOnCanvas / 2.0,
                    robotDisplayWidthOnCanvas,
                    robotDisplayHeightOnCanvas
            );
            gc.restore();
        }

        // Draw robot trail
        gc.setFill(Color.YELLOW);
        synchronized (robotTrailDots) {
            for (Point2D dot : robotTrailDots) {
                gc.fillOval(dot.getX() - TRAIL_DOT_RADIUS_PIXELS / 2, dot.getY() - TRAIL_DOT_RADIUS_PIXELS / 2, TRAIL_DOT_RADIUS_PIXELS, TRAIL_DOT_RADIUS_PIXELS);
            }
        }

        // Draw robot text message
        synchronized (robotTextLock) {
            if (currentRobotTextMessage != null && !currentRobotTextMessage.isEmpty() && robot != null) {
                gc.save();
                double robotCanvasX = fieldYtoCanvasX(robot.getYInches());
                double robotCanvasY = fieldXtoCanvasY(robot.getXInches());
                gc.setFont(ROBOT_TEXT_FONT);
                gc.setTextAlign(TextAlignment.CENTER);
                gc.setTextBaseline(VPos.TOP);
                double textWidth = new javafx.scene.text.Text(currentRobotTextMessage).getLayoutBounds().getWidth();
                double textHeight = new javafx.scene.text.Text(currentRobotTextMessage).getLayoutBounds().getHeight();
                gc.setFill(ROBOT_TEXT_BACKGROUND_COLOR);
                gc.fillRoundRect(
                        robotCanvasX - (textWidth / 2) - ROBOT_TEXT_PADDING,
                        robotCanvasY + ROBOT_TEXT_Y_CANVAS_OFFSET_PIXELS - ROBOT_TEXT_PADDING,
                        textWidth + (ROBOT_TEXT_PADDING * 2),
                        textHeight + (ROBOT_TEXT_PADDING * 2),
                        5, 5
                );
                gc.setFill(ROBOT_TEXT_COLOR);
                gc.fillText(currentRobotTextMessage, robotCanvasX, robotCanvasY + ROBOT_TEXT_Y_CANVAS_OFFSET_PIXELS);
                gc.restore();
            }
        }

        // --- Draw Debug Circle as an Outline with a Heading Line ---
        synchronized (debugCircleLock) {
            if (currentDebugCircle != null) {
                gc.save();
                double canvasX = fieldYtoCanvasX(currentDebugCircle.fieldY);
                double canvasY = fieldXtoCanvasY(currentDebugCircle.fieldX);

                double radiusInches = currentDebugCircle.radiusInches;
                double radiusPixels = radiusInches * scaleForFieldY_Horizontal;

//                double canvasX = fieldYtoCanvasX(currentDebugCircle.fieldY);
//                double canvasY = fieldXtoCanvasY(currentDebugCircle.fieldX);
//                double radiusPixels = currentDebugCircle.radiusInches * scaleForFieldY_Horizontal;

                // 1. Set the STROKE color and line width for the circle
                Color cColor = currentDebugCircle.color != null ? currentDebugCircle.color : Color.ORANGE;
                gc.setStroke(cColor);
                gc.setLineWidth(2.0); // A visible line width

                // 2. Use strokeOval to draw the outline (circumference)
                gc.strokeOval(canvasX - radiusPixels, canvasY - radiusPixels, radiusPixels * 2, radiusPixels * 2);

                // 3. Draw the heading line from the center to the edge
                // It will automatically use the same stroke color and width set above
                double headingRad_Canvas = Math.toRadians(90 - currentDebugCircle.headingDegrees);
                gc.strokeLine(
                        canvasX, // from center X
                        canvasY, // from center Y
                        canvasX + radiusPixels * Math.cos(headingRad_Canvas), // to edge X
                        canvasY - radiusPixels * Math.sin(headingRad_Canvas)  // to edge Y
                );

                gc.restore();
            }
        }
//        // Draw debug circle
//        synchronized (debugCircleLock) {
//            if (currentDebugCircle != null) {
//                gc.save();
//                double canvasX = fieldYtoCanvasX(currentDebugCircle.fieldY);
//                double canvasY = fieldXtoCanvasY(currentDebugCircle.fieldX);
//                double radiusPixels = currentDebugCircle.radiusInches * scaleForFieldY_Horizontal;
//                gc.setFill(currentDebugCircle.color);
//                gc.fillOval(canvasX - radiusPixels, canvasY - radiusPixels, radiusPixels * 2, radiusPixels * 2);
//                gc.setStroke(Color.WHITE);
//                gc.setLineWidth(2.0);
//                double headingRad_Canvas = Math.toRadians(90 - currentDebugCircle.headingDegrees);
//                gc.strokeLine(
//                        canvasX,
//                        canvasY,
//                        canvasX + radiusPixels * Math.cos(headingRad_Canvas),
//                        canvasY - radiusPixels * Math.sin(headingRad_Canvas)
//                );
//                gc.restore();
//            }
//        }
    }


//    public void drawCurrentState() {
//        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
//        gc.setFill(Color.rgb(50, 50, 50, this.backgroundAlpha));
//        gc.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());
//
//        if (fieldImage != null) {
//            double originalAlpha = gc.getGlobalAlpha();
//            gc.setGlobalAlpha(this.fieldImageAlpha);
//            gc.drawImage(fieldImage, 0, 0, canvas.getWidth(), canvas.getHeight());
//            gc.setGlobalAlpha(originalAlpha);
//        }
//
//        // Draw axes
//        drawFieldAxes(gc);
//
//        // Draw the CURRENT PATH being created or displayed
//        // currentPathToDraw points are (FieldX, FieldY)
//        if (currentPathToDraw != null && !currentPathToDraw.isEmpty()) {
//            gc.setStroke(Color.ORANGERED);
//            gc.setLineWidth(3.0);
//            gc.setLineDashes(isPathCreationMode ? 5 : 0);
//            gc.beginPath();
//            CurvePoint firstP = currentPathToDraw.get(0);
//            // firstP.x is FieldX (vertical), firstP.y is FieldY (horizontal)
//            gc.moveTo(fieldYtoCanvasX(firstP.y), fieldXtoCanvasY(firstP.x));
//            for (int i = 1; i < currentPathToDraw.size(); i++) {
//                CurvePoint p = currentPathToDraw.get(i);
//                gc.lineTo(fieldYtoCanvasX(p.y), fieldXtoCanvasY(p.x));
//            }
//            gc.stroke();
//            gc.setLineDashes(0);
//            gc.setFill(Color.RED);
//            for (CurvePoint p : currentPathToDraw) {
//                gc.fillOval(fieldYtoCanvasX(p.y) - 4, fieldXtoCanvasY(p.x) - 4, 8, 8);
//            }
//        }
//
//        // Draw Robot Image
//        double robotCanvasPixelX = 0; // robot's center X on canvas (horizontal)
//        double robotCanvasPixelY = 0; // robot's center Y on canvas (vertical)
//
//        if (robot != null) {
//            Image robotImg = robot.getRobotImage();
//            double robotFieldX_vertical = robot.getXInches();    // Robot's Field X (vertical)
//            double robotFieldY_horizontal = robot.getYInches();  // Robot's Field Y (horizontal)
//
//            robotCanvasPixelX = fieldYtoCanvasX(robotFieldY_horizontal);
//            robotCanvasPixelY = fieldXtoCanvasY(robotFieldX_vertical);
//
//            // Robot.ROBOT_WIDTH_INCHES: robot's extent along its own local Y-axis (which aligns with Field Y when heading=0)
//            // Robot.ROBOT_HEIGHT_INCHES: robot's extent along its own local X-axis (which aligns with Field X when heading=0)
//            // These become the display width/height on canvas *before* rotation.
//            double robotDisplayWidthOnCanvas = Robot.ROBOT_WIDTH_INCHES * scaleForFieldY_Horizontal;
//            double robotDisplayHeightOnCanvas = Robot.ROBOT_HEIGHT_INCHES * scaleForFieldX_Vertical;
//
////            double robotHeadingDegrees = robot.getHeadingDegrees(); // 0 degrees = points +FieldX (down screen)
//            double robotHeadingDegrees_CCW = robot.getHeadingDegrees(); // This is your CCW angle (0 deg = +FieldX/Down)
//
//            gc.save();
//            // JavaFX Rotate: positive angle is Clockwise. Pivot is robot's canvas center.
//            Rotate rotateTransform = new Rotate(-robotHeadingDegrees_CCW, robotCanvasPixelX, robotCanvasPixelY);
//            gc.setTransform(
//                    rotateTransform.getMxx(), rotateTransform.getMyx(),
//                    rotateTransform.getMxy(), rotateTransform.getMyy(),
//                    rotateTransform.getTx(), rotateTransform.getTy()
//            );
//            gc.drawImage(
//                    robotImg,
//                    robotCanvasPixelX - robotDisplayWidthOnCanvas / 2.0,
//                    robotCanvasPixelY - robotDisplayHeightOnCanvas / 2.0,
//                    robotDisplayWidthOnCanvas,
//                    robotDisplayHeightOnCanvas
//            );
//            gc.restore();
//        }
//
//        // Draw Robot Trail Dots (already in canvas pixel coordinates)
//        gc.setFill(Color.YELLOW);
//        synchronized (robotTrailDots) {
//            for (Point2D dot : robotTrailDots) {
//                gc.fillOval(
//                        dot.getX() - TRAIL_DOT_RADIUS_PIXELS,
//                        dot.getY() - TRAIL_DOT_RADIUS_PIXELS,
//                        TRAIL_DOT_RADIUS_PIXELS * 2,
//                        TRAIL_DOT_RADIUS_PIXELS * 2
//                );
//            }
//        }
//
//        // Draw Named, Styled Lines
//        final Color style1Color = Color.CYAN.deriveColor(0, 1, 1, 0.9);
//        final Color style2Color = Color.LIMEGREEN.deriveColor(0, 1, 1, 0.8); // Differentiated color
//        final Color style3ColorDotted = Color.LIGHTPINK.deriveColor(0, 1, 1, 0.85);
//
//        List<UdpPositionListener.LineData> linesToRender;
//        synchronized (namedLinesMapLock) {
//            linesToRender = new ArrayList<>(namedLinesMap.values());
//        }
//
//        if (linesToRender != null) {
//            for (UdpPositionListener.LineData lineData : linesToRender) {
//                if (lineData == null) continue;
//
//                // lineData.x0 is FieldX0 (vertical), lineData.y0 is FieldY0 (horizontal)
//                double canvasX0 = fieldYtoCanvasX(lineData.y0);
//                double canvasY0 = fieldXtoCanvasY(lineData.x0);
//                double canvasX1 = fieldYtoCanvasX(lineData.y1);
//                double canvasY1 = fieldXtoCanvasY(lineData.x1);
//
//                switch (lineData.style) {
//                    case SOLID_THICK:
//                        gc.setStroke(style1Color); gc.setLineWidth(3.0); gc.setLineDashes(0);
//                        gc.strokeLine(canvasX0, canvasY0, canvasX1, canvasY1);
//                        break;
//                    case SOLID_THIN:
//                        gc.setStroke(style2Color); gc.setLineWidth(1.5); gc.setLineDashes(0);
//                        gc.strokeLine(canvasX0, canvasY0, canvasX1, canvasY1);
//                        break;
//                    case DOTTED:
//                        gc.setStroke(style3ColorDotted); gc.setLineWidth(1.5);
//                        gc.setLineDashes(5, 5);
//                        gc.strokeLine(canvasX0, canvasY0, canvasX1, canvasY1);
//                        gc.setLineDashes(0);
//                        break;
//                    default:
//                        gc.setStroke(Color.WHITE); gc.setLineWidth(1.0); gc.setLineDashes(0);
//                        gc.strokeLine(canvasX0, canvasY0, canvasX1, canvasY1);
//                        break;
//                }
//            }
//        }
//
//        // Draw Robot Text Message
//        String textToDraw;
//        synchronized (robotTextLock) {
//            textToDraw = currentRobotTextMessage;
//        }
//        if (textToDraw != null && !textToDraw.isEmpty() && robot != null) {
//            // robotCanvasPixelX and robotCanvasPixelY are the robot's center on canvas
//            // If robot object might be null here but text still needs drawing, this needs care.
//            // But usually text is related to a non-null robot.
//            if(robotCanvasPixelX == 0 && robotCanvasPixelY == 0 && robot!=null){ // Recalculate if not set by robot draw
//                robotCanvasPixelX = fieldYtoCanvasX(robot.getYInches());
//                robotCanvasPixelY = fieldXtoCanvasY(robot.getXInches());
//            }
//
//
//            gc.setFont(ROBOT_TEXT_FONT);
//            gc.setTextAlign(TextAlignment.CENTER);
//            double textWidthEstimate = textToDraw.length() * ROBOT_TEXT_FONT.getSize() * 0.6;
//            double textHeight = ROBOT_TEXT_FONT.getSize();
//            double backgroundWidth = textWidthEstimate + 2 * ROBOT_TEXT_PADDING;
//            double backgroundHeight = textHeight + 2 * ROBOT_TEXT_PADDING;
//
//            // Position text relative to robot's canvas center
//            double textBackgroundCanvasX = robotCanvasPixelX - backgroundWidth / 2;
//            double textBackgroundCanvasY = robotCanvasPixelY + ROBOT_TEXT_Y_CANVAS_OFFSET_PIXELS; // Vertical offset on canvas
//
//            gc.setFill(ROBOT_TEXT_BACKGROUND_COLOR);
//            gc.fillRoundRect(textBackgroundCanvasX, textBackgroundCanvasY, backgroundWidth, backgroundHeight, 6, 6);
//            gc.setFill(ROBOT_TEXT_COLOR);
//            gc.fillText(textToDraw, robotCanvasPixelX, textBackgroundCanvasY + ROBOT_TEXT_PADDING + textHeight * 0.8);
//        }
//
//        // Draw Debug Circle
//        DebugCircle circleToDraw;
//        synchronized (debugCircleLock) {
//            circleToDraw = this.currentDebugCircle;
//        }
//        if (circleToDraw != null) {
//            // circleToDraw.fieldX is FieldX (vertical), circleToDraw.fieldY is FieldY (horizontal)
//            double canvasCircleCenterX = fieldYtoCanvasX(circleToDraw.fieldY);
//            double canvasCircleCenterY = fieldXtoCanvasY(circleToDraw.fieldX);
//
//            // For radius, we need to decide how it scales.
//            // If radiusInches is a general distance, averaging the scales might look most "circular".
//            double pixelRadius = circleToDraw.radiusInches * ((scaleForFieldX_Vertical + scaleForFieldY_Horizontal) / 2.0);
//
//            Color cColor = circleToDraw.color != null ? circleToDraw.color : Color.MAGENTA;
//            gc.setStroke(cColor);
//            gc.setLineWidth(3);
//            gc.strokeOval(
//                    canvasCircleCenterX - pixelRadius,
//                    canvasCircleCenterY - pixelRadius,
//                    pixelRadius * 2,
//                    pixelRadius * 2
//            );
//
//            double headingRad_CCW = Math.toRadians(circleToDraw.headingDegrees);
//
//            // Calculate end point in Field Coordinates
//            double endFieldX = circleToDraw.fieldX + circleToDraw.radiusInches * Math.cos(headingRad_CCW);
//            double endFieldY = circleToDraw.fieldY + circleToDraw.radiusInches * Math.sin(headingRad_CCW);
//
//            gc.beginPath();
//            gc.moveTo(canvasCircleCenterX, canvasCircleCenterY);
//            gc.lineTo(fieldYtoCanvasX(endFieldY), fieldXtoCanvasY(endFieldX)); // Convert end point to canvas pixels
//            gc.stroke();
//        }
//    }
}
