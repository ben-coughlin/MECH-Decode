// In PlotDisplay.java
package com.example.ftcfieldsimulator;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollBar;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.shape.StrokeLineCap;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.text.TextAlignment;
import javafx.geometry.Orientation;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PlotDisplay extends Pane {

    // --- Configuration Constants ---
    public static final double DEFAULT_PLOT_AREA_WIDTH_PIXELS = 1000;
    public static final double DEFAULT_PLOT_AREA_HEIGHT_PIXELS = DEFAULT_PLOT_AREA_WIDTH_PIXELS * (9.0 / 16.0);

    private static final double PADDING_TOP = 30;
    private static final double PADDING_BOTTOM = 50;
    private static final double PADDING_LEFT_FOR_Y_AXIS = 60;
    private static final double PADDING_RIGHT_FOR_Y_AXIS_2 = 60;
    private static final double PADDING_RIGHT_GRAPH = 20;
    private static final double SCROLLBAR_HEIGHT = 20;
    private static final double READOUT_TEXT_X_OFFSET = 7;
    private static final double READOUT_TEXT_Y_SPACING = 12;

    private static final Color MARKER_LINE_COLOR = Color.rgb(100, 100, 100);
    private static final Color MARKER_TEXT_COLOR = Color.BLACK;

    private final Map<Integer, Label> styleToReadoutLabel = new HashMap<>();
    private final Map<Integer, Label> styleToReadoutLabel2 = new HashMap<>();
    private static final String READOUT_LABEL_STYLE = "-fx-padding: 2px 4px; -fx-border-radius: 3px; -fx-background-radius: 3px;";
    private static final Font READOUT_LABEL_FONT = Font.font("Arial", 10);
    private Label cursorXLabel, cursorYLabel, cursorY2Label;

    private Canvas mainGraphCanvas;
    private GraphicsContext mainGc;
    private Canvas yAxisCanvas, yAxisCanvas2;
    private GraphicsContext yAxisGc, yAxisGc2;

    private ScrollBar hScrollBar;
    private StackPane graphContainer;

    private double visibleGraphWidth, visibleGraphHeight;

    // --- Data Storage & State ---
    private final List<PlotDataEvent> plotEvents = new ArrayList<>();
    private static final int MAX_PLOT_EVENTS = 100000;
    private static final long MAX_TIME_GAP_MS = 15000;

    // Y-Axis 1 (Left)
    private double currentMinY = 0.0, currentMaxY = 100.0;
    private String yAxisUnit = "Value";

    // Y-Axis 2 (Right)
    private double currentMinY2 = 0.0, currentMaxY2 = 1.0;
    private String yAxisUnit2 = "Value 2";

    private long firstTimestamp = -1, lastTimestamp = -1;
    private double pixelsPerMillisecond = 0.02;
    private static final double MIN_PIXELS_PER_MS = 0.0001, MAX_PIXELS_PER_MS = 1.0;
    private static final double X_AXIS_LABEL_AREA_HEIGHT_ON_MAIN_CANVAS = 30;
    private double currentScrollOffsetMs = 0;

    private final BooleanProperty autoScrollEnabled = new SimpleBooleanProperty(true);
    private PlotDisplayControlPanel controlPanelProxy;

    // --- Data Readout State ---
    private final DecimalFormat readoutValueFormat = new DecimalFormat("#0.0#");
    private final Map<Integer, String> styleToReadoutValueString = new HashMap<>();
    private final Map<Integer, Double> styleToReadoutScreenY = new HashMap<>();
    private final Map<Integer, Double> styleToReadoutDataY = new HashMap<>();
    private final Map<Integer, Color> styleToReadoutColor = new HashMap<>();
    private final Map<Integer, String> styleToReadoutValueString2 = new HashMap<>();
    private final Map<Integer, Double> styleToReadoutScreenY2 = new HashMap<>();
    private final Map<Integer, Double> styleToReadoutDataY2 = new HashMap<>();
    private final Map<Integer, Color> styleToReadoutColor2 = new HashMap<>();

    private record TimestampedStringValue(long ts, String v) implements Comparable<TimestampedStringValue> {
        @Override
        public int compareTo(TimestampedStringValue other) { return Long.compare(this.ts, other.ts); }
    }
    private final Map<String, List<TimestampedStringValue>> keyValueStore = new ConcurrentHashMap<>();

    // --- Mouse Cursor State ---
    private boolean isMouseInPlotArea = false;
    private double mousePlotX = -1, mousePlotY = -1;
    private long currentCursorTimeMs = -1;
    private static final Color CURSOR_LINE_COLOR = Color.rgb(255, 140, 0);

    // --- Line and Point Styles ---
    private static class LineStyle {
        final Color color; final double width; final double[] dashArray;
        LineStyle(Color c, double w, double[] d) { this.color = c; this.width = w; this.dashArray = d; }
    }
    private static final LineStyle[] LINE_STYLES = {
            new LineStyle(Color.RED, 2, null), new LineStyle(Color.BLUE, 2, null),
            new LineStyle(Color.GREEN, 2, null), new LineStyle(Color.ORANGE, 2, null),
            new LineStyle(Color.CYAN, 2, null), new LineStyle(Color.PURPLE, 2, new double[]{5,3}),
            new LineStyle(Color.LIMEGREEN, 2, new double[]{8,4}), new LineStyle(Color.HOTPINK, 2, new double[]{2,3}),
            new LineStyle(Color.TEAL, 2, new double[]{3,4}), new LineStyle(Color.BLACK, 1, null)
    };
    private final PlotPoint[] lastLinePointByStyle = new PlotPoint[LINE_STYLES.length];
    private final PlotPoint[] lastLinePointByStyle2 = new PlotPoint[LINE_STYLES.length];

    public PlotDisplay(double requestedVisibleWidth, double requestedVisibleHeight) {
        this.visibleGraphWidth = requestedVisibleWidth;
        this.visibleGraphHeight = requestedVisibleHeight;

        this.yAxisCanvas = new Canvas(PADDING_LEFT_FOR_Y_AXIS, PADDING_TOP + this.visibleGraphHeight + PADDING_BOTTOM);
        this.yAxisGc = yAxisCanvas.getGraphicsContext2D();

        this.yAxisCanvas2 = new Canvas(PADDING_RIGHT_FOR_Y_AXIS_2, PADDING_TOP + this.visibleGraphHeight + PADDING_BOTTOM);
        this.yAxisGc2 = yAxisCanvas2.getGraphicsContext2D();
        this.yAxisCanvas2.setVisible(true);

        this.mainGraphCanvas = new Canvas(this.visibleGraphWidth, this.visibleGraphHeight + X_AXIS_LABEL_AREA_HEIGHT_ON_MAIN_CANVAS);
        this.mainGc = mainGraphCanvas.getGraphicsContext2D();
        this.graphContainer = new StackPane(mainGraphCanvas);
        this.graphContainer.setPrefSize(this.visibleGraphWidth, this.visibleGraphHeight + X_AXIS_LABEL_AREA_HEIGHT_ON_MAIN_CANVAS);

        this.hScrollBar = new ScrollBar();
        this.hScrollBar.setOrientation(Orientation.HORIZONTAL);
        hScrollBar.valueProperty().addListener((obs, oldVal, newVal) -> {
            // *** SCROLLING FIX 1/2: Do NOT translate the canvas. Just record the value and redraw. ***
            currentScrollOffsetMs = (pixelsPerMillisecond > 0) ? newVal.doubleValue() / pixelsPerMillisecond : 0;
            redrawMainGraph();
        });
        hScrollBar.pressedProperty().addListener((obs, was, is) -> { if (is) setAutoScrollEnabled(false); });

        double totalPaneWidth = PADDING_LEFT_FOR_Y_AXIS + this.visibleGraphWidth + PADDING_RIGHT_FOR_Y_AXIS_2 + PADDING_RIGHT_GRAPH;
        double totalPaneHeight = PADDING_TOP + this.visibleGraphHeight + X_AXIS_LABEL_AREA_HEIGHT_ON_MAIN_CANVAS + SCROLLBAR_HEIGHT;
        setPrefSize(totalPaneWidth, totalPaneHeight);

        yAxisCanvas.setLayoutX(0);
        yAxisCanvas.setLayoutY(0);
        graphContainer.setLayoutX(PADDING_LEFT_FOR_Y_AXIS);
        graphContainer.setLayoutY(PADDING_TOP);
        yAxisCanvas2.setLayoutX(PADDING_LEFT_FOR_Y_AXIS + this.visibleGraphWidth);
        yAxisCanvas2.setLayoutY(0);
        hScrollBar.setLayoutX(PADDING_LEFT_FOR_Y_AXIS);
        hScrollBar.setLayoutY(PADDING_TOP + this.visibleGraphHeight + X_AXIS_LABEL_AREA_HEIGHT_ON_MAIN_CANVAS);
        hScrollBar.setPrefWidth(this.visibleGraphWidth);
        hScrollBar.setPrefHeight(SCROLLBAR_HEIGHT);

        getChildren().addAll(this.yAxisCanvas, this.yAxisCanvas2, this.graphContainer, this.hScrollBar);

        setupCursorAndLabels();
        redrawFullPlot();
    }

    private void setupCursorAndLabels() {
        mainGraphCanvas.setOnMouseMoved(this::handleMouseMovedOverPlot);
        mainGraphCanvas.setOnMouseExited(this::handleMouseExitedPlot);

        String style = "-fx-padding: 2px 4px; -fx-border-radius: 3px; -fx-background-radius: 3px; -fx-background-color: " + toRgbCode(CURSOR_LINE_COLOR) + ";";
        Font font = Font.font("Arial", 10);

        cursorXLabel = createCursorLabel(font, style);
        cursorYLabel = createCursorLabel(font, style);
        cursorY2Label = createCursorLabel(font, style);
        getChildren().addAll(cursorXLabel, cursorYLabel, cursorY2Label);
    }

    private Label createCursorLabel(Font font, String style) {
        Label label = new Label();
        label.setFont(font);
        label.setStyle(style);
        label.setTextFill(Color.WHITE);
        label.setMouseTransparent(true);
        label.setVisible(false);
        return label;
    }

    private String toRgbCode(Color c) { return String.format("rgb(%d,%d,%d)",(int)(c.getRed()*255),(int)(c.getGreen()*255),(int)(c.getBlue()*255)); }

    public void setAutoScrollEnabled(boolean enabled) {
        autoScrollEnabled.set(enabled);
        if (controlPanelProxy != null) controlPanelProxy.setFollowRealTimeSelected(enabled);
        if (enabled && lastTimestamp != -1) scrollToTimestamp(lastTimestamp);
    }

    private void handleMouseMovedOverPlot(MouseEvent event) {
        isMouseInPlotArea = true;
        mousePlotX = event.getX();
        mousePlotY = event.getY();
        redrawMainGraph();
    }

    private void handleMouseExitedPlot(MouseEvent event) {
        isMouseInPlotArea = false;
        redrawMainGraph();
    }

    public void addPlotEvent(PlotDataEvent event) {
        if (event == null) return;
        if (lastTimestamp != -1 && (event.getTimestamp() - lastTimestamp > MAX_TIME_GAP_MS)) {
            Platform.runLater(() -> { clearPlot(); processNewEvent(event); });
            return;
        }
        Platform.runLater(() -> processNewEvent(event));
    }

    private void processNewEvent(PlotDataEvent event) {
        boolean isFirst = (firstTimestamp == -1);
        if (isFirst) firstTimestamp = event.getTimestamp();
        if (lastTimestamp == -1 || event.getTimestamp() > lastTimestamp) lastTimestamp = event.getTimestamp();

        synchronized (plotEvents) { if (!(event instanceof PlotKeyValueEvent)) { if (plotEvents.size() >= MAX_PLOT_EVENTS) plotEvents.remove(0); plotEvents.add(event); } }

        if (event instanceof PlotKeyValueEvent kv) {
            keyValueStore.computeIfAbsent(kv.getKey(), k -> new ArrayList<>()).add(new TimestampedStringValue(kv.getTimestamp(), kv.getValue()));
            keyValueStore.get(kv.getKey()).sort(null);
        } else if (event instanceof PlotYLimitsEvent yle) setYLimits(yle.getMinY(), yle.getMaxY());
        else if (event instanceof PlotYUnitsEvent yue) { setYUnit(yue.getUnit()); return; }
        else if (event instanceof PlotYLimits2Event yle2) setYLimits2(yle2.getMinY(), yle2.getMaxY());
        else if (event instanceof PlotYUnits2Event yue2) { setYUnit2(yue2.getUnit()); return; }

        updateCanvasWidthAndScrollbar();
        if (autoScrollEnabled.get() && !(event instanceof PlotKeyValueEvent)) scrollToTimestamp(event.getTimestamp());

        if (event instanceof PlotYLimitsEvent || event instanceof PlotYLimits2Event) {
            redrawFullPlot();
        } else {
            redrawMainGraph();
        }
    }

    public void clearPlot() {
        synchronized (plotEvents) { plotEvents.clear(); }
        synchronized (keyValueStore) { keyValueStore.clear(); }
        firstTimestamp = -1; lastTimestamp = -1; currentScrollOffsetMs = 0;
        hScrollBar.setValue(0);
        updateCanvasWidthAndScrollbar();
        redrawFullPlot();
    }

    private void redrawFullPlot() {
        redrawYAxis();
        redrawYAxis2();
        redrawMainGraph();
    }

    private void redrawYAxis() { drawAxis(yAxisGc, yAxisCanvas, PADDING_LEFT_FOR_Y_AXIS - 1, currentMinY, currentMaxY, yAxisUnit, true); }
    private void redrawYAxis2() { drawAxis(yAxisGc2, yAxisCanvas2, 0, currentMinY2, currentMaxY2, yAxisUnit2, false); }

    private void drawAxis(GraphicsContext gc, Canvas canvas, double lineX, double minY, double maxY, String unit, boolean isLeft) {
        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        gc.setFill(Color.WHITE); gc.fillRect(0,0, canvas.getWidth(), canvas.getHeight());
        gc.setStroke(Color.LIGHTGRAY); gc.setLineWidth(0.5);
        for (int i=0; i<=10; i++) { double y = PADDING_TOP + (visibleGraphHeight/10)*i; gc.strokeLine(0, y, canvas.getWidth(), y); }

        gc.setStroke(Color.BLACK); gc.setLineWidth(1.0); gc.setFill(Color.BLACK); gc.setFont(Font.font("Arial", 10));
        gc.strokeLine(lineX, PADDING_TOP, lineX, PADDING_TOP + visibleGraphHeight);
        gc.setTextAlign(isLeft ? TextAlignment.RIGHT : TextAlignment.LEFT);
        double yRange = maxY - minY; if (yRange <= 1e-6) yRange = 1.0;

        for (int i=0; i<=20; i++) {
            double val = minY + (yRange/20)*i;
            double yPos = PADDING_TOP + visibleGraphHeight - ((val-minY)/yRange * visibleGraphHeight);
            if (yPos >= PADDING_TOP - 5 && yPos <= PADDING_TOP + visibleGraphHeight + 5) {
                double textX = isLeft ? lineX - 5 : lineX + 5;
                gc.fillText(formatNiceNumber(val, yRange), textX, yPos + 4);
                double tickX = isLeft ? lineX - 3 : lineX;
                gc.strokeLine(tickX, yPos, tickX + (isLeft ? 3 : -3), yPos);
            }
        }

        gc.save(); gc.setTextAlign(TextAlignment.CENTER);
        gc.translate(isLeft ? 15 : canvas.getWidth() - 15, PADDING_TOP + visibleGraphHeight/2);
        gc.rotate(-90); gc.fillText(unit, 0, 0); gc.restore();
    }

    private void redrawMainGraph() {
        mainGc.save();
        try {
            mainGc.clearRect(0, 0, mainGraphCanvas.getWidth(), mainGraphCanvas.getHeight());

            // *** SCROLLING FIX 2/2: Translate and then Clip the Graphics Context ***
            mainGc.translate(-hScrollBar.getValue(), 0);
            mainGc.beginPath(); // Start a new path for the clipping region
            mainGc.rect(hScrollBar.getValue(), 0, visibleGraphWidth, mainGraphCanvas.getHeight());
            mainGc.clip();

            // --- All drawing from this point on is clipped and translated ---

            // Draw horizontal grid lines
            mainGc.setStroke(Color.LIGHTGRAY);
            mainGc.setLineWidth(0.5);
            for (int i = 0; i <= 10; i++) {
                double y = (visibleGraphHeight / 10) * i;
                mainGc.strokeLine(hScrollBar.getValue(), y, hScrollBar.getValue() + visibleGraphWidth, y);
            }

            // Draw X-axis decorations (which includes vertical grid lines)
            if (firstTimestamp != -1) {
                drawXAxisDecorations();
            }

            // Draw the main X-axis line last so it's on top of grid lines
            mainGc.setStroke(Color.BLACK);
            mainGc.setLineWidth(1.0);
            mainGc.strokeLine(0, visibleGraphHeight - 1, mainGraphCanvas.getWidth(), visibleGraphHeight - 1);

            drawData();
            drawCursor();
        } finally {
            mainGc.restore(); // This undoes both the translation and the clip
        }
    }

    private void drawXAxisDecorations() {
        double step = 500; if(pixelsPerMillisecond*step<40) step=1000; if(pixelsPerMillisecond*step<40) step=2000; if(pixelsPerMillisecond*step<40) step=5000;

        long maxTimeVisible = firstTimestamp + (long)((hScrollBar.getValue() + visibleGraphWidth) / pixelsPerMillisecond);
        long minTimeVisible = firstTimestamp + (long)(hScrollBar.getValue() / pixelsPerMillisecond);

        // --- GRID LINE FIX ---
        // 1. Draw all light gray vertical grid lines first.
        mainGc.setStroke(Color.LIGHTGRAY);
        mainGc.setLineWidth(0.5);
        for (long t = (long)(Math.ceil(minTimeVisible / step) * step); t <= maxTimeVisible; t += step) {
            if (t < firstTimestamp) continue;
            double x = timeMsToScreenX(t);
            mainGc.strokeLine(x, 0, x, visibleGraphHeight - 1);
        }

        // 2. Draw all black tick marks and text labels on top.
        mainGc.setStroke(Color.BLACK);
        mainGc.setFill(Color.BLACK);
        mainGc.setLineWidth(1.0); // Reset for tick marks
        for (long t = (long)(Math.ceil(minTimeVisible / step) * step); t <= maxTimeVisible; t += step) {
            if (t < firstTimestamp) continue;
            double x = timeMsToScreenX(t);
            mainGc.strokeLine(x, visibleGraphHeight - 1, x, visibleGraphHeight - 1 + 4);
            mainGc.setFont(Font.font("Arial", 10));
            mainGc.setTextAlign(TextAlignment.CENTER);
            mainGc.fillText(String.format(Locale.US, "%.1f", (t - firstTimestamp) / 1000.0), x, visibleGraphHeight + 15);
        }
        // --- END GRID LINE FIX ---

        // Draw the "Seconds" label
        mainGc.setFont(Font.font("Arial", FontWeight.NORMAL, 12));
        mainGc.setTextAlign(TextAlignment.CENTER);
        mainGc.fillText("Seconds", hScrollBar.getValue() + visibleGraphWidth / 2.0, visibleGraphHeight + X_AXIS_LABEL_AREA_HEIGHT_ON_MAIN_CANVAS - 5);
    }

    private void drawData() {
        if (firstTimestamp == -1) return;
        long viewStartMs = firstTimestamp + (long)currentScrollOffsetMs;
        synchronized (plotEvents) {
            for(int i=0; i<LINE_STYLES.length; i++) { lastLinePointByStyle[i]=null; lastLinePointByStyle2[i]=null; }
            for (PlotDataEvent e : plotEvents) {
                if(e.getTimestamp() < viewStartMs){
                    if(e instanceof PlotLineEvent le) lastLinePointByStyle[le.getStyle()-1]=new PlotPoint(timeMsToScreenX(le.getTimestamp()),yValueToScreenY(le.getYValue()));
                    else if(e instanceof PlotLine2Event le2) lastLinePointByStyle2[le2.getStyle()-1]=new PlotPoint(timeMsToScreenX(le2.getTimestamp()),yValueToScreenY2(le2.getYValue()));
                } else {
                    break;
                }
            }
            for(PlotDataEvent e : plotEvents) {
                if(e instanceof PlotPointEvent p) drawPlotPoint(p);
                else if(e instanceof PlotLineEvent p) drawPlotLine(p);
                else if(e instanceof PlotPoint2Event p) drawPlotPoint2(p);
                else if(e instanceof PlotLine2Event p) drawPlotLine2(p);
            }
            for (PlotDataEvent e : plotEvents) { if (e instanceof PlotTextAnnotationEvent p) drawMarkerTextAnnotation(p); }
        }
    }

    private void drawCursor() {
        if (isMouseInPlotArea && pixelsPerMillisecond > 0 && firstTimestamp != -1) {
            currentCursorTimeMs = firstTimestamp + (long)((mousePlotX + hScrollBar.getValue()) / pixelsPerMillisecond);
        } else {
            currentCursorTimeMs = -1;
        }

        refreshKeyValueTable();
        updateAndDrawDataReadouts();

        if (!isMouseInPlotArea || currentCursorTimeMs == -1) {
            cursorXLabel.setVisible(false);
            cursorYLabel.setVisible(false);
            cursorY2Label.setVisible(false);
            return;
        }

        double cursorDrawX = mousePlotX + hScrollBar.getValue();

        mainGc.save();
        mainGc.setStroke(CURSOR_LINE_COLOR);
        mainGc.setLineWidth(1.0);
        mainGc.setLineDashes(5, 3);
        mainGc.setLineCap(StrokeLineCap.BUTT);
        mainGc.strokeLine(cursorDrawX, 0, cursorDrawX, visibleGraphHeight - 1);
        if (mousePlotY >= 0 && mousePlotY < visibleGraphHeight) {
            mainGc.strokeLine(hScrollBar.getValue(), mousePlotY, hScrollBar.getValue() + visibleGraphWidth, mousePlotY);
        }
        mainGc.restore();

        if (mousePlotY >= 0 && mousePlotY < visibleGraphHeight) {
            cursorXLabel.setText(String.format(Locale.US, "%.2fs", (currentCursorTimeMs - firstTimestamp) / 1000.0));
            cursorYLabel.setText(formatNiceNumber(screenYToYValue(mousePlotY), currentMaxY - currentMinY));

            double cursorScreenX = graphContainer.getLayoutX() + mousePlotX;
            cursorXLabel.setLayoutX(cursorScreenX - (cursorXLabel.prefWidth(-1) / 2));
            cursorXLabel.setLayoutY(PADDING_TOP + visibleGraphHeight + 3);

            cursorYLabel.setLayoutY(PADDING_TOP + mousePlotY - (cursorYLabel.prefHeight(-1) / 2));
            cursorYLabel.setLayoutX(PADDING_LEFT_FOR_Y_AXIS - cursorYLabel.prefWidth(-1) - 3);

            cursorXLabel.setVisible(true);
            cursorYLabel.setVisible(true);

            cursorY2Label.setText(formatNiceNumber(screenYToYValue2(mousePlotY), currentMaxY2 - currentMinY2));
            cursorY2Label.setLayoutY(PADDING_TOP + mousePlotY - (cursorY2Label.prefHeight(-1) / 2));
            cursorY2Label.setLayoutX(PADDING_LEFT_FOR_Y_AXIS + visibleGraphWidth + 3);
            cursorY2Label.setVisible(true);
        } else {
            cursorXLabel.setVisible(false);
            cursorYLabel.setVisible(false);
            cursorY2Label.setVisible(false);
        }
        updateReadoutLabelPositions();
    }

    private void drawPlotLine(PlotLineEvent e) { drawPlotLineSegment(e.getStyle(),e.getTimestamp(),e.getYValue(),lastLinePointByStyle,this::yValueToScreenY); }
    private void drawPlotLine2(PlotLine2Event e) { drawPlotLineSegment(e.getStyle(),e.getTimestamp(),e.getYValue(),lastLinePointByStyle2,this::yValueToScreenY2); }
    private void drawPlotLineSegment(int s, long t, double y, PlotPoint[] lp, java.util.function.Function<Double, Double> ym) {
        if(s<1||s>LINE_STYLES.length)return; int si=s-1; LineStyle ls=LINE_STYLES[si]; double x2=timeMsToScreenX(t), y2=ym.apply(y);
        if(lp[si]!=null){ mainGc.setStroke(ls.color); mainGc.setLineWidth(ls.width); mainGc.setLineDashes(ls.dashArray!=null?ls.dashArray:new double[0]); mainGc.strokeLine(lp[si].x,lp[si].y,x2,y2);}
        lp[si]=new PlotPoint(x2,y2);
    }
    private void drawPlotPoint(PlotPointEvent e){drawStyledPoint(e.getStyle(),e.getTimestamp(),e.getYValue(),this::yValueToScreenY);}
    private void drawPlotPoint2(PlotPoint2Event e){drawStyledPoint(e.getStyle(),e.getTimestamp(),e.getYValue(),this::yValueToScreenY2);}

    private void drawStyledPoint(int s, long t, double y, java.util.function.Function<Double, Double> ym){
        double yS=ym.apply(y), xS=timeMsToScreenX(t);
        mainGc.setFill((s>=1&&s<=LINE_STYLES.length)?LINE_STYLES[s-1].color:Color.BLACK); mainGc.fillOval(xS-2,yS-2,4,4);
    }

    private void drawMarkerTextAnnotation(PlotTextAnnotationEvent e){
        double x = timeMsToScreenX(e.getTimestamp());
        mainGc.save();
        mainGc.setStroke(MARKER_LINE_COLOR);
        mainGc.setLineWidth(1.0);
        mainGc.setLineDashes(5, 3);
        mainGc.strokeLine(x,0,x,visibleGraphHeight-1);
        mainGc.setFill(MARKER_TEXT_COLOR);
        mainGc.setFont(Font.font("Arial",FontWeight.BOLD,11));
        mainGc.setTextAlign(TextAlignment.CENTER);
        double y;
        switch(e.getPositionKeyword()){
            case"top":y=12;break;
            case"bot":y=visibleGraphHeight-5;break;
            default:y=visibleGraphHeight/2.0;break;
        }
        mainGc.fillText(e.getText(),x,y);
        mainGc.restore();
    }


    private double timeMsToScreenX(long t){return(t-firstTimestamp)*pixelsPerMillisecond;}
    private double yValueToScreenY(double y){return visibleGraphHeight-(((y-currentMinY)/(currentMaxY-currentMinY))*visibleGraphHeight);}
    private double yValueToScreenY2(double y){return visibleGraphHeight-(((y-currentMinY2)/(currentMaxY2-currentMinY2))*visibleGraphHeight);}
    private double screenYToYValue(double y){return currentMinY+((visibleGraphHeight-y)/visibleGraphHeight)*(currentMaxY-currentMinY);}
    private double screenYToYValue2(double y){return currentMinY2+((visibleGraphHeight-y)/visibleGraphHeight)*(currentMaxY2-currentMinY2);}

    public void setYLimits(double min,double max){if(max>min){this.currentMinY=min;this.currentMaxY=max;redrawFullPlot();}}
    public void setYUnit(String u){this.yAxisUnit=u;redrawYAxis();}
    public void setYLimits2(double min,double max){if(max>min){this.currentMinY2=min;this.currentMaxY2=max;redrawFullPlot();}}
    public void setYUnit2(String u){this.yAxisUnit2=u;redrawYAxis2();}

    public void savePlotData(File file) {
        StringBuilder sb = new StringBuilder();
        List<PlotDataEvent> allData = new ArrayList<>();
        synchronized(plotEvents){ allData.addAll(plotEvents); }
        synchronized(keyValueStore){for(Map.Entry<String,List<TimestampedStringValue>>e:keyValueStore.entrySet()){for(TimestampedStringValue v:e.getValue()){allData.add(new PlotKeyValueEvent(v.ts(),e.getKey(),v.v()));}}}
        allData.sort(Comparator.comparingLong(PlotDataEvent::getTimestamp));

        for(PlotDataEvent e:allData){
            long ts=e.getTimestamp();
            if(e instanceof PlotPointEvent p) sb.append(String.format(Locale.US,"POINT %d %d %.6f\n",ts,p.getStyle(),p.getYValue()));
            else if(e instanceof PlotLineEvent p) sb.append(String.format(Locale.US,"LINE %d %d %.6f\n",ts,p.getStyle(),p.getYValue()));
            else if(e instanceof PlotPoint2Event p) sb.append(String.format(Locale.US,"POINT2 %d %d %.6f\n",ts,p.getStyle(),p.getYValue()));
            else if(e instanceof PlotLine2Event p) sb.append(String.format(Locale.US,"LINE2 %d %d %.6f\n",ts,p.getStyle(),p.getYValue()));
            else if(e instanceof PlotKeyValueEvent p) sb.append(String.format("KV %d \"%s\" \"%s\"\n",ts,p.getKey(),p.getValue()));
            else if(e instanceof PlotTextAnnotationEvent p) sb.append(String.format("MARKER %d %s \"%s\"\n",ts,p.getPositionKeyword(),p.getText()));
            else if(e instanceof PlotYLimitsEvent p) sb.append(String.format(Locale.US,"YLIMITS %d %.6f %.6f\n",ts,p.getMinY(),p.getMaxY()));
            else if(e instanceof PlotYUnitsEvent p) sb.append(String.format("YUNITS %d \"%s\"\n",ts,p.getUnit()));
            else if(e instanceof PlotYLimits2Event p) sb.append(String.format(Locale.US,"YLIMITS2 %d %.6f %.6f\n",ts,p.getMinY(),p.getMaxY()));
            else if(e instanceof PlotYUnits2Event p) sb.append(String.format("YUNITS2 %d \"%s\"\n",ts,p.getUnit()));
        }
        try(BufferedWriter w=new BufferedWriter(new FileWriter(file))){w.write(sb.toString());}catch(IOException ex){ex.printStackTrace();}
    }

    public void loadPlotData(File file) {
        Platform.runLater(() -> {
            clearPlot();
            Pattern linePattern=Pattern.compile("^(\\S+)\\s(.*)$"), argPattern=Pattern.compile("\"([^\"]*)\"|\\S+");
            List<String>allLines=new ArrayList<>(); long minTs=Long.MAX_VALUE;
            try(BufferedReader r=new BufferedReader(new FileReader(file))){
                String line; while((line=r.readLine())!=null){allLines.add(line); Matcher m=linePattern.matcher(line.trim()); if(!m.matches())continue; String[]p=m.group(2).split(" ",2); if(p.length>0)try{long ts=Long.parseLong(p[0]);if(ts<minTs)minTs=ts;}catch(NumberFormatException ex){}}
                if(minTs==Long.MAX_VALUE){resetViewToFitData();return;}
                for(String sL:allLines){
                    Matcher lM=linePattern.matcher(sL.trim()); if(!lM.matches())continue;
                    String type=lM.group(1),argsStr=lM.group(2);
                    Matcher aM=argPattern.matcher(argsStr); List<String>args=new ArrayList<>(); while(aM.find())args.add(aM.group(1)!=null?aM.group(1):aM.group());
                    if(args.isEmpty())continue;
                    try{
                        long ts=Long.parseLong(args.get(0))-minTs; PlotDataEvent e=null;
                        switch(type){
                            case "POINT": if(args.size()>=3) e=new PlotPointEvent(ts,Double.parseDouble(args.get(2)),Integer.parseInt(args.get(1))); break;
                            case "LINE": if(args.size()>=3) e=new PlotLineEvent(ts,Double.parseDouble(args.get(2)),Integer.parseInt(args.get(1))); break;
                            case "POINT2": if(args.size()>=3) e=new PlotPoint2Event(ts,Double.parseDouble(args.get(2)),Integer.parseInt(args.get(1))); break;
                            case "LINE2": if(args.size()>=3) e=new PlotLine2Event(ts,Double.parseDouble(args.get(2)),Integer.parseInt(args.get(1))); break;
                            case "KV": if(args.size()>=3) e=new PlotKeyValueEvent(ts,args.get(1),args.get(2)); break;
                            case "MARKER": if(args.size()>=3) e=new PlotTextAnnotationEvent(ts,args.get(2),args.get(1)); break;
                            case "YLIMITS": if(args.size()>=3) e=new PlotYLimitsEvent(ts,Double.parseDouble(args.get(2)),Double.parseDouble(args.get(1))); break;
                            case "YUNITS": if(args.size()>=2) e=new PlotYUnitsEvent(ts,args.get(1)); break;
                            case "YLIMITS2": if(args.size()>=3) e=new PlotYLimits2Event(ts,Double.parseDouble(args.get(2)),Double.parseDouble(args.get(1))); break;
                            case "YUNITS2": if(args.size()>=2) e=new PlotYUnits2Event(ts,args.get(1)); break;
                        }
                        if(e!=null)processNewEvent(e);
                    }catch(NumberFormatException ex){System.err.println("Skipping malformed line: "+sL);}
                }
            }catch(IOException ex){ex.printStackTrace();}
            setAutoScrollEnabled(false); resetViewToFitData();
        });
    }

    private void updateAndDrawDataReadouts() {
        if (isMouseInPlotArea && currentCursorTimeMs != -1) {
            clearReadoutLabels();
            synchronized (plotEvents) {
                populateReadoutMaps(plotEvents, currentCursorTimeMs, styleToReadoutValueString, styleToReadoutDataY, styleToReadoutColor, PlotLineEvent.class, PlotPointEvent.class);
                populateReadoutMaps(plotEvents, currentCursorTimeMs, styleToReadoutValueString2, styleToReadoutDataY2, styleToReadoutColor2, PlotLine2Event.class, PlotPoint2Event.class);
            }
            staggerReadoutLabels();
        } else {
            clearReadoutLabels();
        }
        updateReadoutLabelNodes();
    }

    private void clearReadoutLabels() {
        styleToReadoutValueString.clear(); styleToReadoutDataY.clear(); styleToReadoutColor.clear();
        styleToReadoutValueString2.clear(); styleToReadoutDataY2.clear(); styleToReadoutColor2.clear();
    }
    private void populateReadoutMaps(List<PlotDataEvent> events, long time, Map<Integer,String> valMap, Map<Integer,Double> dataYMap, Map<Integer,Color> colorMap, Class<?> lineClass, Class<?> pointClass) {
        Set<Integer> styles = new TreeSet<>();
        for(PlotDataEvent e : events) {
            if(lineClass.isInstance(e)) {
                if (e instanceof PlotLineEvent ple) styles.add(ple.getStyle());
                else if (e instanceof PlotLine2Event ple2) styles.add(ple2.getStyle());
            } else if(pointClass.isInstance(e)) {
                if (e instanceof PlotPointEvent ppe) styles.add(ppe.getStyle());
                else if (e instanceof PlotPoint2Event ppe2) styles.add(ppe2.getStyle());
            }
        }
        for(int styleId : styles) {
            double interpolatedY = Double.NaN;
            PlotDataEvent lastEventOfStyle = null;
            for(PlotDataEvent e : events) {
                if((lineClass.isInstance(e) && getStyle(e) == styleId)) {
                    if(lastEventOfStyle != null){
                        long t1=lastEventOfStyle.getTimestamp(), t2=e.getTimestamp();
                        if((t1<=time && time<=t2) || (t2<=time && time<=t1)){
                            double y1=getYValue(lastEventOfStyle), y2=getYValue(e);
                            interpolatedY = (t1==t2)?y1:y1+(double)(time-t1)*(y2-y1)/(double)(t2-t1);
                            if(styleId>=1&&styleId<=LINE_STYLES.length)colorMap.put(styleId,LINE_STYLES[styleId-1].color);
                            break;
                        }
                    }
                    lastEventOfStyle = e;
                }
            }
            if(Double.isNaN(interpolatedY)){
                long minDiff=Long.MAX_VALUE;
                for(PlotDataEvent e:events){
                    if(getStyle(e)==styleId){
                        long diff=Math.abs(e.getTimestamp()-time);
                        if(diff<minDiff){
                            minDiff=diff;
                            interpolatedY=getYValue(e);
                            if(styleId>=1&&styleId<=LINE_STYLES.length) colorMap.put(styleId,LINE_STYLES[styleId-1].color);
                        }
                    }
                }
            }
            if(!Double.isNaN(interpolatedY)){ valMap.put(styleId,readoutValueFormat.format(interpolatedY)); dataYMap.put(styleId,interpolatedY); }
        }
    }
    private int getStyle(PlotDataEvent event) {
        if (event instanceof PlotLineEvent e) return e.getStyle();
        if (event instanceof PlotPointEvent e) return e.getStyle();
        if (event instanceof PlotLine2Event e) return e.getStyle();
        if (event instanceof PlotPoint2Event e) return e.getStyle();
        return -1;
    }
    private double getYValue(PlotDataEvent event) {
        if (event instanceof PlotLineEvent e) return e.getYValue();
        if (event instanceof PlotPointEvent e) return e.getYValue();
        if (event instanceof PlotLine2Event e) return e.getYValue();
        if (event instanceof PlotPoint2Event e) return e.getYValue();
        return Double.NaN;
    }

    private void staggerReadoutLabels() {
        Map<Integer,Double> combinedDataY = new HashMap<>();
        combinedDataY.putAll(styleToReadoutDataY);
        for(Map.Entry<Integer,Double> entry : styleToReadoutDataY2.entrySet()){ combinedDataY.put(entry.getKey() + 1000, entry.getValue()); } // Offset Y2 styles to avoid key collision
        List<Map.Entry<Integer, Double>> sorted = new ArrayList<>(combinedDataY.entrySet());
        sorted.sort(Map.Entry.comparingByValue());
        TreeSet<Double> slots = new TreeSet<>();
        for (Map.Entry<Integer, Double> entry : sorted) {
            int styleId = entry.getKey();
            double screenY = (styleId < 1000) ? yValueToScreenY(entry.getValue()) : yValueToScreenY2(entry.getValue());
            while(isSlotOccupied(screenY, slots)) screenY += READOUT_TEXT_Y_SPACING / 2.0;
            screenY = MathUtil.clip(screenY, 5, visibleGraphHeight - 10);
            if (styleId < 1000) styleToReadoutScreenY.put(styleId, screenY); else styleToReadoutScreenY2.put(styleId - 1000, screenY);
            slots.add(screenY - READOUT_TEXT_Y_SPACING/2.0); slots.add(screenY + READOUT_TEXT_Y_SPACING/2.0);
        }
    }
    private void updateReadoutLabelNodes() {
        Platform.runLater(()->{
            updateLabelSet(styleToReadoutLabel, styleToReadoutValueString, styleToReadoutColor);
            updateLabelSet(styleToReadoutLabel2, styleToReadoutValueString2, styleToReadoutColor2);
        });
    }
    private void updateLabelSet(Map<Integer, Label> labels, Map<Integer, String> values, Map<Integer, Color> colors){
        for(Map.Entry<Integer, Label> e:labels.entrySet()){if(!values.containsKey(e.getKey()))e.getValue().setVisible(false);}
        for(Map.Entry<Integer,String>e:values.entrySet()){
            int id=e.getKey(); String val=e.getValue(); Color c=colors.getOrDefault(id,Color.BLACK);
            Label l=labels.computeIfAbsent(id,i->{Label nL=new Label(); nL.setFont(READOUT_LABEL_FONT); nL.setStyle(READOUT_LABEL_STYLE); nL.setMouseTransparent(true);this.getChildren().add(nL);return nL;});
            l.setText(val); l.setTextFill(Color.WHITE); String r=(int)(c.getRed()*255)+"",g=(int)(c.getGreen()*255)+"",b=(int)(c.getBlue()*255)+"";
            l.setStyle(READOUT_LABEL_STYLE+String.format("-fx-background-color:rgb(%s,%s,%s);",r,g,b));
        }
    }

    private boolean isSlotOccupied(double y, TreeSet<Double> slots) { Double f=slots.floor(y),c=slots.ceiling(y); if(f!=null&&y-f<READOUT_TEXT_Y_SPACING)return true; return c!=null&&c-y<READOUT_TEXT_Y_SPACING; }
    private void updateReadoutLabelPositions() {
        if (!isMouseInPlotArea || mousePlotX < 0) { for(Label l:styleToReadoutLabel.values())l.setVisible(false); for(Label l:styleToReadoutLabel2.values())l.setVisible(false); return; }
        double cX=this.graphContainer.getLayoutX()+mousePlotX;
        positionLabelSet(styleToReadoutLabel, styleToReadoutScreenY, cX);
        positionLabelSet(styleToReadoutLabel2, styleToReadoutScreenY2, cX);
    }
    private void positionLabelSet(Map<Integer, Label> labels, Map<Integer, Double> screenYMap, double cursorX) {
        for (Map.Entry<Integer, Label> e : labels.entrySet()) {
            if (screenYMap.containsKey(e.getKey())) {
                Label l = e.getValue();
                l.setLayoutX(cursorX + READOUT_TEXT_X_OFFSET);
                l.setLayoutY(PADDING_TOP + screenYMap.get(e.getKey()));
                l.setVisible(true);
            } else {
                e.getValue().setVisible(false);
            }
        }
    }

    private record PlotPoint(double x, double y) {}
    private String formatNiceNumber(double v, double r) {if(r>=200||Math.abs(v)>=100)return String.format(Locale.US,"%.0f",v); if(r>=20||Math.abs(v)>=10)return String.format(Locale.US,"%.1f",v); if(r>=1||Math.abs(v)>=1)return String.format(Locale.US,"%.2f",v); return String.format(Locale.US,"%.3f",v); }
    private void updateCanvasWidthAndScrollbar(){if(firstTimestamp==-1||lastTimestamp==-1||firstTimestamp>lastTimestamp){mainGraphCanvas.setWidth(visibleGraphWidth);hScrollBar.setMin(0);hScrollBar.setMax(0);hScrollBar.setValue(0);hScrollBar.setVisibleAmount(visibleGraphWidth);hScrollBar.setDisable(true);currentScrollOffsetMs=0;return;}long d=lastTimestamp-firstTimestamp;double tS=d*pixelsPerMillisecond,cW=tS+PADDING_RIGHT_GRAPH,rW=Math.max(cW,visibleGraphWidth);mainGraphCanvas.setWidth(rW);hScrollBar.setMin(0);hScrollBar.setMax(cW);hScrollBar.setVisibleAmount(visibleGraphWidth);boolean dis=cW<=visibleGraphWidth;hScrollBar.setDisable(dis);if(dis){hScrollBar.setValue(0);currentScrollOffsetMs=0;}else{double mSV=hScrollBar.getMax()-hScrollBar.getVisibleAmount();if(mSV<0)mSV=0;if(hScrollBar.getValue()>mSV)hScrollBar.setValue(mSV);currentScrollOffsetMs=pixelsPerMillisecond>0?hScrollBar.getValue()/pixelsPerMillisecond:0;}}
    private void scrollToTimestamp(long ts){if(firstTimestamp==-1||pixelsPerMillisecond<=0||hScrollBar.isDisabled())return;double tSOV=visibleGraphWidth/pixelsPerMillisecond,dTALE=(ts-firstTimestamp)-(tSOV*0.95);if(dTALE<0)dTALE=0;double nSPV=dTALE*pixelsPerMillisecond,mPSV=hScrollBar.getMax()-hScrollBar.getVisibleAmount();if(mPSV<0)mPSV=0;nSPV=MathUtil.clip(nSPV,0,mPSV);if(Math.abs(hScrollBar.getValue()-nSPV)>0.5)hScrollBar.setValue(nSPV);}
    private void refreshKeyValueTable(){if(controlPanelProxy==null)return;List<PlotDisplayControlPanel.KeyTableEntry>entries=new ArrayList<>();if(currentCursorTimeMs!=-1){synchronized(keyValueStore){for(Map.Entry<String,List<TimestampedStringValue>>e:keyValueStore.entrySet()){List<TimestampedStringValue>vs=e.getValue();TimestampedStringValue rV=null;for(int i=vs.size()-1;i>=0;i--){if(vs.get(i).ts()<=currentCursorTimeMs){rV=vs.get(i);break;}}if(rV!=null)entries.add(new PlotDisplayControlPanel.KeyTableEntry(fT(rV.ts()),e.getKey(),rV.v()));}}}else{synchronized(keyValueStore){for(Map.Entry<String,List<TimestampedStringValue>>e:keyValueStore.entrySet()){List<TimestampedStringValue>vs=e.getValue();if(!vs.isEmpty()){TimestampedStringValue lV=vs.get(vs.size()-1);entries.add(new PlotDisplayControlPanel.KeyTableEntry(fT(lV.ts()),e.getKey(),lV.v()));}}}}entries.sort(Comparator.comparing(PlotDisplayControlPanel.KeyTableEntry::getKey));Platform.runLater(()->controlPanelProxy.updateKeyValueTable(entries));}
    private String fT(long t){if(firstTimestamp==-1)return"N/A";return String.format(Locale.US,"%.2f",(t-firstTimestamp)/1000.0);}
//    private void drawMarkerTextAnnotation(PlotTextAnnotationEvent e){double x=timeMsToScreenX(e.getTimestamp());if(x>=0&&x<=mainGraphCanvas.getWidth()){mainGc.save();mainGc.setStroke(MARKER_LINE_COLOR);mainGc.setLineWidth(1.0);mainGc.setLineDashes(5,3);mainGc.strokeLine(x,0,x,visibleGraphHeight-1);mainGc.setFill(MARKER_TEXT_COLOR);mainGc.setFont(Font.font("Arial",FontWeight.BOLD,11));mainGc.setTextAlign(TextAlignment.CENTER);double y;switch(e.getPositionKeyword()){case"top":y=PADDING_TOP+12;break;case"bot":y=visibleGraphHeight-5;break;default:y=visibleGraphHeight/2.0;break;}mainGc.fillText(e.getText(),x,y);mainGc.restore();}}
    public BooleanProperty autoScrollEnabledProperty() { return autoScrollEnabled; }
    public void setControlPanelProxy(PlotDisplayControlPanel p) {this.controlPanelProxy=p; if (p != null) p.setFollowRealTimeSelected(autoScrollEnabled.get());}
    public void stretchTimeAxis(double f){double old=pixelsPerMillisecond;pixelsPerMillisecond*=f;pixelsPerMillisecond=MathUtil.clip(pixelsPerMillisecond,MIN_PIXELS_PER_MS,MAX_PIXELS_PER_MS);setAutoScrollEnabled(false);if(Math.abs(old-pixelsPerMillisecond)>1e-9&&firstTimestamp!=-1){double vcT=currentScrollOffsetMs+(visibleGraphWidth/(2*old));updateCanvasWidthAndScrollbar();double nSV=(vcT*pixelsPerMillisecond)-(visibleGraphWidth/2.0);nSV=MathUtil.clip(nSV,0,hScrollBar.getMax()-hScrollBar.getVisibleAmount());if(hScrollBar.getMax()<=visibleGraphWidth)nSV=0;hScrollBar.setValue(nSV);currentScrollOffsetMs=hScrollBar.getValue()/pixelsPerMillisecond;}else{updateCanvasWidthAndScrollbar();}redrawFullPlot();}
    public void resetViewToFitData(){if(plotEvents.isEmpty()){firstTimestamp=-1;lastTimestamp=-1;updateCanvasWidthAndScrollbar();redrawFullPlot();return;}long minTs=Long.MAX_VALUE,maxTs=Long.MIN_VALUE;synchronized(plotEvents){for(PlotDataEvent e:plotEvents){if(e.getTimestamp()<minTs)minTs=e.getTimestamp();if(e.getTimestamp()>maxTs)maxTs=e.getTimestamp();}}synchronized(keyValueStore){for(Map.Entry<String,List<TimestampedStringValue>>e:keyValueStore.entrySet()){for(TimestampedStringValue v:e.getValue()){if(v.ts()<minTs)minTs=v.ts();if(v.ts()>maxTs)maxTs=v.ts();}}}if(minTs==Long.MAX_VALUE){firstTimestamp=-1;lastTimestamp=-1;}else{firstTimestamp=minTs;lastTimestamp=maxTs;}long d=lastTimestamp-firstTimestamp;if(d>0&&visibleGraphWidth>0)pixelsPerMillisecond=(visibleGraphWidth*0.98)/d;else pixelsPerMillisecond=0.02;pixelsPerMillisecond=MathUtil.clip(pixelsPerMillisecond,MIN_PIXELS_PER_MS,MAX_PIXELS_PER_MS);updateCanvasWidthAndScrollbar();hScrollBar.setValue(0);redrawFullPlot();}
}
