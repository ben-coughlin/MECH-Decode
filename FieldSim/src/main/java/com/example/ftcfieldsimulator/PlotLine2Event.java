package com.example.ftcfieldsimulator;

/**
 * Represents a data point for a line series on the secondary (right) Y-axis.
 * This is functionally identical to PlotLineEvent but is a distinct type
 * to be routed to the second Y-axis in the plot display.
 */
public class PlotLine2Event implements PlotDataEvent {
    private final long timestamp;
    private final double yValue;
    private final int style; // 1-10, maps to LineStyle array

    public PlotLine2Event(long timestamp, double yValue, int style) {
        this.timestamp = timestamp;
        this.yValue = yValue;
        this.style = style;
    }

    @Override
    public long getTimestamp() {
        return timestamp;
    }

    public double getYValue() {
        return yValue;
    }

    public int getStyle() {
        return style;
    }

    @Override
    public String toString() {
        return "PlotLine2Event{" +
                "timestamp=" + timestamp +
                ", yValue=" + yValue +
                ", style=" + style +
                '}';
    }
}
