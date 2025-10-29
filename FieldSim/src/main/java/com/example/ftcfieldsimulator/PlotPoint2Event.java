package com.example.ftcfieldsimulator;

/**
 * Represents a single, disconnected data point for the secondary (right) Y-axis.
 */
public class PlotPoint2Event implements PlotDataEvent {
    private final long timestamp;
    private final double yValue;
    private final int style;

    public PlotPoint2Event(long timestamp, double yValue, int style) {
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
        return "PlotPoint2Event{" +
                "timestamp=" + timestamp +
                ", yValue=" + yValue +
                ", style=" + style +
                '}';
    }
}
