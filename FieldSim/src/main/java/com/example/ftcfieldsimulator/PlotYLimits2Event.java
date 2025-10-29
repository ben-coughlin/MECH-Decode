package com.example.ftcfieldsimulator;

/**
 * Event to set the Y-axis limits for the secondary (right) axis.
 */
public class PlotYLimits2Event implements PlotDataEvent {
    private final long timestamp;
    private final double maxY;
    private final double minY;

    public PlotYLimits2Event(long timestamp, double maxY, double minY) {
        this.timestamp = timestamp;
        this.maxY = maxY;
        this.minY = minY;
    }

    @Override
    public long getTimestamp() {
        return timestamp;
    }

    public double getMaxY() {
        return maxY;
    }

    public double getMinY() {
        return minY;
    }

    @Override
    public String toString() {
        return "PlotYLimits2Event{" +
                "timestamp=" + timestamp +
                ", maxY=" + maxY +
                ", minY=" + minY +
                '}';
    }
}
