// Create this file: PlotYLimitsEvent.java
package com.example.ftcfieldsimulator;

public class PlotYLimitsEvent implements PlotDataEvent {
    private final long timestamp;
    private final double maxY;
    private final double minY;

    public PlotYLimitsEvent(long timestamp, double maxY, double minY) {
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
        return "PlotYLimitsEvent{" +
                "timestamp=" + timestamp +
                ", maxY=" + maxY +
                ", minY=" + minY +
                '}';
    }
}
