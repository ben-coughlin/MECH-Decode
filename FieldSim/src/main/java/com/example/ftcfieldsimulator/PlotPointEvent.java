// Create this file: PlotPointEvent.java
package com.example.ftcfieldsimulator;

public class PlotPointEvent implements PlotDataEvent {
    private final long timestamp;
    private final double yValue;
    private final int style;

    public PlotPointEvent(long timestamp, double yValue, int style) {
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
        return "PlotPointEvent{" +
                "timestamp=" + timestamp +
                ", yValue=" + yValue +
                ", style=" + style +
                '}';
    }
}
