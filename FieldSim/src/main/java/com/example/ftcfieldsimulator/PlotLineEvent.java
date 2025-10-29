// Create/Update this file: PlotLineEvent.java
package com.example.ftcfieldsimulator;

public class PlotLineEvent implements PlotDataEvent {
    private final long timestamp;
    private final double yValue;
    private final int style; // 1-10, maps to LineStyle array

    public PlotLineEvent(long timestamp, double yValue, int style) {
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
        return "PlotLineEvent{" +
                "timestamp=" + timestamp +
                ", yValue=" + yValue +
                ", style=" + style +
                '}';
    }
}
