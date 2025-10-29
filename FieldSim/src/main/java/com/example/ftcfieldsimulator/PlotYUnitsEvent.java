// Create this file: PlotYUnitsEvent.java
package com.example.ftcfieldsimulator;

public class PlotYUnitsEvent implements PlotDataEvent {
    private final long timestamp;
    private final String unit;

    public PlotYUnitsEvent(long timestamp, String unit) {
        this.timestamp = timestamp;
        this.unit = unit;
    }

    @Override
    public long getTimestamp() {
        return timestamp;
    }

    public String getUnit() {
        return unit;
    }

    @Override
    public String toString() {
        return "PlotYUnitsEvent{" +
                "timestamp=" + timestamp +
                ", unit='" + unit + '\'' +
                '}';
    }
}