package com.example.ftcfieldsimulator;

/**
 * Event to set the Y-axis unit label for the secondary (right) axis.
 */
public class PlotYUnits2Event implements PlotDataEvent {
    private final long timestamp;
    private final String unit;

    public PlotYUnits2Event(long timestamp, String unit) {
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
        return "PlotYUnits2Event{" +
                "timestamp=" + timestamp +
                ", unit='" + unit + '\'' +
                '}';
    }
}
