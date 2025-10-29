// Create this file: PlotKeyValueEvent.java
package com.example.ftcfieldsimulator;

public class PlotKeyValueEvent implements PlotDataEvent {
    private final long timestamp;
    private final String key;
    private final String value;

    public PlotKeyValueEvent(long timestamp, String key, String value) {
        this.timestamp = timestamp;
        this.key = key;
        this.value = value;
    }

    @Override
    public long getTimestamp() {
        return timestamp;
    }

    public String getKey() {
        return key;
    }

    public String getValue() {
        return value;
    }

    @Override
    public String toString() {
        return "PlotKeyValueEvent{" +
                "timestamp=" + timestamp +
                ", key='" + key + '\'' +
                ", value='" + value + '\'' +
                '}';
    }
}
