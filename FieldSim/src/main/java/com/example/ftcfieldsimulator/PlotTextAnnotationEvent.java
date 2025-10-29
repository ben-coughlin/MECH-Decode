// In PlotTextAnnotationEvent.java
package com.example.ftcfieldsimulator;

// Note: We are re-purposing this event for the new "text marker" requirement.
// The 'yValue' and 'color' fields from the previous text annotation are no longer directly used
// in the same way. 'yValue' could be re-interpreted or ignored. 'Color' is fixed for the line.
// The position ("top", "mid", "bot") is new.

public class PlotTextAnnotationEvent implements PlotDataEvent {
    private final long timestamp; // X-coordinate for the vertical line and text anchor
    private final String text;
    private final String positionKeyword; // "top", "mid", "bot"

    // Color of the vertical line can be fixed or added as a field if needed later
    // For now, PlotDisplay will handle the line color.

    public PlotTextAnnotationEvent(long timestamp, String text, String positionKeyword) {
        this.timestamp = timestamp;
        this.text = text;
        this.positionKeyword = positionKeyword != null ? positionKeyword.toLowerCase() : "mid"; // Default to mid
    }

    @Override
    public long getTimestamp() {
        return timestamp;
    }

    public String getText() {
        return text;
    }

    public String getPositionKeyword() {
        return positionKeyword;
    }

    @Override
    public String toString() {
        return "PlotTextAnnotationEvent{" + // Consider renaming class if this feels too different
                "timestamp=" + timestamp +
                ", text='" + text + '\'' +
                ", positionKeyword='" + positionKeyword + '\'' +
                '}';
    }
}

