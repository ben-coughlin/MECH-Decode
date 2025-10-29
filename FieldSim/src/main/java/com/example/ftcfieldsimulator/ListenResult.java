package com.example.ftcfieldsimulator;

import java.util.ArrayList;

public class ListenResult {
    private final ListenStatus status;
    private final ArrayList<CurvePoint> points;
    private final String errorMessage;

    public ListenResult(ListenStatus status, ArrayList<CurvePoint> points, String errorMessage) {
        this.status = status;
        this.points = points != null ? new ArrayList<>(points) : new ArrayList<>(); // Defensive copy
        this.errorMessage = errorMessage;
    }

    public ListenStatus getStatus() {
        return status;
    }

    public ArrayList<CurvePoint> getPoints() {
        return new ArrayList<>(points); // Return a copy to prevent external modification
    }

    public String getErrorMessage() {
        return errorMessage;
    }

    public boolean hasData() {
        return points != null && !points.isEmpty();
    }
}

