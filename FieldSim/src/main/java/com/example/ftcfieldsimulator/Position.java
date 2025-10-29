// Create this file in:
// /Users/alejandro/StudioProjects/MECH-Decode/FtcFieldSimulator/src/main/java/com/example/ftcfieldsimulator/Position.java

package com.example.ftcfieldsimulator;

public class Position {
    public double x;            // X-coordinate in inches (field coordinate system)
    public double y;            // Y-coordinate in inches (field coordinate system)
    public double heading;      // Heading in degrees (e.g., 0 for East, 90 for North)

    // Constructor for a point with x, y, and heading
    public Position(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Constructor for a point with only x and y (heading might be defaulted or set later)
    public Position(double x, double y) {
        this.x = x;
        this.y = y;
        this.heading = 0.0; // Default heading to 0 degrees (East) or another sensible default
    }

    // --- Optional: Getters and Setters if you prefer private fields ---
    // public double getX() { return x; }
    // public void setX(double x) { this.x = x; }
    // public double getY() { return y; }
    // public void setY(double y) { this.y = y; }
    // public double getHeading() { return heading; }
    // public void setHeading(double heading) { this.heading = heading; }

    @Override
    public String toString() {
        return "Position{" +
                "x=" + x +
                ", y=" + y +
                ", heading=" + heading +
                '}';
    }
}
