package com.example.ftcfieldsimulator; // Adjust package as needed

import javafx.scene.image.Image;
import javafx.scene.paint.Color;

public class Robot {

    // --- Robot Configuration Constants ---
    public static final double ROBOT_WIDTH_INCHES = 18.0;
    public static final double ROBOT_HEIGHT_INCHES = 18.0;
    private double xInches;         // Current X position on the field
    private double yInches;         // Current Y position on the field
    private double headingDegrees;  // Current heading (degrees, 0 might be facing right/East, 90 North)

    private Image robotImage;
    private static final String DEFAULT_ROBOT_IMAGE_PATH = "/robot.png";
    public Robot(double initialXInches, double initialYInches, double initialHeadingDegrees, String imagePath) {
        this.xInches = initialXInches;
        this.yInches = initialYInches;
        this.headingDegrees = initialHeadingDegrees;
        this.robotImage = ImageLoader.loadImage(
                imagePath, // Path already includes leading slash if passed correctly
                (int) (ROBOT_WIDTH_INCHES * 5), // Placeholder size, arbitrary scaling for visibility
                (int) (ROBOT_HEIGHT_INCHES * 5),
                "Robot",
                Color.DIMGRAY
        );
    }

    // Overloaded constructor using default image path
    public Robot(double xInches, double yInches, double headingDegrees) {
        this(xInches, yInches, headingDegrees, DEFAULT_ROBOT_IMAGE_PATH);
    }

    // Getters
    public double getXInches() {
        return xInches;
    }

    public double getYInches() {
        return yInches;
    }

    public double getHeadingDegrees() {
        return headingDegrees;
    }

    public Image getRobotImage() {
        return robotImage;
    }

    // Setters - these will be used by the UDP listener later
    public void setPosition(double xInches, double yInches) {
        this.xInches = xInches;
        this.yInches = yInches;
    }

    public void setHeading(double headingDegrees) {
        this.headingDegrees = headingDegrees;
    }

    public void updatePose(double xInches, double yInches, double headingDegrees) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingDegrees = headingDegrees;
    }
}
