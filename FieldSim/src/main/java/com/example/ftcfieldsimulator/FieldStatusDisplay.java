package com.example.ftcfieldsimulator;

import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

import java.util.Locale;

public class FieldStatusDisplay extends VBox {

    private final Label lblXValue;
    private final Label lblYValue;
    private final Label lblHeadingValue;

    public FieldStatusDisplay() {
        super(5); // Spacing for VBox
        setPadding(new Insets(10));
        setAlignment(Pos.CENTER_LEFT);
        setStyle("-fx-background-color: #ECEFF1;");

        Label title = new Label("Robot Status");
        title.setFont(Font.font("Arial", FontWeight.BOLD, 14));
        title.setPadding(new Insets(0, 0, 10, 0));

        GridPane statusGrid = new GridPane();
        statusGrid.setHgap(10);
        statusGrid.setVgap(5);

        // --- Create Labels ---
        lblXValue = new Label("-");
        lblYValue = new Label("-");
        lblHeadingValue = new Label("-");

        // --- Add to Grid ---
        statusGrid.add(new Label("X (in):"), 0, 0);
        statusGrid.add(lblXValue, 1, 0);
        statusGrid.add(new Label("Y (in):"), 0, 1);
        statusGrid.add(lblYValue, 1, 1);
        statusGrid.add(new Label("Heading (°):"), 0, 2);
        statusGrid.add(lblHeadingValue, 1, 2);

        getChildren().addAll(title, statusGrid);
    }

    /**
     * Updates the displayed robot status values.
     */
    public void updateRobotStatus(double xInches, double yInches, double headingDegrees) {
        lblXValue.setText(String.format(Locale.US, "%.2f", xInches));
        lblYValue.setText(String.format(Locale.US, "%.2f", yInches));
        lblHeadingValue.setText(String.format(Locale.US, "%.1f", headingDegrees));
    }
}
