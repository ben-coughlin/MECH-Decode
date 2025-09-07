package com.example.ftcfieldsimulator;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

public class ControlPanel extends VBox {

    public static final double PREFERRED_WIDTH_RATIO_TO_FIELD = 1.0 / 3.0; // 1/3 of field width

    // Labels for status display
    private Label robotXLabel;
    private Label robotYLabel;
    private Label robotHeadingLabel;

    // Labels for the actual values (to be updated)
    private Label robotXValueLabel;
    private Label robotYValueLabel;
    private Label robotHeadingValueLabel;

    private Button newPathButton;
    private Button deletePathButton;
    private Button exportPathButton;
    private Button clearTrailButton;
    private Button clearNamedLinesButton;

    public ControlPanel(double preferredWidth) {
        super(10); // Spacing between elements in VBox
        setPadding(new Insets(15));
        setPrefWidth(preferredWidth);
        setStyle("-fx-background-color: #ECEFF1;"); // Light gray background for the control pane

        Label controlsTitle = new Label("Controls");
        controlsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 20));
        controlsTitle.setTextFill(Color.DARKSLATEBLUE);
        controlsTitle.setMaxWidth(Double.MAX_VALUE); // Allow title to take full width
        controlsTitle.setAlignment(Pos.CENTER);     // Center title text

//        // Placeholder for future actual controls
//        VBox mainControlsArea = new VBox(8); // Spacing for controls
//        mainControlsArea.setPadding(new Insets(10, 0, 10, 0)); // Padding around controls area
//        Label placeholderLabel1 = new Label("Robot X: [future_textField]");
//        Label placeholderLabel2 = new Label("Robot Y: [future_textField]");
//        Label placeholderLabel3 = new Label("Robot H: [future_slider]");
//        Label placeholderLabel4 = new Label("Path Point: [future_button_add]");
//        mainControlsArea.getChildren().addAll(placeholderLabel1, placeholderLabel2, placeholderLabel3, placeholderLabel4);

        // --- Path Controls Section ---
        Label pathControlsTitle = new Label("Path Management");
        pathControlsTitle.setFont(Font.font("Arial", FontWeight.BOLD, 16));
        pathControlsTitle.setPadding(new Insets(10, 0, 5, 0));

        newPathButton = new Button("New Path");
        newPathButton.setMaxWidth(Double.MAX_VALUE);
        deletePathButton = new Button("Delete Path");
        deletePathButton.setMaxWidth(Double.MAX_VALUE);
        exportPathButton = new Button("Export Path (CSV)");
        exportPathButton.setMaxWidth(Double.MAX_VALUE);
        clearTrailButton = new Button("Clear Robot Trail");
        clearTrailButton.setMaxWidth(Double.MAX_VALUE);
        clearNamedLinesButton = new Button("Clear Custom Lines");
        clearNamedLinesButton.setMaxWidth(Double.MAX_VALUE);

        VBox pathControlsBox = new VBox(8, pathControlsTitle, newPathButton, deletePathButton, exportPathButton, clearTrailButton, clearNamedLinesButton);
        pathControlsBox.setPadding(new Insets(0,0,10,0));


        // --- Placeholder for other future controls (e.g., Robot Manual Control) ---
        VBox otherControlsArea = new VBox(8);
        otherControlsArea.setPadding(new Insets(10, 0, 10, 0));
        // Add other controls here if needed in the future

        // --- Status Section ---
        Label statusTitle = new Label("Status");
        statusTitle.setFont(Font.font("Arial", FontWeight.BOLD, 18));
        statusTitle.setTextFill(Color.DARKSLATEBLUE);
        statusTitle.setPadding(new Insets(10, 0, 5, 0)); // Top padding for status title
        statusTitle.setMaxWidth(Double.MAX_VALUE);
        statusTitle.setAlignment(Pos.CENTER);

        // Initialize status labels
        robotXLabel = new Label("Robot X:");
        robotYLabel = new Label("Robot Y:");
        robotHeadingLabel = new Label("Heading:");

        robotXValueLabel = new Label("N/A");
        robotYValueLabel = new Label("N/A");
        robotHeadingValueLabel = new Label("N/A");

        // Style value labels
        Font valueFont = Font.font("Arial", 14);
        robotXValueLabel.setFont(valueFont);
        robotYValueLabel.setFont(valueFont);
        robotHeadingValueLabel.setFont(valueFont);

        // Layout for status items using HBox for label + value pairs
        HBox xStatusBox = new HBox(5, robotXLabel, robotXValueLabel);
        HBox yStatusBox = new HBox(5, robotYLabel, robotYValueLabel);
        HBox headingStatusBox = new HBox(5, robotHeadingLabel, robotHeadingValueLabel);
        xStatusBox.setAlignment(Pos.CENTER_LEFT);
        yStatusBox.setAlignment(Pos.CENTER_LEFT);
        headingStatusBox.setAlignment(Pos.CENTER_LEFT);

        VBox statusInfoBox = new VBox(5); // Vertical layout for all status items
        statusInfoBox.setPadding(new Insets(5, 10, 10, 10)); // Padding around the status info
        statusInfoBox.setStyle("-fx-border-color: #B0BEC5; -fx-border-width: 1; -fx-border-radius: 5;"); // Light border
        statusInfoBox.getChildren().addAll(xStatusBox, yStatusBox, headingStatusBox);

        // --- Layout within ControlPanel ---
        // Spacer to push status section to the bottom
        VBox spacer = new VBox();
        VBox.setVgrow(spacer, Priority.ALWAYS); // Spacer takes all available vertical space

        getChildren().addAll(controlsTitle, pathControlsBox, otherControlsArea, spacer, statusTitle, statusInfoBox);
    }

    // --- Public methods to update status display ---
    public void updateRobotStatus(double x, double y, double heading) {
        // Format the numbers for display (e.g., to 2 decimal places)
        robotXValueLabel.setText(String.format("%.2f in", x));
        robotYValueLabel.setText(String.format("%.2f in", y));
        robotHeadingValueLabel.setText(String.format("%.1f°", heading));
    }

    public void setOnNewPathAction(EventHandler<ActionEvent> handler) {
        newPathButton.setOnAction(handler);
    }

    public void setOnDeletePathAction(EventHandler<ActionEvent> handler) {
        deletePathButton.setOnAction(handler);
    }

    public void setOnExportPathAction(EventHandler<ActionEvent> handler) {
        exportPathButton.setOnAction(handler);
    }

    public void setOnClearTrailAction(EventHandler<ActionEvent> handler) {
        clearTrailButton.setOnAction(handler);
    }

    public void setOnClearNamedLinesAction(EventHandler<ActionEvent> handler) {
        clearNamedLinesButton.setOnAction(handler);
    }

    // --- Methods to enable/disable buttons ---
    public void setPathEditingActive(boolean isActive) {
        newPathButton.setDisable(isActive); // Disable "New Path" while editing
        deletePathButton.setDisable(isActive);
        exportPathButton.setDisable(isActive);
    }

    public void enablePathControls(boolean pathExists) {
        deletePathButton.setDisable(!pathExists);
        exportPathButton.setDisable(!pathExists);
    }
}