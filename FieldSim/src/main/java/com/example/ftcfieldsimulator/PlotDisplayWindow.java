// Create this file: PlotDisplayWindow.java
package com.example.ftcfieldsimulator;
import com.example.ftcfieldsimulator.PlotDisplayControlPanel;

import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.stage.Modality;
import javafx.stage.Window;
import javafx.stage.FileChooser; // Add this import
import java.io.File; // Add this import

public class PlotDisplayWindow {

    private Stage plotStage;
    private PlotDisplay plotDisplay;
    private PlotDisplayControlPanel plotControlPanel;

    public PlotDisplayWindow(Window owner) {
        plotStage = new Stage();
        plotStage.initModality(Modality.NONE);
        plotStage.initOwner(owner);
        plotStage.setTitle("Time Series Plot");

        BorderPane rootLayout = new BorderPane();

        plotDisplay = new PlotDisplay(
                PlotDisplay.DEFAULT_PLOT_AREA_WIDTH_PIXELS, // Correct constant
                PlotDisplay.DEFAULT_PLOT_AREA_HEIGHT_PIXELS // Correct constant
        );

        plotControlPanel = new PlotDisplayControlPanel();
        plotControlPanel.setOnClearPlotAction(e -> {
            if (plotDisplay != null) {
                plotDisplay.clearPlot();
            }
        });
        plotControlPanel.setOnStretchTimeAction(e -> {
            if (plotDisplay != null) {
                plotDisplay.stretchTimeAxis(1.25);
            }
        });
        plotControlPanel.setOnShrinkTimeAction(e -> {
            if (plotDisplay != null) {
                plotDisplay.stretchTimeAxis(0.8);
            }
        });

        plotControlPanel.setOnFollowRealTimeAction(e -> {
            if (plotDisplay != null) {
                plotDisplay.setAutoScrollEnabled(plotControlPanel.isFollowRealTimeSelected());
            }
        });

        plotControlPanel.setOnSaveAction(e -> {
            if (plotDisplay == null) return;

            FileChooser fileChooser = new FileChooser();
            fileChooser.setTitle("Save Plot Data");
            fileChooser.getExtensionFilters().addAll(
                    new FileChooser.ExtensionFilter("Plot Data Files", "*.pdat"),
                    new FileChooser.ExtensionFilter("Text Files", "*.txt"),
                    new FileChooser.ExtensionFilter("All Files", "*.*")
            );
            File file = fileChooser.showSaveDialog(plotStage);
            if (file != null) {
                plotDisplay.savePlotData(file);
            }
        });

        plotControlPanel.setOnLoadAction(e -> {
            if (plotDisplay == null) return;

            FileChooser fileChooser = new FileChooser();
            fileChooser.setTitle("Load Plot Data");
            fileChooser.getExtensionFilters().addAll(
                    new FileChooser.ExtensionFilter("Plot Data Files", "*.pdat"),
                    new FileChooser.ExtensionFilter("Text Files", "*.txt"),
                    new FileChooser.ExtensionFilter("All Files", "*.*")
            );
            File file = fileChooser.showOpenDialog(plotStage);
            if (file != null) {
                plotDisplay.loadPlotData(file);
            }
        });

        if (plotDisplay != null) {
            plotDisplay.setControlPanelProxy(plotControlPanel);
        }

        rootLayout.setCenter(plotDisplay);
        rootLayout.setLeft(plotControlPanel);

        // Scene size needs to accommodate the control panel width
        // Use the preferred sizes of the components for a more accurate scene dimension.
        // PlotDisplay's preferred size now includes its internal padding, Y-axis area, and scrollbar.
        // PlotDisplayControlPanel reports its own preferred width.

        // Get these values AFTER plotDisplay and plotControlPanel are constructed.
        double actualPlotDisplayWidth = plotDisplay.getPrefWidth();
        double actualPlotDisplayHeight = plotDisplay.getPrefHeight();
        double controlPanelWidth = plotControlPanel.getPrefWidth();
        double controlPanelHeight = plotControlPanel.getPrefHeight(); // Though often less critical than width

        // The scene width is the sum of the plot display's actual width and the control panel's width
        double sceneWidth = actualPlotDisplayWidth + controlPanelWidth;
        // The scene height is the maximum of the two components' heights
        double sceneHeight = Math.max(actualPlotDisplayHeight, controlPanelHeight);


        Scene scene = new Scene(rootLayout, sceneWidth, sceneHeight);
        plotStage.setScene(scene);

        plotStage.setOnCloseRequest(event -> {
            System.out.println("Plot Display Window closing.");
        });
    }

    public void show() {
        if (!plotStage.isShowing()) {
            plotStage.show();
        } else {
            plotStage.toFront();
        }
    }

    public void hide() {
        plotStage.hide();
    }

    public boolean isShowing() {
        return plotStage.isShowing();
    }

    // Method to pass data to the plot display (will be used in later tasks)
    public PlotDisplay getPlotDisplay() {
        return plotDisplay;
    }
}
