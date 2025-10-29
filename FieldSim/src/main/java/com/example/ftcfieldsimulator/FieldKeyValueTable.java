package com.example.ftcfieldsimulator;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.collections.ObservableMap;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class FieldKeyValueTable extends VBox {

    private TableView<Map.Entry<String, String>> keyValueTable;
    private final ObservableList<Map.Entry<String, String>> tableData;
    private final ObservableMap<String, String> dataMap = FXCollections.observableMap(new  ConcurrentHashMap<>());

    public FieldKeyValueTable(double preferredWidth) {
        super(10); // Spacing
        setPadding(new Insets(10));
        setPrefWidth(preferredWidth);
        setStyle("-fx-background-color: #ECEFF1;");
        setAlignment(Pos.TOP_CENTER);

        Label title = new Label("Live Telemetry");
        title.setFont(Font.font("Arial", FontWeight.BOLD, 14));
        title.setPadding(new Insets(0, 0, 10, 0));

        keyValueTable = new TableView<>();
        tableData = FXCollections.observableArrayList(dataMap.entrySet());
        keyValueTable.setItems(tableData);

        dataMap.addListener((javafx.collections.MapChangeListener<String, String>) change -> {
            tableData.setAll(dataMap.entrySet());
        });

        TableColumn<Map.Entry<String, String>, String> keyCol = new TableColumn<>("Key");
        keyCol.setCellValueFactory(p -> new SimpleStringProperty(p.getValue().getKey()));
        keyCol.setPrefWidth(preferredWidth * 0.4);

        TableColumn<Map.Entry<String, String>, String> valueCol = new TableColumn<>("Value");
        valueCol.setCellValueFactory(p -> new SimpleStringProperty(p.getValue().getValue()));
        valueCol.prefWidthProperty().bind(
                keyValueTable.widthProperty()
                        .subtract(keyCol.widthProperty())
                        .subtract(2)
        );

        keyValueTable.getColumns().setAll(keyCol, valueCol);
        keyValueTable.setPlaceholder(new Label("No telemetry received"));
        VBox.setVgrow(keyValueTable, Priority.ALWAYS);

        getChildren().addAll(title, keyValueTable);
    }

    public void updateValue(String key, String value) {
        dataMap.put(key, value);
    }
}









