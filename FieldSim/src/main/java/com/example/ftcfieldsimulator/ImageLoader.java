package com.example.ftcfieldsimulator; // Or your utility package

import java.io.InputStream;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.Text;

public class ImageLoader {

    /**
     * Loads an image from the classpath. If loading fails or the image data is invalid,
     * a placeholder image is returned.
     *
     * @param resourcePath The path to the image resource (e.g., "/field.png", "/robot.png").
     *                     The leading slash indicates the root of the classpath.
     * @param desiredPlaceholderWidth  Approximate desired width for the placeholder image if created.
     * @param desiredPlaceholderHeight Approximate desired height for the placeholder image if created.
     * @param placeholderMessage       The message to display on the placeholder image.
     * @param placeholderColor         The background color for the placeholder image.
     * @return The loaded Image object, or a placeholder Image if loading failed.
     */
    public static Image loadImage(String resourcePath,
                                  int desiredPlaceholderWidth,
                                  int desiredPlaceholderHeight,
                                  String placeholderMessage,
                                  Color placeholderColor) {
        Image image;
        try {
            InputStream imageStream = ImageLoader.class.getResourceAsStream(resourcePath);
            if (imageStream == null) {
                System.err.println("Image resource not found in classpath: " + resourcePath);
                image = createPlaceholderImage(desiredPlaceholderWidth, desiredPlaceholderHeight, placeholderColor, placeholderMessage + "\n(Not Found)");
            } else {
                image = new Image(imageStream);
                if (image.isError()) {
                    System.err.println("Error decoding image data for: " + resourcePath);
                    if (image.getException() != null) {
                        image.getException().printStackTrace();
                    }
                    image = createPlaceholderImage(desiredPlaceholderWidth, desiredPlaceholderHeight, placeholderColor, placeholderMessage + "\n(Load Error)");
                } else {
                    System.out.println("Successfully loaded image: " + resourcePath);
                }
            }
        } catch (Exception e) {
            System.err.println("Generic exception loading image: " + resourcePath);
            e.printStackTrace();
            image = createPlaceholderImage(desiredPlaceholderWidth, desiredPlaceholderHeight, placeholderColor, placeholderMessage + "\n(Exception)");
        }
        return image;
    }

    /**
     * Creates a placeholder image with a solid color and a message.
     *
     * @param width   The width of the placeholder image.
     * @param height  The height of the placeholder image.
     * @param color   The background color of the placeholder.
     * @param message The message to display on the placeholder.
     * @return A WritableImage serving as a placeholder.
     */
    public static Image createPlaceholderImage(int width, int height, Color color, String message) {
        // Ensure minimum dimensions for placeholder
        int actualWidth = Math.max(width, 64); // Min width 64px
        int actualHeight = Math.max(height, 64); // Min height 64px

        WritableImage writableImage = new WritableImage(actualWidth, actualHeight);
        Canvas placeholderCanvas = new Canvas(actualWidth, actualHeight);
        GraphicsContext pgc = placeholderCanvas.getGraphicsContext2D();

        pgc.setFill(color);
        pgc.fillRect(0, 0, actualWidth, actualHeight);

        pgc.setFill(Color.BLACK); // Text color - choose one that contrasts with most placeholderColors
        pgc.setFont(Font.font("Arial", 12)); // Example font

        // Handle multi-line messages
        String[] lines = message.split("\n");
        double lineHeight = new Text("Sample").getLayoutBounds().getHeight() * 1.2; // Approximate line height

        for (int i = 0; i < lines.length; i++) {
            Text textNode = new Text(lines[i]);
            textNode.setFont(pgc.getFont()); // Ensure Text node uses the same font for measurement
            double textWidth = textNode.getLayoutBounds().getWidth();
            double textX = (actualWidth - textWidth) / 2.0;
            // Adjust Y for multiple lines, starting from a position that looks decent
            double textY = (actualHeight / 2.0) - (lines.length - 1) * lineHeight / 2.0 + i * lineHeight;
            pgc.fillText(lines[i], textX, textY);
        }

        placeholderCanvas.snapshot(null, writableImage);
        return writableImage;
    }
}
