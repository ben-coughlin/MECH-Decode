package com.example.ftcfieldsimulator;

import javafx.application.Platform;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.function.Consumer; // For callback

public class UdpPositionListener implements Runnable {

    // --- Define Data Structures for different message types ---
    public interface UdpMessageData {
        // Base interface for all message data types
        String getMessageType(); // Helper to identify the type
    }

    public static class PositionData implements UdpMessageData {
        public final double x;
        public final double y;
        public final double heading;

        public PositionData(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        @Override
        public String getMessageType() { return "pos"; }

        @Override
        public String toString() {
            return String.format("PositionData[X:%.2f, Y:%.2f, H:%.1f]", x, y, heading);
        }
    }

    public static class CircleData implements UdpMessageData {
        public final double radiusInches;
        public final double heading;

        public CircleData(double radiusInches, double heading) {
            this.radiusInches = radiusInches;
            this.heading = heading;
        }

        @Override
        public String getMessageType() { return "cir"; }

        @Override
        public String toString() {
            return String.format("CircleData[Radius:%.2f inches, Heading:%.1f deg]", radiusInches, heading);
        }
    }

    public enum LineStyle {
        SOLID_THICK(1),  // Style 1: Solid, 3px width (default color to be decided by renderer)
        SOLID_THIN(2),   // Style 2: Solid, 1.5px width (different default color by renderer)
        DOTTED(3);       // Style 3: Dotted

        private final int styleCode;
        LineStyle(int code) { this.styleCode = code; }
        public int getStyleCode() { return styleCode; }

        public static LineStyle fromCode(int code) {
            for (LineStyle style : values()) {
                if (style.styleCode == code) {
                    return style;
                }
            }
            return SOLID_THICK; // Default if code is invalid
        }
    }

    public static class LineData implements UdpMessageData {
        public final String name; // Unique identifier for the line
        public final double x0, y0, x1, y1; // Coordinates in inches
        public final LineStyle style;

        public LineData(String name, double x0, double y0, double x1, double y1, LineStyle style) {
            this.name = name;
            this.x0 = x0;
            this.y0 = y0;
            this.x1 = x1;
            this.y1 = y1;
            this.style = style;
        }

        @Override
        public String getMessageType() { return "line"; } // Message type remains "line"

        @Override
        public String toString() {
            return String.format("LineData[Name:'%s', From(%.2f,%.2f) To(%.2f,%.2f), Style:%s]",
                    name, x0, y0, x1, y1, style.name());
        }
    }

    public static class TextData implements UdpMessageData {
        public final String text;

        public TextData(String text) {
            this.text = text;
        }

        @Override
        public String getMessageType() { return "txt"; }

        @Override
        public String toString() {
            return String.format("TextData[Text:'%s']", text);
        }
    }

    private final int port;
    private volatile boolean running = true; // Use volatile for visibility across threads
    private DatagramSocket socket;
    private final Consumer<UdpMessageData> messageCallback;

    public UdpPositionListener(int port, Consumer<UdpMessageData> messageCallback) {
        this.port = port;
        this.messageCallback = messageCallback;
    }
    @Override
    public void run() {
        try {
            socket = new DatagramSocket(port);
            System.out.println("UDP Listener started on port: " + port + ". Listening for pos:, cir:, line:, txt: messages.");
            byte[] buffer = new byte[1024];

            while (running) {
                try {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);

                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();
                    System.out.println("UDP Message Received: " + rawMessage);

                    try {
                        UdpMessageData dataToSend = null;
                        String prefixPos = "pos:";
                        String prefixCir = "cir:";
                        String prefixLine = "line:";
                        String prefixTxt = "txt:";

                        if (rawMessage.startsWith(prefixPos)) {
                            String content = rawMessage.substring(prefixPos.length());
                            String[] parts = content.split(",");
                            if (parts.length == 3) {
                                dataToSend = new PositionData(
                                        Double.parseDouble(parts[0].trim()),
                                        Double.parseDouble(parts[1].trim()),
                                        Double.parseDouble(parts[2].trim()));
                            } else { System.err.println("Invalid 'pos' format: " + rawMessage); }
                        } else if (rawMessage.startsWith(prefixCir)) {
                            String content = rawMessage.substring(prefixCir.length());
                            String[] parts = content.split(",");
                            if (parts.length == 2) {
                                dataToSend = new CircleData(
                                        Double.parseDouble(parts[0].trim()),
                                        Double.parseDouble(parts[1].trim()));
                            } else { System.err.println("Invalid 'cir' format: " + rawMessage); }
                        } else if (rawMessage.startsWith(prefixLine)) {
                            String content = rawMessage.substring(prefixLine.length());
                            String[] parts = content.split(",", 6); // Split into exactly 6 parts
                            // name,x0,y0,x1,y1,style
                            if (parts.length == 6) {
                                try {
                                    String name = parts[0].trim();
                                    double x0 = Double.parseDouble(parts[1].trim());
                                    double y0 = Double.parseDouble(parts[2].trim());
                                    double x1 = Double.parseDouble(parts[3].trim());
                                    double y1 = Double.parseDouble(parts[4].trim());
                                    int styleCode = Integer.parseInt(parts[5].trim());
                                    LineStyle style = LineStyle.fromCode(styleCode);

                                    if (name.isEmpty()) {
                                        System.err.println("Invalid 'line' format: Name cannot be empty. " + rawMessage);
                                    } else {
                                        dataToSend = new LineData(name, x0, y0, x1, y1, style);
                                    }
                                } catch (NumberFormatException e) {
                                    System.err.println("Invalid number in 'line' parameters: " + rawMessage + " - " + e.getMessage());
                                }
                            } else {
                                System.err.println("Invalid 'line' format: " + rawMessage + ". Expected: name,x0,y0,x1,y1,styleCode(1-3)");
                            }
                        } else if (rawMessage.startsWith(prefixTxt)) {
                            dataToSend = new TextData(rawMessage.substring(prefixTxt.length()).trim());
                        } else {
                            System.err.println("Unknown UDP message prefix: " + rawMessage);
                        }

                        if (dataToSend != null && messageCallback != null) {
                            final UdpMessageData finalData = dataToSend;
                            Platform.runLater(() -> messageCallback.accept(finalData));
                        }
                    } catch (NumberFormatException e) {
                        System.err.println("Error parsing UDP message numbers: " + rawMessage + " - " + e.getMessage());
                    } catch (IndexOutOfBoundsException e) {
                        System.err.println("Error parsing UDP message - not enough parts: " + rawMessage + " - " + e.getMessage());
                    } catch (Exception e) {
                        System.err.println("Unexpected error parsing UDP message: " + rawMessage + " - " + e.getMessage());
                        e.printStackTrace();
                    }
                } catch (SocketException se) {
                    if (!running) { System.out.println("UDP socket closed intentionally."); }
                    else { System.err.println("SocketException in UDP Listener: " + se.getMessage()); running = false; }
                } catch (IOException e) {
                    if (running) { System.err.println("IOException in UDP Listener: " + e.getMessage()); }
                }
            }
        } catch (SocketException e) {
            System.err.println("Could not start UDP Listener on port " + port + ": " + e.getMessage());
        } finally {
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
            System.out.println("UDP Listener stopped.");
        }
    }
//    @Override
//    public void run() {
//        try {
//            socket = new DatagramSocket(port);
//            System.out.println("UDP Listener started on port: " + port + ". Listening for pos:, cir:, line:, txt: messages.");
//            byte[] buffer = new byte[1024];
//
//            while (running) {
//                try {
//                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
//                    socket.receive(packet); // Blocking call
//
//                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();
//                    System.out.println("UDP Message Received: " + rawMessage);
//
//                    // --- Parse based on message prefix ---
//                    try {
//                        UdpMessageData dataToSend = null;
//                        String prefixTxt = "txt:";
//
//                        if (rawMessage.startsWith("pos:")) {
//                            String content = rawMessage.substring("pos:".length());
//                            String[] parts = content.split(",");
//                            if (parts.length == 3) {
//                                double x = Double.parseDouble(parts[0].trim());
//                                double y = Double.parseDouble(parts[1].trim());
//                                double heading = Double.parseDouble(parts[2].trim());
//                                dataToSend = new PositionData(x, y, heading);
//                            } else {
//                                System.err.println("Invalid 'pos' message format: " + rawMessage);
//                            }
//                        } else if (rawMessage.startsWith("cir:")) {
//                            String content = rawMessage.substring("cir:".length());
//                            String[] parts = content.split(",");
//                            if (parts.length == 2) {
//                                double radius = Double.parseDouble(parts[0].trim());
//                                double heading = Double.parseDouble(parts[1].trim());
//                                dataToSend = new CircleData(radius, heading);
//                            } else {
//                                System.err.println("Invalid 'cir' message format (expected radius,heading): " + rawMessage);
//                            }
//                        } else if (rawMessage.startsWith("line:")) {
//                            String content = rawMessage.substring("line:".length());
//                            String[] parts = content.split(",");
//                            if (parts.length == 4) {
//                                double x0 = Double.parseDouble(parts[0].trim());
//                                double y0 = Double.parseDouble(parts[1].trim());
//                                double x1 = Double.parseDouble(parts[2].trim());
//                                double y1 = Double.parseDouble(parts[3].trim());
//                                dataToSend = new LineData(x0, y0, x1, y1);
//                            } else {
//                                System.err.println("Invalid 'line' message format: " + rawMessage);
//                            }
//                        } else if (rawMessage.startsWith(prefixTxt)) {
//                            String textContent = rawMessage.substring(prefixTxt.length());
//                            // No further splitting or parsing needed for the content itself, it's just a string.
//                            // You might want to trim it if leading/trailing whitespace isn't desired.
//                            dataToSend = new TextData(textContent.trim());
//                        } else {
//                            System.err.println("Unknown UDP message prefix: " + rawMessage);
//                        }
//
//                        // Safely pass the parsed data to the main application thread
//                        if (dataToSend != null && messageCallback != null) {
//                            final UdpMessageData finalData = dataToSend; // Effectively final for lambda
//                            Platform.runLater(() -> messageCallback.accept(finalData));
//                        }
//                    } catch (NumberFormatException e) {
//                        System.err.println("Error parsing UDP message numbers: " + rawMessage + " - " + e.getMessage());
//                    } catch (IndexOutOfBoundsException e) {
//                        System.err.println("Error parsing UDP message - not enough parts: " + rawMessage + " - " + e.getMessage());
//                    } catch (Exception e) { // Catch any other parsing related errors
//                        System.err.println("Unexpected error parsing UDP message: " + rawMessage + " - " + e.getMessage());
//                        e.printStackTrace(); // Good to see the full stack trace for unexpected errors
//                    }
//                } catch (SocketException se) {
//                    if (!running) {
//                        System.out.println("UDP socket closed intentionally.");
//                    } else {
//                        System.err.println("SocketException in UDP Listener (is port in use?): " + se.getMessage());
//                        running = false;
//                    }
//                } catch (IOException e) {
//                    if (running) {
//                        System.err.println("IOException in UDP Listener: " + e.getMessage());
//                    }
//                }
//            }
//        } catch (SocketException e) {
//            System.err.println("Could not start UDP Listener on port " + port + ": " + e.getMessage());
//        } finally {
//            if (socket != null && !socket.isClosed()) {
//                socket.close();
//            }
//            System.out.println("UDP Listener stopped.");
//        }
//    }

    public void stopListener() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            // Interrupting the receive() call by closing the socket from another thread
            socket.close();
        }
    }
}
