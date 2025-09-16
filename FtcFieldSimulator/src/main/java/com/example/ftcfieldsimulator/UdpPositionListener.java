package com.example.ftcfieldsimulator;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.function.Consumer;

/**
 * Listens for UDP packets on a specified port and parses them into message data objects.
 * This version is designed to parse text-based messages with prefixes (e.g., "pos:", "line:").
 */
public class UdpPositionListener implements Runnable {

    // --- Define Data Structures for different message types ---
    // This interface is the base for all our data packets.
    public interface UdpMessageData {}

    // Data for robot position
    public static class PositionData implements UdpMessageData {
        public final double x, y, heading;
        public PositionData(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    // Data for a debug circle (based on your old format)
    public static class CircleData implements UdpMessageData {
        public final double radiusInches, heading;
        public CircleData(double radiusInches, double heading) {
            this.radiusInches = radiusInches; this.heading = heading;
        }
    }

    // Data for a named, styled line
    public static class LineData implements UdpMessageData {
        public final String name;
        public final double x1, y1, x2, y2;
        public final int styleCode; // The integer style code (e.g., 1, 2, 3)

        public LineData(String name, double x1, double y1, double x2, double y2, int styleCode) {
            this.name = name;
            this.x1 = x1; this.y1 = y1;
            this.x2 = x2; this.y2 = y2;
            this.styleCode = styleCode;
        }
    }

    // Data for a text message
    public static class TextData implements UdpMessageData {
        public final String text;
        public TextData(String text) { this.text = text; }
    }

    // --- Listener Implementation ---
    private final int port;
    private volatile boolean isRunning = true;
    private DatagramSocket socket;
    private final Consumer<UdpMessageData> messageConsumer;

    public UdpPositionListener(int port, Consumer<UdpMessageData> messageConsumer) {
        this.port = port;
        this.messageConsumer = messageConsumer;
    }

    public void stopListener() {
        isRunning = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }

    @Override
    public void run() {
        try {
            socket = new DatagramSocket(port);
            System.out.println("UDP Listener started on port: " + port);
            byte[] buffer = new byte[1024];

            while (isRunning) {
                try {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet); // Blocks until a packet is received

                    if (!isRunning) break;

                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();
                    UdpMessageData parsedData = parseMessage(rawMessage);

                    if (parsedData != null) {
                        messageConsumer.accept(parsedData);
                    }
                } catch (IOException e) {
                    if (!isRunning) {
                        System.out.println("UDP listener stopping as requested.");
                    } else {
                        System.err.println("IOException in UDP listener: " + e.getMessage());
                    }
                }
            }
        } catch (SocketException e) {
            System.err.println("Could not create UDP socket on port " + port + ". Is it in use?");
        } finally {
            if (socket != null && !socket.isClosed()) socket.close();
            System.out.println("UDP Listener has shut down.");
        }
    }

    /**
     * Parses a string message with a prefix (e.g., "pos:...") into a UdpMessageData object.
     */
    private UdpMessageData parseMessage(String rawMessage) {
        if (rawMessage == null || rawMessage.isEmpty()) return null;

        try {
            if (rawMessage.startsWith("pos:")) {
                String content = rawMessage.substring(4); // 4 = length of "pos:"
                String[] parts = content.split(",");
                if (parts.length == 3) {
                    return new PositionData(
                            Double.parseDouble(parts[0].trim()),
                            Double.parseDouble(parts[1].trim()),
                            Double.parseDouble(parts[2].trim())
                    );
                }
            } else if (rawMessage.startsWith("cir:")) {
                String content = rawMessage.substring(4); // 4 = length of "cir:"
                String[] parts = content.split(",");
                if (parts.length == 2) {
                    return new CircleData(
                            Double.parseDouble(parts[0].trim()),
                            Double.parseDouble(parts[1].trim())
                    );
                }
            } else if (rawMessage.startsWith("line:")) {
                String content = rawMessage.substring(5); // 5 = length of "line:"
                String[] parts = content.split(",", 6); // name,x1,y1,x2,y2,styleCode
                if (parts.length == 6) {
                    return new LineData(
                            parts[0].trim(),
                            Double.parseDouble(parts[1].trim()),
                            Double.parseDouble(parts[2].trim()),
                            Double.parseDouble(parts[3].trim()),
                            Double.parseDouble(parts[4].trim()),
                            Integer.parseInt(parts[5].trim())
                    );
                }
            } else if (rawMessage.startsWith("txt:")) {
                String content = rawMessage.substring(4); // 4 = length of "txt:"
                return new TextData(content);
            } else {
                System.err.println("Received unknown message format: " + rawMessage);
            }
        } catch (NumberFormatException | ArrayIndexOutOfBoundsException e) {
            System.err.println("Failed to parse UDP message: '" + rawMessage + "'. Error: " + e.getMessage());
        }
        return null; // Return null if parsing fails
    }
}


//package com.example.ftcfieldsimulator;
//
//import java.io.IOException;
//import java.lang.reflect.Constructor;
//import java.lang.reflect.Method;
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.SocketException;
//import java.time.LocalTime;
//import java.util.function.Consumer;
//
///**
// * Listens for UDP packets on a specified port and parses them into message data objects.
// */
//public class UdpPositionListener implements Runnable {
//
//    // --- Data Classes for Messages ---
//    public static abstract class UdpMessageData {}
//
//    public static class PositionData extends UdpMessageData {
//        public final double x, y, heading;
//        public PositionData(double x, double y, double heading) {
//            this.x = x; this.y = y; this.heading = heading;
//        }
//    }
//
//    public static class CircleData extends UdpMessageData {
//        public final double x, y, radiusInches, heading;
//        public final String colorStr;
//        public CircleData(double x, double y, double radiusInches, double heading, String colorStr) {
//            this.x = x; this.y = y; this.radiusInches = radiusInches; this.heading = heading; this.colorStr = colorStr;
//        }
//    }
//
//    /**
//     * +++ CORRECTED CLASS DEFINITION +++
//     * Contains data to draw a named line segment, including the integer style code from the UDP packet.
//     */
//    public static class LineData extends UdpMessageData {
//        public final String name;
//        public final double x1, y1, x2, y2;
//        public final int styleCode; // The raw style code (e.g., 1, 2, 3)
//
//        public LineData(String name, double x1, double y1, double x2, double y2, int styleCode) {
//            this.name = name;
//            this.x1 = x1;
//            this.y1 = y1;
//            this.x2 = x2;
//            this.y2 = y2;
//            this.styleCode = styleCode;
//        }
//    }
//
//    public static class TextData extends UdpMessageData {
//        public final String text;
//        public TextData(String text) { this.text = text; }
//    }
//
//    // --- Listener Implementation ---
//
//    private final int port;
//    private final Consumer<UdpMessageData> messageConsumer;
//    private volatile boolean isRunning = true;
//    private DatagramSocket socket;
//
//    public UdpPositionListener(int port, Consumer<UdpMessageData> messageConsumer) {
//        this.port = port;
//        this.messageConsumer = messageConsumer;
//    }
//
//    public void stopListener() {
//        isRunning = false;
//        if (socket != null && !socket.isClosed()) {
//            socket.close(); // This will interrupt the blocking receive() call
//        }
//    }
//
//    @Override
//    public void run() {
//        try {
//            socket = new DatagramSocket(port);
//            byte[] buffer = new byte[1024];
//            System.out.println("UDP Listener started on port " + port);
//
//            while (isRunning) {
//                try {
//                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
//                    socket.receive(packet);
//
//                    if (!isRunning) break;
//
//                    String received = new String(packet.getData(), 0, packet.getLength());
//                    UdpMessageData parsedData = parseMessage(received);
//
//                    if (parsedData != null) {
//                        messageConsumer.accept(parsedData);
//                    }
//
//                } catch (IOException e) {
//                    if (!isRunning) {
//                        System.out.println("UDP listener stopping as requested.");
//                    } else {
//                        System.err.println("IOException in UDP listener: " + e.getMessage());
//                    }
//                }
//            }
//        } catch (SocketException e) {
//            System.err.println("Could not create UDP socket on port " + port + ". Is it already in use?");
//        } finally {
//            if (socket != null && !socket.isClosed()) {
//                socket.close();
//            }
//            System.out.println("UDP Listener has shut down.");
//        }
//    }
//
//    /**
//     * Parses a string message into a specific UdpMessageData object.
//     */
//    private UdpMessageData parseMessage(String message) {
//        if (message == null || message.trim().isEmpty()) {
//            return null;
//        }
//
//        try {
//            String[] parts = message.split("\\|");
//            String type = parts[0];
//
//            switch (type) {
//                case "POS": // Format: "POS|x|y|heading"
//                    return new PositionData(Double.parseDouble(parts[1]), Double.parseDouble(parts[2]), Double.parseDouble(parts[3]));
//
//                case "CIRCLE": // Format: "CIRCLE|x|y|radius|heading|color"
//                    return new CircleData(Double.parseDouble(parts[1]), Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), Double.parseDouble(parts[4]), parts[5]);
//
//                case "LINE": // Format: "LINE|name|x1|y1|x2|y2|styleCode"
//                    return new LineData(parts[1], Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), Double.parseDouble(parts[4]), Double.parseDouble(parts[5]), Integer.parseInt(parts[6]));
//
//                case "TEXT": // Format: "TEXT|the message content"
//                    return new TextData(parts[1]);
//
//                default:
//                    System.err.println("Received unknown UDP message type: " + type);
//                    return null;
//            }
//        } catch (NumberFormatException | ArrayIndexOutOfBoundsException e) {
//            System.err.println("Failed to parse UDP message: '" + message + "'. Error: " + e.getMessage());
//            return null;
//        }
//    }
//}


//package com.example.ftcfieldsimulator;
//
//import javafx.application.Platform;
//import java.io.IOException;
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.SocketException;
//import java.util.function.Consumer; // For callback
//
//public class UdpPositionListener implements Runnable {
//
//    // --- Define Data Structures for different message types ---
//    public interface UdpMessageData {
//        // Base interface for all message data types
//        String getMessageType(); // Helper to identify the type
//    }
//
//    public static class PositionData implements UdpMessageData {
//        public final double x;
//        public final double y;
//        public final double heading;
//
//        public PositionData(double x, double y, double heading) {
//            this.x = x;
//            this.y = y;
//            this.heading = heading;
//        }
//
//        @Override
//        public String getMessageType() { return "pos"; }
//
//        @Override
//        public String toString() {
//            return String.format("PositionData[X:%.2f, Y:%.2f, H:%.1f]", x, y, heading);
//        }
//    }
//
//    public static class CircleData implements UdpMessageData {
//        public final double radiusInches;
//        public final double heading;
//
//        public CircleData(double radiusInches, double heading) {
//            this.radiusInches = radiusInches;
//            this.heading = heading;
//        }
//
//        @Override
//        public String getMessageType() { return "cir"; }
//
//        @Override
//        public String toString() {
//            return String.format("CircleData[Radius:%.2f inches, Heading:%.1f deg]", radiusInches, heading);
//        }
//    }
//
//    public enum LineStyle {
//        SOLID_THICK(1),  // Style 1: Solid, 3px width (default color to be decided by renderer)
//        SOLID_THIN(2),   // Style 2: Solid, 1.5px width (different default color by renderer)
//        DOTTED(3);       // Style 3: Dotted
//
//        private final int styleCode;
//        LineStyle(int code) { this.styleCode = code; }
//        public int getStyleCode() { return styleCode; }
//
//        public static LineStyle fromCode(int code) {
//            for (LineStyle style : values()) {
//                if (style.styleCode == code) {
//                    return style;
//                }
//            }
//            return SOLID_THICK; // Default if code is invalid
//        }
//    }
//
//    public static class LineData implements UdpMessageData {
//        public final String name; // Unique identifier for the line
//        public final double x0, y0, x1, y1; // Coordinates in inches
//        public final LineStyle style;
//
//        public LineData(String name, double x0, double y0, double x1, double y1, LineStyle style) {
//            this.name = name;
//            this.x0 = x0;
//            this.y0 = y0;
//            this.x1 = x1;
//            this.y1 = y1;
//            this.style = style;
//        }
//
//        @Override
//        public String getMessageType() { return "line"; } // Message type remains "line"
//
//        @Override
//        public String toString() {
//            return String.format("LineData[Name:'%s', From(%.2f,%.2f) To(%.2f,%.2f), Style:%s]",
//                    name, x0, y0, x1, y1, style.name());
//        }
//    }
//
//    public static class TextData implements UdpMessageData {
//        public final String text;
//
//        public TextData(String text) {
//            this.text = text;
//        }
//
//        @Override
//        public String getMessageType() { return "txt"; }
//
//        @Override
//        public String toString() {
//            return String.format("TextData[Text:'%s']", text);
//        }
//    }
//
//    private final int port;
//    private volatile boolean running = true; // Use volatile for visibility across threads
//    private DatagramSocket socket;
//    private final Consumer<UdpMessageData> messageCallback;
//
//    public UdpPositionListener(int port, Consumer<UdpMessageData> messageCallback) {
//        this.port = port;
//        this.messageCallback = messageCallback;
//    }
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
//                    socket.receive(packet);
//
//                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();
//                    System.out.println("UDP Message Received: " + rawMessage);
//
//                    try {
//                        UdpMessageData dataToSend = null;
//                        String prefixPos = "pos:";
//                        String prefixCir = "cir:";
//                        String prefixLine = "line:";
//                        String prefixTxt = "txt:";
//
//                        if (rawMessage.startsWith(prefixPos)) {
//                            String content = rawMessage.substring(prefixPos.length());
//                            String[] parts = content.split(",");
//                            if (parts.length == 3) {
//                                dataToSend = new PositionData(
//                                        Double.parseDouble(parts[0].trim()),
//                                        Double.parseDouble(parts[1].trim()),
//                                        Double.parseDouble(parts[2].trim()));
//                            } else { System.err.println("Invalid 'pos' format: " + rawMessage); }
//                        } else if (rawMessage.startsWith(prefixCir)) {
//                            String content = rawMessage.substring(prefixCir.length());
//                            String[] parts = content.split(",");
//                            if (parts.length == 2) {
//                                dataToSend = new CircleData(
//                                        Double.parseDouble(parts[0].trim()),
//                                        Double.parseDouble(parts[1].trim()));
//                            } else { System.err.println("Invalid 'cir' format: " + rawMessage); }
//                        } else if (rawMessage.startsWith(prefixLine)) {
//                            String content = rawMessage.substring(prefixLine.length());
//                            String[] parts = content.split(",", 6); // Split into exactly 6 parts
//                            // name,x0,y0,x1,y1,style
//                            if (parts.length == 6) {
//                                try {
//                                    String name = parts[0].trim();
//                                    double x0 = Double.parseDouble(parts[1].trim());
//                                    double y0 = Double.parseDouble(parts[2].trim());
//                                    double x1 = Double.parseDouble(parts[3].trim());
//                                    double y1 = Double.parseDouble(parts[4].trim());
//                                    int styleCode = Integer.parseInt(parts[5].trim());
//                                    LineStyle style = LineStyle.fromCode(styleCode);
//
//                                    if (name.isEmpty()) {
//                                        System.err.println("Invalid 'line' format: Name cannot be empty. " + rawMessage);
//                                    } else {
//                                        dataToSend = new LineData(name, x0, y0, x1, y1, style);
//                                    }
//                                } catch (NumberFormatException e) {
//                                    System.err.println("Invalid number in 'line' parameters: " + rawMessage + " - " + e.getMessage());
//                                }
//                            } else {
//                                System.err.println("Invalid 'line' format: " + rawMessage + ". Expected: name,x0,y0,x1,y1,styleCode(1-3)");
//                            }
//                        } else if (rawMessage.startsWith(prefixTxt)) {
//                            dataToSend = new TextData(rawMessage.substring(prefixTxt.length()).trim());
//                        } else {
//                            System.err.println("Unknown UDP message prefix: " + rawMessage);
//                        }
//
//                        if (dataToSend != null && messageCallback != null) {
//                            final UdpMessageData finalData = dataToSend;
//                            Platform.runLater(() -> messageCallback.accept(finalData));
//                        }
//                    } catch (NumberFormatException e) {
//                        System.err.println("Error parsing UDP message numbers: " + rawMessage + " - " + e.getMessage());
//                    } catch (IndexOutOfBoundsException e) {
//                        System.err.println("Error parsing UDP message - not enough parts: " + rawMessage + " - " + e.getMessage());
//                    } catch (Exception e) {
//                        System.err.println("Unexpected error parsing UDP message: " + rawMessage + " - " + e.getMessage());
//                        e.printStackTrace();
//                    }
//                } catch (SocketException se) {
//                    if (!running) { System.out.println("UDP socket closed intentionally."); }
//                    else { System.err.println("SocketException in UDP Listener: " + se.getMessage()); running = false; }
//                } catch (IOException e) {
//                    if (running) { System.err.println("IOException in UDP Listener: " + e.getMessage()); }
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
//
//    public void stopListener() {
//        running = false;
//        if (socket != null && !socket.isClosed()) {
//            // Interrupting the receive() call by closing the socket from another thread
//            socket.close();
//        }
//    }
//}
