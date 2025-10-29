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

    // Data for a key-value pair
    public static class KeyValueData implements UdpMessageData {
        public final String key;
        public final String value;
        public KeyValueData(String key, String value) { this.key = key; this.value = value; }
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
            } else if (rawMessage.startsWith("kv:")) {
                String content = rawMessage.substring(3); // 3 = length of "kv:"
                String[] parts = content.split(",", 2); // Split only on the first comma
                if (parts.length == 2) {
                    return new KeyValueData(parts[0].trim(), parts[1].trim());
                } else if (parts.length == 1) {
                    // Handle case where value might be empty (e.g., "kv:MyKey,")
                    return new KeyValueData(parts[0].trim(), "");
                }
            } else {
                System.err.println("Received unknown message format: " + rawMessage);
            }
        } catch (NumberFormatException | ArrayIndexOutOfBoundsException e) {
            System.err.println("Failed to parse UDP message: '" + rawMessage + "'. Error: " + e.getMessage());
        }
        return null; // Return null if parsing fails
    }
}
