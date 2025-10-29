// Create this file: UdpPlotListener.java
package com.example.ftcfieldsimulator;

import javafx.application.Platform;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class UdpPlotListener implements Runnable {

    public static final int DEFAULT_PLOT_LISTENER_PORT = 7778;

    private final int port;
    private volatile boolean running = true;
    private DatagramSocket socket;
    private final Consumer<PlotDataEvent> eventConsumer;

    public UdpPlotListener(int port, Consumer<PlotDataEvent> eventConsumer) throws SocketException {
        this.port = port;
        this.eventConsumer = eventConsumer;
        this.socket = new DatagramSocket(port);
        System.out.println("UDP Plot Listener initialized on port: " + port);
    }

    public UdpPlotListener(Consumer<PlotDataEvent> eventConsumer) throws SocketException {
        this(DEFAULT_PLOT_LISTENER_PORT, eventConsumer);
    }

    public void stopListener() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
        System.out.println("UDP Plot Listener stopping.");
    }

    @Override
    public void run() {
        byte[] buffer = new byte[1024];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        System.out.println("UDP Plot Listener started on port " + port + " and waiting for messages...");

        while (running && socket != null && !socket.isClosed()) {
            try {
                socket.receive(packet);
                String received = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
                parseAndConsume(received);
            } catch (SocketException se) {
                if (!running) { // Expected exception when stopping
                    System.out.println("UDP Plot Listener socket closed (expected during stop).");
                } else {
                    System.err.println("UDP Plot Listener SocketException: " + se.getMessage());
                    running = false;
                }
            } catch (IOException e) {
                if (running) System.err.println("UDP Plot Listener IOException: " + e.getMessage());
            } catch (Exception e) {
                System.err.println("Error processing UDP plot message: '" + new String(packet.getData(), 0, packet.getLength()) + "'. Error: " + e.getMessage());
            }
        }
        System.out.println("UDP Plot Listener thread finished.");
    }

    /**
     * Parses messages in the space-delimited format (e.g., "LINE 12345 1 50.0").
     * This parser is robust and handles quoted strings for text arguments.
     */
    private void parseAndConsume(String message) {
        if (message == null || message.trim().isEmpty()) return;

        // Regex to find the first word (the command)
        Pattern commandPattern = Pattern.compile("^(\\S+)\\s(.*)$");
        // Regex to find subsequent arguments, respecting quoted strings
        Pattern argsPattern = Pattern.compile("\"([^\"]*)\"|\\S+");

        Matcher commandMatcher = commandPattern.matcher(message.trim());
        if (!commandMatcher.matches()) {
            System.err.println("Malformed plot message (no command/args): " + message);
            return;
        }

        String type = commandMatcher.group(1).toUpperCase();
        String argsStr = commandMatcher.group(2);

        Matcher argsMatcher = argsPattern.matcher(argsStr);
        List<String> args = new ArrayList<>();
        while (argsMatcher.find()) {
            // If group 1 is not null, it's a quoted string. Otherwise, it's a regular word.
            args.add(argsMatcher.group(1) != null ? argsMatcher.group(1) : argsMatcher.group());
        }

        if (args.isEmpty()) {
            System.err.println("Malformed plot message (command only, no timestamp): " + message);
            return;
        }

        try {
            long timestamp = Long.parseLong(args.get(0));
            PlotDataEvent event = null;

            switch (type) {
                case "POINT":
                    if (args.size() >= 3) event = new PlotPointEvent(timestamp, Double.parseDouble(args.get(2)), Integer.parseInt(args.get(1)));
                    break;
                case "LINE":
                    if (args.size() >= 3) event = new PlotLineEvent(timestamp, Double.parseDouble(args.get(2)), Integer.parseInt(args.get(1)));
                    break;
                case "POINT2":
                    if (args.size() >= 3) event = new PlotPoint2Event(timestamp, Double.parseDouble(args.get(2)), Integer.parseInt(args.get(1)));
                    break;
                case "LINE2":
                    if (args.size() >= 3) event = new PlotLine2Event(timestamp, Double.parseDouble(args.get(2)), Integer.parseInt(args.get(1)));
                    break;
                case "KV":
                    if (args.size() >= 3) event = new PlotKeyValueEvent(timestamp, args.get(1), args.get(2));
                    break;
                case "MARKER":
                    if (args.size() >= 3) event = new PlotTextAnnotationEvent(timestamp, args.get(2), args.get(1));
                    break;
                case "YLIMITS":
                    // Format: YLIMITS ts min max
                    if (args.size() >= 3) event = new PlotYLimitsEvent(timestamp, Double.parseDouble(args.get(2)), Double.parseDouble(args.get(1)));
                    break;
                case "YUNITS":
                    if (args.size() >= 2) event = new PlotYUnitsEvent(timestamp, args.get(1));
                    break;
                case "YLIMITS2":
                    if (args.size() >= 3) event = new PlotYLimits2Event(timestamp, Double.parseDouble(args.get(2)), Double.parseDouble(args.get(1)));
                    break;
                case "YUNITS2":
                    if (args.size() >= 2) event = new PlotYUnits2Event(timestamp, args.get(1));
                    break;
                default:
                    System.err.println("Unknown plot command: " + type);
            }

            if (event != null) {
                // Pass the created event to the consumer (PlotDisplay) on the JavaFX thread
                final PlotDataEvent finalEvent = event;
                Platform.runLater(() -> eventConsumer.accept(finalEvent));
            } else {
                System.err.println("Malformed plot message (incorrect arg count for " + type + "): " + message);
            }
        } catch (NumberFormatException ex) {
            System.err.println("Malformed plot message (number format error): " + message);
        } catch (Exception ex) {
            System.err.println("Generic error parsing plot message '" + message + "': " + ex.getMessage());
        }
    }
}
