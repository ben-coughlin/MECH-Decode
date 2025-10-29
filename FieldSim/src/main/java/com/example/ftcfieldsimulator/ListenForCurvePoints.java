package com.example.ftcfieldsimulator;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
// No need for Collections or List if only using ArrayList and not returning List interface type

public class ListenForCurvePoints {

    // --- Configuration ---
    private static final int UDP_PORT = 6666;
    private static final String CURVE_POINT_PREFIX = "curve_point:";
    private static final String END_MESSAGE = "end";
    private static final int BUFFER_SIZE = 1024;
    public static final int TIMEOUT_MILLISECONDS = 10000; // 10 seconds for activity timeout
    private static final int SOCKET_RECEIVE_TIMEOUT_MS = 500; // Shorter timeout for responsive loop

    // --- Singleton Implementation ---
    private static volatile ListenForCurvePoints instance;

    private ListenForCurvePoints() {
        // Private constructor

    }

    // Double-checked locking for thread-safe singleton initialization
    public static ListenForCurvePoints getInstance() {
        if (instance == null) {
            synchronized (ListenForCurvePoints.class) {
                if (instance == null) {
                    instance = new ListenForCurvePoints();
                }
            }
        }
        return instance;
    }
    // --- End Singleton Implementation ---

    // --- State Variables ---
    private volatile ListenStatus currentStatus = ListenStatus.NOT_STARTED;
    // Use a final ArrayList and clear/addAll for thread safety when returning copies.
    private final ArrayList<CurvePoint> collectedPoints = new ArrayList<>();
    private volatile String internalErrorMessage = null;
    private volatile long lastPacketReceivedTimeMs;

    private Thread listenerThread;
    private DatagramSocket socket; // Keep socket as a field to close it properly

    /**
     * Starts the UDP listening process on a new thread.
     * If already listening, it will attempt to stop the current listener and start a new one.
     * This method is non-blocking.
     */
    public synchronized void startListening() {
        if (currentStatus == ListenStatus.LISTENING) {
            System.out.println("Listener: Already active. Stopping previous and starting new.");
            // Ensure previous thread is fully stopped and resources released before starting new
            // This stopListening might take a moment.
            stopListeningInternal(true); // true to indicate it's an internal restart
        }

        // Reset state for a new session
        // Synchronize modifications to collectedPoints if checkStatusAndGetPoints could be called during this.
        // However, startListening is synchronized, so direct modification here is fine.
        collectedPoints.clear();
        internalErrorMessage = null;
        currentStatus = ListenStatus.LISTENING; // Set status before starting thread
        lastPacketReceivedTimeMs = System.currentTimeMillis(); // Initialize for overall timeout

        listenerThread = new Thread(this::runListeningLoop, "CurvePointListenerThread");
        listenerThread.setDaemon(true); // Allow JVM to exit if this is the only thread
        listenerThread.start();
        System.out.println("Listener: Curve point listener thread started.");
    }

    private void runListeningLoop() {
        DatagramSocket localSocket = null; // Use a local variable for the socket in this thread
        try {
            localSocket = new DatagramSocket(UDP_PORT);
            localSocket.setSoTimeout(SOCKET_RECEIVE_TIMEOUT_MS);
            this.socket = localSocket; // Assign to class field for external closing by stopListening()

            System.out.println("Listener: Listening for curve points on UDP port: " + UDP_PORT +
                    ", activity timeout: " + (TIMEOUT_MILLISECONDS / 1000) + "s, " +
                    "socket poll: " + SOCKET_RECEIVE_TIMEOUT_MS + "ms");

            byte[] buffer = new byte[BUFFER_SIZE];

            while (currentStatus == ListenStatus.LISTENING) {
                if (Thread.currentThread().isInterrupted()) {
                    System.out.println("Listener: Thread interrupted, exiting loop.");
                    currentStatus = ListenStatus.STOPPED;
                    break;
                }

                if (System.currentTimeMillis() - lastPacketReceivedTimeMs > TIMEOUT_MILLISECONDS) {
                    System.out.println("Listener: Activity timeout reached (" + (TIMEOUT_MILLISECONDS / 1000) + "s).");
                    currentStatus = ListenStatus.TIMED_OUT;
                    break;
                }

                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                try {
                    localSocket.receive(packet);
                    lastPacketReceivedTimeMs = System.currentTimeMillis();
                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();
                    System.out.println("Listener: Received UDP: " + rawMessage);

                    if (END_MESSAGE.equalsIgnoreCase(rawMessage)) {
                        System.out.println("Listener: End message received.");
                        currentStatus = ListenStatus.COMPLETED_SUCCESSFULLY;
                        break;
                    } else if (rawMessage.startsWith(CURVE_POINT_PREFIX)) {
                        CurvePoint point = parseCurvePointMessage(rawMessage);
                        if (point != null) {
                            synchronized (collectedPoints) { // Synchronize writing to shared list
                                collectedPoints.add(point);
                            }
                            System.out.println("Listener: Added curve point: x=" + point.x + ", y=" + point.y);
                        }
                    } else {
                        System.err.println("Listener: Received unknown message format (ignoring): " + rawMessage);
                    }

                } catch (SocketTimeoutException e) {
                    // Expected, allows loop to check status and timeout
                } catch (IOException e) {
                    if (currentStatus == ListenStatus.LISTENING) {
                        System.err.println("Listener: IOException during UDP receive: " + e.getMessage());
                        internalErrorMessage = "IOException: " + e.getMessage();
                        currentStatus = ListenStatus.ERROR;
                    }
                    // For most IOExceptions (e.g. socket closed by stopListening), break the loop.
                    // If socket.isClosed() is true, it means stopListening might have closed it.
                    if (localSocket.isClosed() && currentStatus != ListenStatus.LISTENING) {
                        System.out.println("Listener: Socket closed, likely by stopListening(). Current status: " + currentStatus);
                    }
                    break;
                }
            }
        } catch (SocketException e) {
            if (currentStatus == ListenStatus.LISTENING) { // Check if an error wasn't already set
                System.err.println("Listener: Could not create or bind UDP socket on port " + UDP_PORT + ". Error: " + e.getMessage());
                internalErrorMessage = "SocketException: " + e.getMessage();
                currentStatus = ListenStatus.ERROR;
            }
        } finally {
            this.socket = null; // Clear class field reference
            if (localSocket != null && !localSocket.isClosed()) {
                localSocket.close();
                System.out.println("Listener: UDP socket closed in runListeningLoop finally block.");
            }
            // If the loop exited while status was still LISTENING (e.g., due to thread interruption not caught above)
            // or if stopListening changed status to STOPPED.
            if (currentStatus == ListenStatus.LISTENING) {
                currentStatus = ListenStatus.STOPPED; // Ensure it's not stuck in LISTENING
            }
            System.out.println("Listener: Listening loop finished with status: " + currentStatus);
        }
    }

    /**
     * Checks the current status of the listener and retrieves collected points if available.
     * This method is non-blocking and designed to be called repeatedly from a loop.
     *
     * @return A ListenResult object containing the current status, collected points, and any error message.
     */
    public ListenResult checkStatusAndGetPoints() {
        ListenStatus statusSnapshot;
        ArrayList<CurvePoint> pointsSnapshot;
        String errorSnapshot;

        synchronized (this) { // Synchronize to get a consistent snapshot of state
            statusSnapshot = this.currentStatus;
            errorSnapshot = this.internalErrorMessage;
            synchronized (collectedPoints) { // Synchronize access to the shared list
                pointsSnapshot = new ArrayList<>(this.collectedPoints); // Create a copy
            }
        }
        return new ListenResult(statusSnapshot, pointsSnapshot, errorSnapshot);
    }

    /**
     * Public method to stop the listening process.
     */
    public synchronized void stopListening() {
        stopListeningInternal(false);
    }

    /**
     * Internal method to stop the listening process, with a flag for context.
     * @param internalCall true if called as part of an internal restart (e.g. by startListening)
     */
    private void stopListeningInternal(boolean internalCall) {
        // Only proceed if actually listening or thread seems alive
        if (currentStatus == ListenStatus.LISTENING || (listenerThread != null && listenerThread.isAlive())) {
            if (!internalCall) { // Don't print "Attempting to stop" if it's an internal restart
                System.out.println("Listener: Attempting to stop listener thread...");
            }
            currentStatus = ListenStatus.STOPPED; // Signal the loop to terminate

            DatagramSocket s = this.socket; // Local reference for thread safety
            if (s != null && !s.isClosed()) {
                s.close(); // This should interrupt a blocking receive()
                if (!internalCall) System.out.println("Listener: Socket closed by stopListening().");
            }

            if (listenerThread != null && listenerThread.isAlive()) {
                listenerThread.interrupt(); // Interrupt the thread
                try {
                    // Short join timeout. The loop should exit quickly due to status change or socket close.
                    listenerThread.join(SOCKET_RECEIVE_TIMEOUT_MS + 200);
                } catch (InterruptedException e) {
                    System.err.println("Listener: Interrupted while waiting for listener thread to stop.");
                    Thread.currentThread().interrupt(); // Preserve interrupt status
                }
                if (listenerThread.isAlive()) {
                    if (!internalCall) System.err.println("Listener: Listener thread did not stop in the expected time.");
                } else {
                    if (!internalCall) System.out.println("Listener: Listener thread stopped.");
                }
            }
        } else {
            // If not actively listening, but perhaps in an error state or already stopped
            if (currentStatus != ListenStatus.NOT_STARTED && currentStatus != ListenStatus.STOPPED) {
                // If it's in TIMED_OUT, COMPLETED_SUCCESSFULLY, ERROR - just ensure it's marked as STOPPED if a stop is explicitly called.
                // However, we generally want to preserve the terminal state (TIMED_OUT, etc.) unless reset.
                // For now, if stopListening is called, it means user intends to stop whatever state it's in.
                if (currentStatus != ListenStatus.ERROR && // Don't override an error state with STOPPED unless reset
                        currentStatus != ListenStatus.TIMED_OUT &&
                        currentStatus != ListenStatus.COMPLETED_SUCCESSFULLY) {
                    currentStatus = ListenStatus.STOPPED;
                }
            }
        }
        listenerThread = null; // Clear the thread reference
        this.socket = null; // Clear the class field socket reference
    }


    /**
     * Resets the listener to its initial state, stopping any active listening.
     */
    public synchronized void reset() {
        stopListeningInternal(true); // Stop silently as part of reset
        synchronized (collectedPoints) {
            collectedPoints.clear();
        }
        internalErrorMessage = null;
        currentStatus = ListenStatus.NOT_STARTED;
        System.out.println("Listener: Reset complete.");
    }

    private CurvePoint parseCurvePointMessage(String message) {
        if (!message.startsWith(CURVE_POINT_PREFIX)) {
            System.err.println("Listener Parser: Called with wrong prefix: " + message);
            return null;
        }
        String content = message.substring(CURVE_POINT_PREFIX.length());
        String[] parts = content.split(",");

        if (parts.length == 8) {
            try {
                double x = Double.parseDouble(parts[0].trim());
                double y = Double.parseDouble(parts[1].trim());
                double moveSpeed = Double.parseDouble(parts[2].trim());
                double turnSpeed = Double.parseDouble(parts[3].trim());
                double followDistance = Double.parseDouble(parts[4].trim());
                double pointLength = Double.parseDouble(parts[5].trim());
                double slowDownTurnRadians = Double.parseDouble(parts[6].trim());
                double slowDownTurnAmount = Double.parseDouble(parts[7].trim());

                return new CurvePoint(x, y, moveSpeed, turnSpeed,
                        followDistance, pointLength,
                        slowDownTurnRadians, slowDownTurnAmount);

            } catch (NumberFormatException e) {
                System.err.println("Listener Parser: Malformed curve point data (ignoring point). Msg: '" + message + "'. Error: " + e.getMessage());
                return null;
            }
        } else {
            System.err.println("Listener Parser: Malformed curve point message (ignoring point). Expected 8 data parts, got " + parts.length + ": '" + message + "'");
            return null;
        }
    }
}


//package com.example.ftcfieldsimulator; // Assuming same package for CurvePoint
//
//import java.io.IOException;
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.SocketException;
//import java.net.SocketTimeoutException;
//import java.util.ArrayList;
//// Removed java.util.List import as we're explicitly using ArrayList for return type
//
//public class ListenForCurvePoints {
//
//    private static final int UDP_PORT = 6666;
//    private static final String CURVE_POINT_PREFIX = "curve_point:";
//    private static final String END_MESSAGE = "end";
//    private static final int BUFFER_SIZE = 1024;
//    private static final int TIMEOUT_MILLISECONDS = 10000; // 10 seconds
//
//    // --- Singleton Implementation ---
//    private static ListenForCurvePoints instance;
//
//    private ListenForCurvePoints() {
//        // Private constructor to prevent external instantiation
//    }
//
//    public static synchronized ListenForCurvePoints getInstance() {
//        if (instance == null) {
//            instance = new ListenForCurvePoints();
//        }
//        return instance;
//    }
//    // --- End Singleton Implementation ---
//
//
//    /**
//     * Listens for UDP packets to collect CurvePoint data until an "end" message is received
//     * or a timeout occurs.
//     * This method is blocking.
//     *
//     * @return An ArrayList of CurvePoint objects received. Returns an empty list if a timeout
//     *         occurs before the "end" message, if "end" is received before any valid points,
//     *         or if a critical socket error occurs during setup.
//     */
//    public ArrayList<CurvePoint> receiveCurvePoints() {
//        ArrayList<CurvePoint> curvePoints = new ArrayList<>();
//        DatagramSocket socket = null;
//
//        try {
//            socket = new DatagramSocket(UDP_PORT);
//            socket.setSoTimeout(TIMEOUT_MILLISECONDS); // Set the timeout for socket.receive()
//            System.out.println("Listening for curve points on UDP port: " + UDP_PORT + " with a " + (TIMEOUT_MILLISECONDS / 1000) + "s timeout.");
//            byte[] buffer = new byte[BUFFER_SIZE];
//
//            long startTime = System.currentTimeMillis(); // For overall timeout logic, though setSoTimeout is primary
//
//            while (true) {
//                // Check overall timeout in case setSoTimeout doesn't cover all blocking scenarios
//                // or if we wanted a loop-based timeout (setSoTimeout is generally better for receive)
//                // if (System.currentTimeMillis() - startTime > TIMEOUT_MILLISECONDS) {
//                //     System.out.println("Overall listening timeout reached after " + (TIMEOUT_MILLISECONDS / 1000) + " seconds. No 'end' message.");
//                //     break; // Exit loop, will return current (possibly empty) list
//                // }
//
//                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
//                try {
//                    socket.receive(packet); // Blocks until a packet is received or timeout
//
//                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();
//                    System.out.println("Received UDP: " + rawMessage); // For debugging
//
//                    if (END_MESSAGE.equalsIgnoreCase(rawMessage)) {
//                        System.out.println("End message received. Finished collecting curve points.");
//                        break; // Exit the listening loop, return collected points
//                    } else if (rawMessage.startsWith(CURVE_POINT_PREFIX)) {
//                        CurvePoint point = parseCurvePointMessage(rawMessage);
//                        if (point != null) {
//                            curvePoints.add(point);
//                            System.out.println("Added curve point: x=" + point.x + ", y=" + point.y);
//                        }
//                    } else {
//                        System.err.println("Received unknown message format (ignoring): " + rawMessage);
//                    }
//                    // Reset start time after each successful packet to make timeout per-packet based if desired.
//                    // For now, setSoTimeout handles the "no data" scenario.
//                    // startTime = System.currentTimeMillis();
//
//                } catch (SocketTimeoutException e) {
//                    // This is the specific exception for socket.receive() timeout
//                    System.out.println("Socket receive timeout after " + (TIMEOUT_MILLISECONDS / 1000) + " seconds of inactivity. No 'end' message.");
//                    // Do not clear curvePoints here; return what has been collected so far or an empty list if nothing.
//                    break; // Exit loop, will return current (possibly empty) list
//                }
//                catch (IOException e) {
//                    System.err.println("IOException during UDP receive or processing (ignoring packet): " + e.getMessage());
//                    if (socket.isClosed()) { // If socket got closed by some other means
//                        System.err.println("Socket was closed externally. Terminating listening.");
//                        return new ArrayList<>(); // Return empty list on critical socket failure
//                    }
//                    // Continue listening for the next packet if it's a non-fatal IO error
//                }
//            }
//        } catch (SocketException e) {
//            System.err.println("Could not create or bind UDP socket on port " + UDP_PORT + ". Error: " + e.getMessage());
//            // Return an empty list as per requirement for critical errors
//            return new ArrayList<>();
//        } finally {
//            if (socket != null && !socket.isClosed()) {
//                socket.close();
//                System.out.println("UDP socket closed.");
//            }
//        }
//        // If loop was broken by timeout or "end", this returns the collected points.
//        // If an early critical error (like SocketException in setup), it would have returned an empty list already.
//        return curvePoints;
//    }
//
//    private CurvePoint parseCurvePointMessage(String message) {
//        // Expected format: curve_point:<x>,<y>,<moveSpeed>,<turnSpeed>,<followDistance>,<pointLength>,<slowDownTurnRadians>,<slowDownTurnAmount>
//        // Total 8 parts
//        if (!message.startsWith(CURVE_POINT_PREFIX)) { // Defensive check
//            System.err.println("Parser called with wrong prefix (should not happen if called correctly): " + message);
//            return null;
//        }
//        String content = message.substring(CURVE_POINT_PREFIX.length());
//        String[] parts = content.split(",");
//
//        if (parts.length == 8) {
//            try {
//                double x = Double.parseDouble(parts[0].trim());
//                double y = Double.parseDouble(parts[1].trim());
//                double moveSpeed = Double.parseDouble(parts[2].trim());
//                double turnSpeed = Double.parseDouble(parts[3].trim());
//                double followDistance = Double.parseDouble(parts[4].trim());
//                double pointLength = Double.parseDouble(parts[5].trim());
//                double slowDownTurnRadians = Double.parseDouble(parts[6].trim());
//                double slowDownTurnAmount = Double.parseDouble(parts[7].trim());
//
//                // +++ Use the CurvePoint constructor +++
//                return new CurvePoint(x, y, moveSpeed, turnSpeed,
//                        followDistance, pointLength,
//                        slowDownTurnRadians, slowDownTurnAmount);
//
//            } catch (NumberFormatException e) {
//                System.err.println("Malformed curve point data (ignoring point). Error parsing number: '" + message + "'. Error: " + e.getMessage());
//                return null;
//            }
//        } else {
//            System.err.println("Malformed curve point message (ignoring point). Expected 8 data parts, got " + parts.length + ": '" + message + "'");
//            return null;
//        }
//    }
//}
