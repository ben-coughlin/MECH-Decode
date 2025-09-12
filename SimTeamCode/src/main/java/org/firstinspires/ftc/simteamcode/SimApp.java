package org.firstinspires.ftc.simteamcode;

public class SimApp extends AutoSummerPinPoint {
    private static final String SIMULATOR_HOST = "localhost";
    private static final int SIMULATOR_PORT = 7777;

    public static UdpClientFieldSim clientSim;

    public static void initializeSimulatorClient() {
        if (clientSim == null) { // Initialize only if not already done
            clientSim = new UdpClientFieldSim(SIMULATOR_HOST, SIMULATOR_PORT);

            if (clientSim.isInitialized()) {
                System.out.println("[MyMainApplication] clientSim initialized to " + SIMULATOR_HOST + ":" + SIMULATOR_PORT);
                // clientSim.sendText("MyMainApplication Debug Client Online via init method");
            } else {
                System.err.println("[MyMainApplication] Failed to initialize clientSim to " + SIMULATOR_HOST + ":" + SIMULATOR_PORT + ". UDP Debug will be affected.");
                // clientSim will be non-null but its isInitialized() will be false
            }
        } else {
            System.out.println("[MyMainApplication] clientSim was already initialized.");
        }
    }

    public static void main(String[] args) {
        System.out.println("Starting SimApp...");

        initializeSimulatorClient();

        SimApp sim = new SimApp();

        sim.init();

        // TODO: usually will be called until START is pressed
        sim.init_loop();
        sim.init_loop();

        sim.start();

        System.out.println("\nStarting main simulation loop (1000 iterations)...");
        long mainLoopStartTime = System.currentTimeMillis(); // Using System's time for this outer loop timing

        for (int i = 0; i < 1000; i++) {
            System.out.println("-------------- Main Loop Iteration: " + (i + 1)); // Print 1 to 1000
            sim.mainLoop();
             try {
                 Thread.sleep(70); // e.g., sleep for 20 milliseconds
             } catch (InterruptedException e) {
                 Thread.currentThread().interrupt();
                 System.out.println("Loop interrupted.");
                 break;
             }
        }
        long mainLoopEndTime = System.currentTimeMillis();
        System.out.println("\nFinished 1000 main simulation loop iterations.");
        System.out.println("Total time for main loop iterations: " + (mainLoopEndTime - mainLoopStartTime) + " ms");

        System.out.println("SimApp finished.");

        if (clientSim != null && clientSim.isInitialized()) {
            // clientSim.sendText("MyMainApplication Debug Client Shutting Down");
            clientSim.close();
            System.out.println("[MyMainApplication] clientSim closed.");
        } else if (clientSim != null) {
            clientSim.close(); // Safe to call
            System.out.println("[MyMainApplication] Attempted to close clientSim (was not initialized or already closed).");
        }
    }
}
