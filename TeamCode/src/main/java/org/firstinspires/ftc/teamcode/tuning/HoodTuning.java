package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Hood Tuning OpMode
 * Used to create/tune the launch angle lookup table
 *
 * WORKFLOW:
 * 1. Position robot at different distances from goal
 * 2. Adjust hood position and RPM until shots are accurate
 * 3. Press A to save that data point
 * 4. Repeat for multiple distances
 * 5. Copy the lookup table from telemetry into Turret.java
 */
@TeleOp(name = "Hood Tuner", group = "Tuning")
public class HoodTuning extends OpMode {

    private Turret turret;
    private Limelight limelight;

    // Current tuning values
    private double currentHoodPosition = 0.5;
    private double currentRPM = 3000;

    // Saved tuning points
    private ArrayList<TuningPoint> savedPoints = new ArrayList<>();

    // Control variables
    private final ElapsedTime buttonCooldown = new ElapsedTime();
    private static final double COOLDOWN_MS = 300;

    // Adjustment speeds
    private static final double HOOD_COARSE_STEP = 0.02;   // Stick control
    private static final double HOOD_FINE_STEP = 0.005;    // Dpad control
    private static final double RPM_COARSE_STEP = 100;      // Stick control
    private static final double RPM_FINE_STEP = 25;         // Dpad control

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("    HOOD TUNER INITIALIZED");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("Position robot at a known distance");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void start() {
        buttonCooldown.reset();
        turret.turnOnFlywheel();
    }

    @Override
    public void loop() {
        // Update subsystems
        limelight.updateLimelight();
        turret.updateTurret();

        // Get current distance from Limelight
        double currentDistance = Limelight.getDistance();

        // ═══════════════════════════════════════════════════════
        // CONTROLS - Hood Position
        // ═══════════════════════════════════════════════════════

        // Left Stick Y: Coarse hood adjustment
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            currentHoodPosition -= gamepad1.left_stick_y * HOOD_COARSE_STEP;
        }

        // Dpad Up/Down: Fine hood adjustment
        if (gamepad1.dpad_up && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            currentHoodPosition += HOOD_FINE_STEP;
            buttonCooldown.reset();
        }
        if (gamepad1.dpad_down && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            currentHoodPosition -= HOOD_FINE_STEP;
            buttonCooldown.reset();
        }

        // Clamp hood position to valid servo range
        currentHoodPosition = Math.max(0.0, Math.min(1.0, currentHoodPosition));
        turret.setHoodPos(currentHoodPosition);

        // ═══════════════════════════════════════════════════════
        // CONTROLS - RPM
        // ═══════════════════════════════════════════════════════

        // Right Stick Y: Coarse RPM adjustment
        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            currentRPM -= gamepad1.right_stick_y * RPM_COARSE_STEP;
        }

        // Dpad Left/Right: Fine RPM adjustment
        if (gamepad1.dpad_right && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            currentRPM += RPM_FINE_STEP;
            buttonCooldown.reset();
        }
        if (gamepad1.dpad_left && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            currentRPM -= RPM_FINE_STEP;
            buttonCooldown.reset();
        }

        // Clamp RPM to reasonable range
        currentRPM = Math.max(1500, Math.min(6000, currentRPM));
        turret.setFlywheelRPM(currentRPM);

        // ═══════════════════════════════════════════════════════
        // CONTROLS - Data Management
        // ═══════════════════════════════════════════════════════

        // A Button: Save current tuning point
        if (gamepad1.a && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            if (currentDistance > 0) {  // Only save if we have valid distance
                savedPoints.add(new TuningPoint(currentDistance, currentHoodPosition, currentRPM));
                // Sort by distance
                savedPoints.sort((p1, p2) -> Double.compare(p1.distance, p2.distance));
                buttonCooldown.reset();
            }
        }

        // X Button: Clear all saved points
        if (gamepad1.x && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            savedPoints.clear();
            buttonCooldown.reset();
        }

        // B Button: Remove last saved point
        if (gamepad1.b && buttonCooldown.milliseconds() > COOLDOWN_MS) {
            if (!savedPoints.isEmpty()) {
                savedPoints.remove(savedPoints.size() - 1);
            }
            buttonCooldown.reset();
        }

        // ═══════════════════════════════════════════════════════
        // TELEMETRY
        // ═══════════════════════════════════════════════════════

        telemetry.addLine("═══════════════════════════════════════════════════════");
        telemetry.addLine("                   HOOD TUNER");
        telemetry.addLine("═══════════════════════════════════════════════════════");
        telemetry.addLine();

        // Current readings
        telemetry.addLine("───────────── CURRENT VALUES ─────────────");
        telemetry.addData(" Distance", "%.1f inches", currentDistance);
        telemetry.addData(" Hood Position", "%.3f", currentHoodPosition);
        telemetry.addData(" Target RPM", "%.0f", currentRPM);
        telemetry.addData(" Current RPM", "%.0f", turret.getCurrentFlywheelRPM());
        telemetry.addData(" Flywheel Ready?", turret.isFlywheelReady() ? "YES" : "NO");
        telemetry.addLine();

        // Controls
        telemetry.addLine("──────────────── CONTROLS ────────────────");
        telemetry.addLine("HOOD POSITION:");
        telemetry.addLine("  • Left Stick Y:  Coarse adjust (±0.02)");
        telemetry.addLine("  • DPad Up/Down:  Fine adjust (±0.005)");
        telemetry.addLine();
        telemetry.addLine("RPM:");
        telemetry.addLine("  • Right Stick Y:    Coarse adjust (±100)");
        telemetry.addLine("  • DPad Left/Right:  Fine adjust (±25)");
        telemetry.addLine();
        telemetry.addLine("DATA MANAGEMENT:");
        telemetry.addLine("  • A Button:  Save current point");
        telemetry.addLine("  • B Button:  Delete last point");
        telemetry.addLine("  • X Button:  Clear all points");
        telemetry.addLine();

        // Saved points
        telemetry.addLine("──────────── SAVED POINTS (" + savedPoints.size() + ") ────────────");
        if (savedPoints.isEmpty()) {
            telemetry.addLine("  No points saved yet");
            telemetry.addLine("  Adjust hood and RPM, then press A to save");
        } else {
            for (int i = 0; i < savedPoints.size(); i++) {
                TuningPoint p = savedPoints.get(i);
                telemetry.addLine(String.format(Locale.US,
                        "  %d: %.1f\" → Hood:%.3f RPM:%.0f",
                        i + 1, p.distance, p.hoodPos, p.rpm));
            }
        }
        telemetry.addLine();

        // Lookup table code output
        if (!savedPoints.isEmpty()) {
            telemetry.addLine("─────── COPY THIS TO Turret.java ─────────");
            telemetry.addLine("private final double[][] launchAngleLookupTable = {");
            for (int i = 0; i < savedPoints.size(); i++) {
                TuningPoint p = savedPoints.get(i);
                String comma = (i < savedPoints.size() - 1) ? "," : "";
                telemetry.addLine(String.format(Locale.US,
                        "    { %.0f, %.2f, %.0f }%s  // %.1f inches",
                        p.distance, p.hoodPos, p.rpm, comma, p.distance));
            }
            telemetry.addLine("};");
            telemetry.addLine("──────────────────────────────────────────");
        }

        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════════════════════════════");

        telemetry.update();
    }

    @Override
    public void stop() {
        turret.turnOffFlywheel();
    }

    /**
     * Helper class to store a tuning data point
     */
    private static class TuningPoint {
        double distance;
        double hoodPos;
        double rpm;

        TuningPoint(double distance, double hoodPos, double rpm) {
            this.distance = distance;
            this.hoodPos = hoodPos;
            this.rpm = rpm;
        }
    }
}