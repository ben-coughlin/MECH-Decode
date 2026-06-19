package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@Disabled
@TeleOp(name = "Hood Tuning")
public class HoodTuning extends LinearOpMode {

    // ── Tuning increments ──────────────────────────────────────────────────
    private static final double SERVO_INCREMENT          = 0.005;
    private static final double MOTOR_INCREMENT          = 50.0;

    // ── Flywheel readiness thresholds ──────────────────────────────────────
    // Flywheel must be within this many RPM of the target...
    private static final double RPM_READY_THRESHOLD_RPM  = 50.0;
    // ...and hold that for this long before a shot is allowed.
    private static final double FLYWHEEL_SETTLE_MS        = 500.0;

    // ── Distance stability ─────────────────────────────────────────────────
    // Rolling window size and max peak-to-peak spread to call distance "stable"
    private static final int    DISTANCE_SAMPLE_COUNT     = 10;
    private static final double DISTANCE_STABLE_IN        = 1.0;   // inches

    // ── State ──────────────────────────────────────────────────────────────
    private double  currentServoPosition = 0.5;
    private double  rpm                  = 2000.0;
    private boolean flywheelActive       = false;

    // ── Distance rolling average ───────────────────────────────────────────
    private final double[] distanceSamples = new double[DISTANCE_SAMPLE_COUNT];
    private int     sampleIndex      = 0;
    private boolean samplesPopulated = false;

    // ── Shot tracking ──────────────────────────────────────────────────────
    private int    shotsAttempted    = 0;
    private int    shotsScored       = 0;
    private double lastShotDistance  = 0;
    private double lastShotHood      = 0;
    private double lastShotRPM       = 0;

    // ── Flywheel settle timer ──────────────────────────────────────────────
    private final ElapsedTime flywheelSettleTimer = new ElapsedTime();
    private boolean flywheelReady = false;

    // ── Drivetrain ─────────────────────────────────────────────────────────
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // ── Button debounce flags ──────────────────────────────────────────────
    private boolean dpadUpPressed      = false;
    private boolean dpadDownPressed    = false;
    private boolean dpadLeftPressed    = false;
    private boolean dpadRightPressed   = false;
    private boolean circlePressed      = false;
    private boolean crossPressed       = false;
    private boolean trianglePressed    = false;
    private boolean rBumperPressed     = false;
    private boolean optionsPressed     = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight   limelight   = new Limelight(hardwareMap);
        Servo       launchServo = hardwareMap.get(Servo.class, "hood");
        Turret      turret      = new Turret(hardwareMap);
        Intake      intake      = new Intake(hardwareMap);
        Transfer    transfer    = new Transfer(hardwareMap);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        for (double d : distanceSamples) d = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        flywheelSettleTimer.reset();

        while (opModeIsActive()) {

            // ── Subsystem updates ──────────────────────────────────────────
            transfer.updateTransfer();
            colorSensor.updateDetection();
            limelight.updateLimelight();

            // ── Drive ──────────────────────────────────────────────────────
            double y  = -gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x * 1.1;
            double rx =  gamepad1.right_stick_x;
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            leftFront .setPower((y + x + rx) / denom);
            leftBack  .setPower((y - x - rx) / denom);
            rightFront.setPower((y - x + rx) / denom);
            rightBack .setPower((y + x - rx) / denom);

            // ── Distance rolling average ───────────────────────────────────
            LLResult result       = Limelight.currResult;
            boolean  hasVision    = result != null && result.isValid();
            double   rawDistance  = limelight.getDistanceToTag(result);

            distanceSamples[sampleIndex % DISTANCE_SAMPLE_COUNT] = rawDistance;
            sampleIndex++;
            if (sampleIndex >= DISTANCE_SAMPLE_COUNT) samplesPopulated = true;

            int    validCount   = samplesPopulated ? DISTANCE_SAMPLE_COUNT : sampleIndex;
            double avgDistance  = 0;
            for (int i = 0; i < validCount; i++) avgDistance += distanceSamples[i];
            avgDistance /= Math.max(validCount, 1);

            double variance = 0;
            for (int i = 0; i < validCount; i++)
                variance = Math.max(variance, Math.abs(distanceSamples[i] - avgDistance));
            boolean distanceStable = hasVision && samplesPopulated && variance < DISTANCE_STABLE_IN;

            // ── Flywheel readiness ─────────────────────────────────────────
            double  realRPM  = turret.getRealRPM();
            double  rpmError = Math.abs(realRPM - rpm);
            boolean wasReady = flywheelReady;
            flywheelReady = flywheelActive && rpmError < RPM_READY_THRESHOLD_RPM;

            // Reset settle timer any time we fall out of the ready band
            if (!flywheelReady) flywheelSettleTimer.reset();
            boolean flywheelSettled = flywheelReady
                    && flywheelSettleTimer.milliseconds() > FLYWHEEL_SETTLE_MS;

            // ── Flywheel power ─────────────────────────────────────────────
            // turnOnFlywheel() sets isFlywheelOn = true so that the post-shot
            // RPM boost compensation inside applyFlywheelPower() actually fires.
            // Without this, forceOnFlywheel() alone skips the boost because
            // isFlywheelOn stays false.
            if (flywheelActive) {
                turret.turnOnFlywheel();
                turret.forceOnFlywheel(rpm);
                Turret.isFlywheelRunning = true;
            } else {
                turret.turnOffFlywheel();
                turret.forceOffFlywheel();
                Turret.isFlywheelRunning = false;
            }

            // ── Hood position ──────────────────────────────────────────────
            if (gamepad1.dpad_up && !dpadUpPressed) {
                currentServoPosition += SERVO_INCREMENT;
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) { dpadUpPressed = false; }

            if (gamepad1.dpad_down && !dpadDownPressed) {
                currentServoPosition -= SERVO_INCREMENT;
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) { dpadDownPressed = false; }

            currentServoPosition = Math.max(0.0, Math.min(1.0, currentServoPosition));
            launchServo.setPosition(currentServoPosition);

            // ── RPM adjust ────────────────────────────────────────────────
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                rpm += MOTOR_INCREMENT;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) { dpadLeftPressed = false; }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                rpm -= MOTOR_INCREMENT;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) { dpadRightPressed = false; }

            rpm = Math.max(0.0, Math.min(6000.0, rpm));

            // ── Intake / transfer ──────────────────────────────────────────
            if (gamepad1.right_trigger > 0.05) {
                intake.turnIntakeOn();
                transfer.turnTransferOn();
            } else if (gamepad1.left_trigger > 0.05) {
                intake.turnIntakeOff();
                transfer.turnTransferOff();
            }

            // ── Flywheel toggle ────────────────────────────────────────────
            if (gamepad1.triangle && !trianglePressed) {
                flywheelActive = !flywheelActive;
                trianglePressed = true;
            } else if (!gamepad1.triangle) { trianglePressed = false; }

            // ── Shoot (gated on flywheel settled + distance stable) ────────
            // This ensures every recorded data point is a valid one.
            // If you need to force a shot regardless, hold OPTIONS while pressing CIRCLE.
            boolean forceShoot = gamepad1.options;
            if (gamepad1.circle && !circlePressed) {
                if (flywheelSettled && distanceStable || forceShoot) {
                    transfer.transferShoot();
                    intake.turnIntakeOn();
                    shotsAttempted++;
                    lastShotDistance = avgDistance;
                    lastShotHood     = currentServoPosition;
                    lastShotRPM      = realRPM; // log actual RPM, not commanded
                }
                circlePressed = true;
            } else if (!gamepad1.circle) { circlePressed = false; }

            if (gamepad1.cross && !crossPressed) {
                transfer.transferEndShoot();
                intake.turnIntakeOff();
                crossPressed = true;
            } else if (!gamepad1.cross) { crossPressed = false; }

            // ── Mark shot scored / missed ──────────────────────────────────
            // R bumper = scored, L bumper = missed (still increments for accuracy calc)
            if (gamepad1.right_bumper && !rBumperPressed) {
                shotsScored++;
                rBumperPressed = true;
            } else if (!gamepad1.right_bumper) { rBumperPressed = false; }

            // ── Reset session counters ─────────────────────────────────────
            if (gamepad1.options && !optionsPressed && !gamepad1.circle) {
                shotsAttempted   = 0;
                shotsScored      = 0;
                lastShotDistance = 0;
                lastShotHood     = 0;
                lastShotRPM      = 0;
                optionsPressed   = true;
            } else if (!gamepad1.options) { optionsPressed = false; }

            // ── Telemetry ──────────────────────────────────────────────────
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("DPAD U/D: Hood angle   DPAD L/R: RPM");
            telemetry.addLine("TRIANGLE: Toggle flywheel");
            telemetry.addLine("R-TRIGGER/L-TRIGGER: Intake+transfer on/off");
            telemetry.addLine("CIRCLE: Shoot (gated)  CROSS: End shot");
            telemetry.addLine("R-BUMPER: Mark scored  OPTIONS: Reset counters");
            telemetry.addLine("OPTIONS+CIRCLE: Force shoot (bypasses gate)");
            telemetry.addLine("");

            telemetry.addLine("=== DISTANCE ===");
            telemetry.addData("Raw", "%.2f in", rawDistance);
            telemetry.addData("Avg (rolling)", "%.2f in", avgDistance);
            telemetry.addData("Variance", "%.2f in  %s",
                    variance, distanceStable ? "[STABLE]" : "[MOVE ROBOT]");
            telemetry.addLine("");

            telemetry.addLine("=== FLYWHEEL ===");
            telemetry.addData("Status", flywheelActive ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.0f", rpm);
            telemetry.addData("Real RPM", "%.0f", realRPM);
            telemetry.addData("RPM Error", "%.0f", rpmError);
            telemetry.addData("At Speed", flywheelReady
                    ? String.format("YES (%.0f ms)", flywheelSettleTimer.milliseconds())
                    : "NO - wait");
            telemetry.addData("Settled", flywheelSettled ? ">>> OK TO SHOOT <<<" : "stabilizing...");
            telemetry.addLine("");

            telemetry.addLine("=== HOOD ===");
            telemetry.addData("Position", "%.3f", currentServoPosition);
            telemetry.addLine("");

            telemetry.addLine("=== SHOT SESSION ===");
            telemetry.addData("Attempted", shotsAttempted);
            telemetry.addData("Scored", shotsScored);
            telemetry.addData("Accuracy", shotsAttempted > 0
                    ? String.format("%.0f%%", (shotsScored / (double) shotsAttempted) * 100.0)
                    : "N/A");
            telemetry.addLine("");

            // Only show record block once at least one shot has been taken
            if (shotsAttempted > 0) {
                telemetry.addLine("=== RECORD THIS (when accuracy is good) ===");
                telemetry.addData("Distance", "%.2f", lastShotDistance);
                telemetry.addData("Hood", "%.3f", lastShotHood);
                telemetry.addData("RPM", "%.0f", lastShotRPM);
                telemetry.addLine("{ " + String.format("%.1f", lastShotDistance)
                        + ", " + String.format("%.3f", lastShotHood)
                        + ", " + String.format("%.0f", lastShotRPM) + " }");
            }

            telemetry.update();
        }
    }
}