package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMaster;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

import java.util.Locale;

public class Turret {

    private final double TURRET_TICKS_PER_DEGREE = 2.34426;
    private final double TURRET_MIN_LIMIT_TICKS = -400;
    private final double TURRET_MAX_LIMIT_TICKS = 400;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 28;
    private double lastTurretPower = 0.0;
    private final double TURRET_SLEW_RATE = 0.115;

    private final DcMotorEx turret;
    public final DcMotorEx flywheelRight;  // Master - uses velocity control
    public final DcMotorEx flywheelLeft;   // Follower - copies master
    private final Servo hood;
    private VoltageSensor voltageSensor;

    private double turretDeg;
    private double turretPower;
    private double hoodPos;
    private double targetRPM;
    private double currVoltage;
    private boolean isFlywheelOn;
    public static boolean isFlywheelRunning = false;

    // FLYWHEEL CONTROL SETTINGS
    private static final double RPM_TOLERANCE = 50; // RPM within 50 = "ready"
    private static final double RPM_SLEW = 250; // RPM per loop
    private double commandedRPM = 0;



    // PID Controllers
    private final PIDFController autoAimClose = new PIDFController(0.0063, 0, 0.0014, 0.096);
    private final PIDFController autoAimFar = new PIDFController(0.0074, 0.0001, 0.0009, 0.085);
    private final PIDFController autoAimSlow = new PIDFController(0.004, 0.00005, 0.0008, 0.0);
    private PIDFController activeAutoAimController;

    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 67.0;
    private static final double VELOCITY_THRESHOLD = 0.1;
    private static final double VELOCITY_COMPENSATION_DURATION_MS = 200;
    private static final double HEADING_COMPENSATION_DURATION_MS = 1000;
    private static final double AUTO_HEADING_COMPENSATION_DURATION_MS = 200;
    private static final double RETURN_TO_CENTER_DURATION_MS = 1400;
    private static final double ROTATIONAL_COMPENSATION_GAIN = 0.34;

    private static final double SEEK_START_DELAY_MS = 200;
    private static final double SEEK_SWEEP_SPEED = 0.4;
    private static final double SEEK_SWEEP_RANGE_DEG = 120;
    private static final double SEEK_PAUSE_AT_EDGE_MS = 50;

    // Tracking state
    private enum CompensationMode {
        VISION,
        VELOCITY_COMP,
        HEADING_COMP,
        SEEKING,
        RETURN_TO_CENTER,
        IDLE
    }

    private CompensationMode currentMode = CompensationMode.IDLE;
    private final ElapsedTime compensationTimer = new ElapsedTime();
    private double robotHeadingOnTargetLoss = 0;
    private boolean hasSeenTargetThisSession = false;
    private boolean wasTargetVisibleLastLoop = false;
    private double headingVelocityAtEndOfVeloCompensation = 0;
    private final double HEADING_VELOCITY_COMPENSATION_FACTOR = 0.3;

    private double seekCenterAngle = 0;
    private boolean seekingSweepingRight = true;
    private final ElapsedTime seekPauseTimer = new ElapsedTime();
    private boolean seekPaused = false;

    private final double[][] launchAngleLookupTable = {
            { 21, .72, 2360},   // inches, servo, RPM
            { 31, .72, 2390},
            { 36, .74, 2450},
            { 40, .74, 2540},
            { 50, .820, 2810},
            { 60, .86, 2900},
            { 71, .87, 3110},
            { 80, .94, 3470},
            { 96, .97, 3740},
            { 103, .98, 3770},
            { 115, .99, 4000}
    };

    public Turret(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");
        initVoltageSensor(hwMap);

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setVelocityPIDFCoefficients(
                2.5,   // P
                0.0,    // I
                1.2,    // D
                12.5    // F
        );



        activeAutoAimController = autoAimClose;
        autoAimFar.setReference(0);
        autoAimClose.setReference(0);
        autoAimSlow.setReference(0);
        resetTracking();
    }

    public void updateTurret() {
        turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        turretPower = turret.getPower();
        hoodPos = hood.getPosition();
        double currentDistance = Limelight.getDistance();
        currVoltage = voltageSensor.getVoltage();

        hood.setPosition(getServoPositionFromDistance(currentDistance));
        targetRPM = getRPMFromDistance(currentDistance);

        if (isFlywheelOn) {
            isFlywheelRunning = true;
            setFlywheelRPM(targetRPM);
        } else {
            isFlywheelRunning = false;
            stopFlywheels();
        }
    }

    /**
     * NEW: Set flywheel speed in RPM
     * Master (right) uses velocity control, follower (left) copies
     */
    public void setFlywheelRPM(double targetRPM) {

        // Slew-limit RPM
        commandedRPM += Range.clip(
                targetRPM - commandedRPM,
                -RPM_SLEW,
                RPM_SLEW
        );

        double targetTPS = commandedRPM * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);

        flywheelLeft.setVelocity(targetTPS);

        flywheelRight.setPower(flywheelLeft.getPower());
    }


    /**
     * NEW: Calculate power needed for a given RPM
     * Uses feedforward estimation: Power ≈ RPM / MAX_RPM
     */
    private double calculatePowerFromRPM(double rpm) {
        // Adjust MAX_RPM based on your motors (6000 is typical for many FTC motors)
        final double MAX_RPM = 6000.0;
        double power = rpm / MAX_RPM;
        return Range.clip(power, 0.0, 1.0);
    }

    /**
     * NEW: Stop both flywheels
     */
    private void stopFlywheels() {
        commandedRPM = 0;
        setFlywheelRPM(0);
    }

    /**
     * NEW: Check if flywheel is at target speed
     */
    public boolean isFlywheelReady() {
        if (!isFlywheelOn) return false;

        double currentRPM = getCurrentFlywheelRPM();
        double error = Math.abs(targetRPM - currentRPM);

        return error < RPM_TOLERANCE;
    }

    /**
     * NEW: Get current flywheel RPM from master wheel
     */
    public double getCurrentFlywheelRPM() {
        double ticksPerSecond = flywheelLeft.getVelocity(); //rpm is negative for some fuh ahh reason
        return (ticksPerSecond / FLYWHEEL_TICKS_PER_REVOLUTION) * 60.0;
    }


    public void aimTurret(boolean hasValidTarget, double limelightError, double manualTurnInput,
                          double distance, double robotHeading, double robotHeadingVelocity,
                          double avgXYVelocity) {

        double calculatedPower;
        boolean useAutoAim = !(Math.abs(manualTurnInput) > 0.05) &&
                (hasValidTarget || hasSeenTargetThisSession);

        if (useAutoAim) {
            double errorToUse;

            if (hasValidTarget) {
                currentMode = CompensationMode.VISION;
                seekCenterAngle = turretDeg;
                seekingSweepingRight = true;
                seekPaused = false;

                double angularVelocity = Math.abs(robotHeadingVelocity);

                if (angularVelocity < VELOCITY_THRESHOLD && avgXYVelocity < VELOCITY_THRESHOLD) {
                    if (activeAutoAimController != autoAimSlow) {
                        autoAimClose.reset();
                        autoAimFar.reset();
                    }
                    activeAutoAimController = autoAimSlow;
                } else if (distance < GAIN_SCHEDULING_DISTANCE_THRESHOLD && distance > 0) {
                    if (activeAutoAimController != autoAimClose) {
                        autoAimFar.reset();
                        autoAimSlow.reset();
                    }
                    activeAutoAimController = autoAimClose;
                } else {
                    if (activeAutoAimController != autoAimFar) {
                        autoAimClose.reset();
                        autoAimSlow.reset();
                    }
                    activeAutoAimController = autoAimFar;
                }

                if (limelightError > 180) limelightError -= 360;
                else if (limelightError < -180) limelightError += 360;

                errorToUse = limelightError;
                compensationTimer.reset();
                hasSeenTargetThisSession = true;

            } else {
                if (wasTargetVisibleLastLoop) {
                    compensationTimer.reset();
                    robotHeadingOnTargetLoss = robotHeading;
                    seekCenterAngle = turretDeg;
                }

                double timeSinceLost = compensationTimer.milliseconds();

                if (timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS) {
                    currentMode = CompensationMode.VELOCITY_COMP;
                    double compensationPower = robotHeadingVelocity * ROTATIONAL_COMPENSATION_GAIN;
                    calculatedPower = compensationPower;
                    applyTurretPower(calculatedPower);
                    wasTargetVisibleLastLoop = hasValidTarget;
                    headingVelocityAtEndOfVeloCompensation = robotHeadingVelocity;
                    return;

                } else if (timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS +
                        ((RobotMaster.isAuto) ? AUTO_HEADING_COMPENSATION_DURATION_MS :
                                HEADING_COMPENSATION_DURATION_MS)) {
                    currentMode = CompensationMode.HEADING_COMP;
                    double headingDirection = Math.signum(robotHeading - robotHeadingOnTargetLoss);
                    errorToUse = -headingDirection * headingVelocityAtEndOfVeloCompensation *
                            HEADING_VELOCITY_COMPENSATION_FACTOR;

                    if (activeAutoAimController != autoAimClose) {
                        autoAimFar.reset();
                    }
                    activeAutoAimController = autoAimClose;

                }
                else if (RobotMaster.isAuto && !RobotMaster.isAutoFar &&
                        timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS +
                                AUTO_HEADING_COMPENSATION_DURATION_MS + SEEK_START_DELAY_MS + 5000) {
                    currentMode = CompensationMode.SEEKING;
                    calculatedPower = performSeekingSweep();
                    applyTurretPower(calculatedPower);
                    wasTargetVisibleLastLoop = hasValidTarget;
                    return;

                }
                  else if (
                        timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS +
                                HEADING_COMPENSATION_DURATION_MS + RETURN_TO_CENTER_DURATION_MS) {
                    currentMode = CompensationMode.RETURN_TO_CENTER;
                    errorToUse = -turretDeg;

                    if (activeAutoAimController != autoAimFar) {
                        autoAimClose.reset();
                    }
                    activeAutoAimController = autoAimFar;

                } else {
                    currentMode = CompensationMode.IDLE;
                    calculatedPower = 0.0;
                    applyTurretPower(calculatedPower);
                    wasTargetVisibleLastLoop = hasValidTarget;
                    return;
                }
            }

            calculatedPower = -activeAutoAimController.calculatePIDF(errorToUse);

        } else {
            currentMode = CompensationMode.IDLE;
            calculatedPower = manualTurnInput;
            autoAimClose.reset();
            autoAimFar.reset();
        }

        applyTurretPower(calculatedPower);
        wasTargetVisibleLastLoop = hasValidTarget;
    }

    private double performSeekingSweep() {
        if (seekPaused) {
            if (seekPauseTimer.milliseconds() > SEEK_PAUSE_AT_EDGE_MS) {
                seekPaused = false;
                seekingSweepingRight = !seekingSweepingRight;
            }
            return 0;
        }

        double leftLimit = seekCenterAngle - (SEEK_SWEEP_RANGE_DEG / 2);
        double rightLimit = seekCenterAngle + (SEEK_SWEEP_RANGE_DEG / 2);

        leftLimit = Math.max(leftLimit, TURRET_MIN_LIMIT_TICKS / TURRET_TICKS_PER_DEGREE);
        rightLimit = Math.min(rightLimit, TURRET_MAX_LIMIT_TICKS / TURRET_TICKS_PER_DEGREE);

        if (seekingSweepingRight && turretDeg >= rightLimit) {
            seekPaused = true;
            seekPauseTimer.reset();
            return 0;
        } else if (!seekingSweepingRight && turretDeg <= leftLimit) {
            seekPaused = true;
            seekPauseTimer.reset();
            return 0;
        }


        return seekingSweepingRight ? SEEK_SWEEP_SPEED : -SEEK_SWEEP_SPEED;
    }

    private void applyTurretPower(double calculatedPower) {
        double delta = calculatedPower - lastTurretPower;
        if (Math.abs(delta) > TURRET_SLEW_RATE) {
            calculatedPower = lastTurretPower + Math.signum(delta) * TURRET_SLEW_RATE;
        }
        lastTurretPower = calculatedPower;

        int currentPosition = turret.getCurrentPosition();
        if (currentPosition >= TURRET_MAX_LIMIT_TICKS && calculatedPower > 0) {
            turret.setPower(0);
            lastTurretPower = calculatedPower;
        } else if (currentPosition <= TURRET_MIN_LIMIT_TICKS && calculatedPower < 0) {
            turret.setPower(0);
            lastTurretPower = calculatedPower;
        } else {
            turret.setPower(calculatedPower);
        }
    }

    public void resetTracking() {
        hasSeenTargetThisSession = false;
        currentMode = CompensationMode.IDLE;
        compensationTimer.reset();
        robotHeadingOnTargetLoss = 0;
        wasTargetVisibleLastLoop = false;
        seekCenterAngle = 0;
        seekingSweepingRight = true;
        seekPaused = false;
    }

    public String getTrackingModeForTelemetry() {
        switch (currentMode) {
            case VISION: return "VISION";
            case VELOCITY_COMP:
                return String.format(Locale.US, "VELOCITY (%.0fms)", compensationTimer.milliseconds());
            case HEADING_COMP:
                return String.format(Locale.US, "HEADING (%.0fms)", compensationTimer.milliseconds());
            case SEEKING:
                return String.format(Locale.US, "SEEKING %s (center:%.1f°)",
                        seekingSweepingRight ? "→" : "←", seekCenterAngle);
            case RETURN_TO_CENTER:
                return String.format(Locale.US, "CENTERING (%.0fms)", compensationTimer.milliseconds());
            case IDLE: return "IDLE";
            default: return "UNKNOWN";
        }
    }

    public boolean hasGoodLock() {
        return currentMode == CompensationMode.VISION && wasTargetVisibleLastLoop;
    }

    public void turnOnFlywheel() { isFlywheelOn = true; }
    public void turnOffFlywheel() { isFlywheelOn = false; }

    public double getServoPositionFromDistance(double distance) {
        if (distance <= launchAngleLookupTable[0][0]) {
            return launchAngleLookupTable[0][1];
        }
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) {
            return launchAngleLookupTable[launchAngleLookupTable.length - 1][1];
        }

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lowerBound = launchAngleLookupTable[i];
            double[] upperBound = launchAngleLookupTable[i+1];

            if (distance >= lowerBound[0] && distance <= upperBound[0]) {
                double distanceRange = upperBound[0] - lowerBound[0];
                double servoRange = upperBound[1] - lowerBound[1];
                double distanceRatio = (distance - lowerBound[0]) / distanceRange;

                return lowerBound[1] + (distanceRatio * servoRange);
            }
        }
        return 0.5;
    }

    // CHANGED: Now returns RPM instead of power
    public double getRPMFromDistance(double distance) {
        if (distance <= launchAngleLookupTable[0][0]) {
            return launchAngleLookupTable[0][2];
        }
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) {
            return launchAngleLookupTable[launchAngleLookupTable.length - 1][2];
        }

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lowerBound = launchAngleLookupTable[i];
            double[] upperBound = launchAngleLookupTable[i+1];

            if (distance >= lowerBound[0] && distance <= upperBound[0]) {
                double distanceRange = upperBound[0] - lowerBound[0];
                double rpmRange = upperBound[2] - lowerBound[2];
                double distanceRatio = (distance - lowerBound[0]) / distanceRange;

                return Range.clip((lowerBound[2] + (distanceRatio * rpmRange)), 3000, 6000);
            }
        }
        return 4500;
    }

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Aim ---");
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Degrees", "%.2f°", turretDeg);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Hood Position", "%.2f", getServoPositionFromDistance(Limelight.getDistance()));

        // IMPROVED: Show RPM info
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", getCurrentFlywheelRPM());
        telemetry.addData("RPM Error", "%.0f", targetRPM - getCurrentFlywheelRPM());
        telemetry.addData("Flywheel Ready?", isFlywheelReady() ? "✓ YES" : "✗ NO");

        telemetry.addData("Motor Power (L/R)", String.format(Locale.US,"%.2f / %.2f",
                flywheelLeft.getPower(), flywheelRight.getPower()));
        telemetry.addData("Current Voltage", "%.2fV", currVoltage);
        telemetry.addData("Turret Tracking Mode", getTrackingModeForTelemetry());
    }

    public double getTurretDeg() { return turretDeg; }
    public double getTurretPower() { return turretPower; }
    public void setTurretPower(double turretPower) { turret.setPower(turretPower); }
    public double getHoodPos() { return hoodPos; }
    public void setHoodPos(double hoodPos) { hood.setPosition(hoodPos); }

    private void initVoltageSensor(HardwareMap hwMap) {
        voltageSensor = hwMap.voltageSensor.iterator().next();
    }

    public void resetPID() {
        autoAimClose.reset();
        autoAimFar.reset();
        autoAimSlow.reset();
    }
}