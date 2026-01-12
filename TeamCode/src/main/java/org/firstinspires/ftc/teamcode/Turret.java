package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class Turret
{
    private final double TURRET_TICKS_PER_DEGREE = 2.563; // 384.5 PPR encoder with 2.4:1 gear reduction (48/20 gears)
    private final double TURRET_MIN_LIMIT_TICKS = -450;
    private final double TURRET_MAX_LIMIT_TICKS = 450;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 28; //rev throughbore
    private double lastTurretPower = 0.0;
    private final double TURRET_SLEW_RATE = 0.1; // max change in power per loop`
    private final DcMotorEx turret;
    public final DcMotorEx flywheelLeft;
    public final DcMotorEx flywheelRight;
    private final Servo hood;
    private VoltageSensor voltageSensor;
    private final PIDFController autoAimClose = new PIDFController(0.0064, 0.000001, 0.009, 0.097);
    private final PIDFController autoAimFar = new PIDFController(0.007, 0.0001, 0.0009, 0.08);
    private  PIDFController activeAutoAimController;
    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 72.0;
    private double turretDeg;
    private double turretPower;
    private double llError = 0;
    private double hoodPos;
    private double targetRPM;
    private double currVoltage;
    private final double NOMINAL_VOLTAGE = 12.0;
    private boolean isFlywheelOn;

    // Constants
    private static final double VISION_TIMEOUT_MS = 500; // how long before switching to odometry mode

    // Odometry tracking fields
    private double lastSeenTagX = 0; // field coordinates of tag when last seen
    private double lastSeenTagY = 0;
    private long lastVisionUpdateTime = 0;
    private boolean hasSeenTagBefore = false;

    private final double[][] launchAngleLookupTable = {
            { 25, 0.61, 4000},   // inches from apriltag, servo angle, rpm
            { 36, 0.75, 4050},
            {42, .77, 4500},
            {48, .787, 4500},
            { 60, 0.795, 4800 },
            { 70, 0.810, 5200 },
            { 93, 0.765, 5400 },
            { 125, 0.8, 6000 }
    };

    public Turret(HardwareMap hwMap)
    {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");
        initVoltageSensor(hwMap);


        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight.setVelocityPIDFCoefficients(100, 0, 0, 0.6);


        activeAutoAimController = autoAimClose;
        autoAimFar.setReference(0);
        autoAimClose.setReference(0);

    }

    public void updateTurret()
    {
        turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        turretPower = turret.getPower();
        hoodPos = hood.getPosition();
        double currentDistance = Limelight.getDistance();
        currVoltage = voltageSensor.getVoltage();

        hood.setPosition(getServoPositionFromDistance(currentDistance));
        if ((isFlywheelOn)) {
            setFlywheelVelocity(getPowerFromDistance(currentDistance));
        } else {
            setFlywheelVelocity(0);
        }
        targetRPM = getPowerFromDistance(currentDistance);


    }

    /**
     * Aims the turret using either auto-aim or manual input, and selects PIDF gains based on distance.
     * @param useAutoAim        Boolean to enable/disable auto-aim.
     * @param limelightError    The 'tx' error from the Limelight.
     * @param manualTurnInput   Manual joystick input.
     * @param distance          The distance to the target, used for gain scheduling.
     */
    /**
     * Call this method when you have a valid AprilTag detection
     * @param tagFieldX - X position of tag in field coordinates
     * @param tagFieldY - Y position of tag in field coordinates
     */
    public void updateLastSeenTagPosition(double tagFieldX, double tagFieldY) {
        lastSeenTagX = tagFieldX;
        lastSeenTagY = tagFieldY;
        lastVisionUpdateTime = System.currentTimeMillis();
        hasSeenTagBefore = true;
    }

    /**
     * Calculate the angle error to the last seen tag using current odometry
     * @param robotX - current robot X position (field coordinates)
     * @param robotY - current robot Y position (field coordinates)
     * @param robotHeading - current robot heading in degrees (0 = forward along field X axis)
     * @param currentTurretAngle - current turret angle in degrees relative to robot (0 = forward)
     * @return angle error in degrees that turret needs to rotate
     */
    public double calculateOdometryBasedError(double robotX, double robotY,
                                              double robotHeading, double currentTurretAngle) {
        // Calculate vector from robot to last seen tag
        double deltaX = lastSeenTagX - robotX;
        double deltaY = lastSeenTagY - robotY;

        // Calculate absolute angle to target in field coordinates
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Calculate angle relative to robot's current heading
        double angleRelativeToRobot = angleToTarget - robotHeading;

        // Normalize to -180 to 180
        while (angleRelativeToRobot > 180) angleRelativeToRobot -= 360;
        while (angleRelativeToRobot < -180) angleRelativeToRobot += 360;

        // Subtract current turret angle to get error
        double turretError = angleRelativeToRobot - currentTurretAngle;

        // Normalize error to -180 to 180
        while (turretError > 180) turretError -= 360;
        while (turretError < -180) turretError += 360;

        return turretError;
    }

    /**
     * Enhanced turret aiming with odometry fallback
     * @param hasValidTarget - true if Limelight currently sees the target
     * @param limelightError - error from Limelight in degrees (only used if hasValidTarget is true)
     * @param manualTurnInput - manual control input (-1 to 1)
     * @param distance - distance to target
     * @param robotX - current robot X position (field coordinates)
     * @param robotY - current robot Y position (field coordinates)
     * @param robotHeading - current robot heading in degrees
     * @param currentTurretAngle - current turret angle in degrees relative to robot
     */
    public void aimTurret(boolean hasValidTarget, double limelightError, double manualTurnInput,
                          double distance, double robotX, double robotY,
                          double robotHeading, double currentTurretAngle) {
        double calculatedPower;
        boolean useAutoAim = hasValidTarget || isUsingOdometryMode();

        if (useAutoAim) {
            double errorToUse;

            if (hasValidTarget) {
                // Use vision-based error
                if (distance < GAIN_SCHEDULING_DISTANCE_THRESHOLD && distance > 0) {
                    if (activeAutoAimController != autoAimClose) {
                        autoAimFar.reset();
                    }
                    activeAutoAimController = autoAimClose;
                } else {
                    if (activeAutoAimController != autoAimFar) {
                        autoAimClose.reset();
                    }
                    activeAutoAimController = autoAimFar;
                }

                if (limelightError > 180) limelightError -= 360;
                else if (limelightError < -180) limelightError += 360;

                errorToUse = limelightError;
            } else {
                // Use odometry-based error
                errorToUse = calculateOdometryBasedError(robotX, robotY, robotHeading, currentTurretAngle);

                // Use far controller for odometry mode (more damped, less aggressive)
                if (activeAutoAimController != autoAimFar) {
                    autoAimClose.reset();
                }
                activeAutoAimController = autoAimFar;
            }

            llError = errorToUse;
            calculatedPower = -activeAutoAimController.calculatePIDF(errorToUse);
        } else {
            // Manual mode
            calculatedPower = manualTurnInput;
            autoAimClose.reset();
            autoAimFar.reset();
        }

        // Slew rate limiting
        double delta = calculatedPower - lastTurretPower;
        if (Math.abs(delta) > TURRET_SLEW_RATE) {
            calculatedPower = lastTurretPower + Math.signum(delta) * TURRET_SLEW_RATE;
        }
        lastTurretPower = calculatedPower;

        // Soft limits
        int currentPosition = turret.getCurrentPosition();
        if (currentPosition >= TURRET_MAX_LIMIT_TICKS && calculatedPower > 0) {
            turret.setPower(0);
            lastTurretPower = calculatedPower; // Don't reset to 0
        } else if (currentPosition <= TURRET_MIN_LIMIT_TICKS && calculatedPower < 0) {
            turret.setPower(0);
            lastTurretPower = calculatedPower; // Don't reset to 0
        } else {
            turret.setPower(calculatedPower);
        }
    }

    /**
     * Determines if we should use odometry-based tracking
     * @return true if we've lost vision recently but have a previous tag position
     */
    private boolean isUsingOdometryMode() {
        if (!hasSeenTagBefore) return false;

        long timeSinceLastVision = System.currentTimeMillis() - lastVisionUpdateTime;
        return timeSinceLastVision < VISION_TIMEOUT_MS;
    }

    /**
     * Call this to reset odometry tracking (e.g., at start of match)
     */
    public void resetOdometryTracking() {
        hasSeenTagBefore = false;
        lastSeenTagX = 0;
        lastSeenTagY = 0;
        lastVisionUpdateTime = 0;
    }

    /**
     * Get status for telemetry
     */
    public String getTrackingMode() {
        long timeSinceLastVision = System.currentTimeMillis() - lastVisionUpdateTime;
        if (timeSinceLastVision < 100) {
            return "VISION";
        } else if (isUsingOdometryMode()) {
            return "ODOMETRY (" + timeSinceLastVision + "ms)";
        } else {
            return "MANUAL";
        }
    }


    public void setFlywheelVelocity(double targetRPM)
    {

        double targetTicksPerSecond = targetRPM * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);
        flywheelRight.setVelocity(targetTicksPerSecond);


        Log.i("Flywheel RPM (Real/Target)", String.format(Locale.US,"%.2f / %.2f", flywheelRight.getVelocity() / FLYWHEEL_TICKS_PER_REVOLUTION * 60, targetRPM));

        double feedforwardPower = targetRPM / 6000.0;

        // Clip the power to the valid -1.0 to 1.0 range.
        feedforwardPower = Range.clip(feedforwardPower, -1.0, 1.0);

        feedforwardPower *= (NOMINAL_VOLTAGE / currVoltage);

       flywheelLeft.setPower(feedforwardPower);
        //flywheelRight.setPower(1);
    }


    public void turnOnFlywheel()
    {
        isFlywheelOn = true;
    }
    public void turnOffFlywheel()
    {
        isFlywheelOn = false;
    }


    /**
     * Uses the lookup table and linear interpolation to find the correct servo position.
     * @param distance The current distance to the target.
     * @return The calculated servo position.
     */
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

    /**
     * Uses the lookup table and linear interpolation to find the correct target power.
     * @param distance The current distance to the target.
     * @return The calculated target power.
     */
    public double getPowerFromDistance(double distance) {
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
                double powerRange = upperBound[2] - lowerBound[2];
                double distanceRatio = (distance - lowerBound[0]) / distanceRange;

                return Range.clip((lowerBound[2] + (distanceRatio * powerRange)) * (NOMINAL_VOLTAGE / currVoltage), 4000, 6000);
            }
        }

        return 6000; //full send if this fails
    }

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Aim ---");
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Degrees", turretDeg);
        telemetry.addData("LLErr", llError);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Interpolated Servo Position", "%.2f", getServoPositionFromDistance(Limelight.getDistance()));
        telemetry.addData("Flywheel RPM", flywheelRight.getVelocity() / FLYWHEEL_TICKS_PER_REVOLUTION * 60);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Motor Power (L/R)", String.format(Locale.US,"%.2f / %.2f", flywheelLeft.getPower(), flywheelRight.getPower()));
        telemetry.addData("Current Voltage", currVoltage);
        telemetry.addData("Turret Tracking Mode", getTrackingMode());
    }

    public double getTurretDeg() { return turretDeg; }


    public double getTurretPower() { return turretPower; }
    public void setTurretPower(double turretPower) { turret.setPower(turretPower); }
    public void setFlywheelLeftPower(double flywheelLeftPower) { flywheelLeft.setPower(flywheelLeftPower); }
    public void setFlywheelRightPower(double flywheelRightPower) { flywheelRight.setPower(flywheelRightPower); }
    public double getHoodPos() { return hoodPos; }
    public void setHoodPos(double hoodPos) { hood.setPosition(hoodPos); }

    public void resetEncoder()
    {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initVoltageSensor(HardwareMap hwMap)
    {
        voltageSensor = hwMap.voltageSensor.iterator().next();
    }
    public void resetPID()
    {
        autoAimClose.reset();
        autoAimFar.reset();
    }







}
