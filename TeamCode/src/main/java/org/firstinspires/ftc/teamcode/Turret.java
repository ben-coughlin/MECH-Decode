package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

public class Turret
{
    private final double TURRET_TICKS_PER_DEGREE = 2.34426;
    private final double TURRET_MIN_LIMIT_TICKS = -400;
    private final double TURRET_MAX_LIMIT_TICKS = 400;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 28;
    private double lastTurretPower = 0.0;
    private final double TURRET_SLEW_RATE = 0.115;
    private final DcMotorEx turret;
    public final DcMotorEx flywheelLeft;
    public final DcMotorEx flywheelRight;
    private final Servo hood;
    private VoltageSensor voltageSensor;
    private double turretDeg;
    private double turretPower;
    private double hoodPos;
    private double targetPower;
    private double currVoltage;
    private boolean isFlywheelOn;
    public static boolean isFlywheelRunning = false;

    //turret stuff
    private final PIDFController autoAimClose = new PIDFController(0.0063, 0, 0.0014, 0.096);
    private final PIDFController autoAimFar = new PIDFController(0.007, 0.0001, 0.0009, 0.08);
    private final PIDFController autoAimSlow = new PIDFController(0.004, 0.00005, 0.0008, 0.0);
    private PIDFController activeAutoAimController;
    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 67.0; // ( ͡°👅 ͡°)
    private static final double VELOCITY_THRESHOLD = 0.1;
    private static final double VELOCITY_COMPENSATION_DURATION_MS = 200;
    private static final double HEADING_COMPENSATION_DURATION_MS = 1000;
    private static final double AUTO_HEADING_COMPENSATION_DURATION_MS = 200;
    private static final double RETURN_TO_CENTER_DURATION_MS = 1400;
    private static final double ROTATIONAL_COMPENSATION_GAIN = 0.34;



    // Tracking state
    private enum CompensationMode {
        VISION,              // Currently seeing target
        VELOCITY_COMP,       // Just lost - compensating based on rotation velocity
        HEADING_COMP,        // Extended - compensating based on heading change
        RETURN_TO_CENTER,    // Returning turret to center
        IDLE                 // No compensation
    }

    private CompensationMode currentMode = CompensationMode.IDLE;
    private ElapsedTime compensationTimer = new ElapsedTime();
    private double robotHeadingOnTargetLoss = 0;
    private boolean hasSeenTargetThisSession = false;
    private boolean wasTargetVisibleLastLoop = false;
    private double headingVelocityAtEndOfVeloCompensation = 0;
    private final double HEADING_VELOCITY_COMPENSATION_FACTOR = 0.4;


    private final double[][] launchAngleLookupTable = {
            { 21, .72, .56}, //inches, servo, powa
            { 31, .72, .565},
            {36, .74, .575},
            {40, .74, .59},
            {50, .820, .635},
            {60, .86, .650},
            {71, .87, .685},
            {80, .94, .745},
            {96, .97, .79},
            {103, .98, .795},
            {115, .99, .83}

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
        flywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight.setVelocityPIDFCoefficients(100, 0, 0, 0.6);

        activeAutoAimController = autoAimClose;
        autoAimFar.setReference(0);
        autoAimClose.setReference(0);
        autoAimSlow.setReference(0);
        resetTracking();
    }

    public void updateTurret()
    {
        turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        turretPower = turret.getPower();
        hoodPos = hood.getPosition();
        double currentDistance = Limelight.getDistance();
        currVoltage = voltageSensor.getVoltage();

        hood.setPosition(getServoPositionFromDistance(currentDistance));
        targetPower = getPowerFromDistance(currentDistance);
        if ((isFlywheelOn)) {
            isFlywheelRunning = true;
            flywheelRight.setPower(targetPower);
            flywheelLeft.setPower(targetPower);
        } else {
            isFlywheelRunning = false;
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
        }

    }

    /**
     * Enhanced turret aiming with multi-stage compensation
     * @param hasValidTarget - true if Limelight currently sees the target
     * @param limelightError - error from Limelight in degrees
     * @param manualTurnInput - manual control input (-1 to 1)
     * @param distance - distance to target
     * @param robotHeading - current robot heading in degrees
     * @param robotHeadingVelocity - rate of robot rotation in degrees/second
     */
    public void aimTurret(boolean hasValidTarget, double limelightError, double manualTurnInput,
                          double distance, double robotHeading, double robotHeadingVelocity, double avgXYVelocity) {
        double calculatedPower;
        boolean useAutoAim = !(Math.abs(manualTurnInput) > 0.05) && (hasValidTarget || hasSeenTargetThisSession);

        if (useAutoAim) {
            double errorToUse;

            if (hasValidTarget) {
                // --- STAGE 0: VISION MODE (Target is visible) ---
                currentMode = CompensationMode.VISION;


                double angularVelocity = Math.abs(robotHeadingVelocity);

                if (angularVelocity < VELOCITY_THRESHOLD || avgXYVelocity < VELOCITY_THRESHOLD) {
                    // Robot is stopped or moving slowly - use stationary controller
                    if (activeAutoAimController != autoAimSlow) {
                        autoAimClose.reset();
                        autoAimFar.reset();
                    }
                    activeAutoAimController = autoAimSlow;
                } else if (distance < GAIN_SCHEDULING_DISTANCE_THRESHOLD && distance > 0) {
                    // Moving and close distance
                    if (activeAutoAimController != autoAimClose) {
                        autoAimFar.reset();
                        autoAimSlow.reset();
                    }
                    activeAutoAimController = autoAimClose;
                } else {
                    // Moving and far distance
                    if (activeAutoAimController != autoAimFar) {
                        autoAimClose.reset();
                        autoAimSlow.reset();
                    }
                    activeAutoAimController = autoAimFar;
                }

                if (limelightError > 180) limelightError -= 360;
                else if (limelightError < -180) limelightError += 360;

                errorToUse = limelightError;

                // Reset compensation state
                compensationTimer.reset();
                hasSeenTargetThisSession = true;

            } else {

                if (wasTargetVisibleLastLoop) {
                    compensationTimer.reset();
                    robotHeadingOnTargetLoss = robotHeading;
                }

                double timeSinceLost = compensationTimer.milliseconds();

                if (timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS) {
                    // --- STAGE 1: VELOCITY-BASED COMPENSATION ---
                    currentMode = CompensationMode.VELOCITY_COMP;

                    // Counteract robot rotation immediately
                    double compensationPower = robotHeadingVelocity * ROTATIONAL_COMPENSATION_GAIN;
                    calculatedPower = compensationPower;
                    errorToUse = Double.NaN; // No PID error in this mode

                    // Apply power directly, skip PID calculation below
                    applyTurretPower(calculatedPower);
                    wasTargetVisibleLastLoop = hasValidTarget;
                    headingVelocityAtEndOfVeloCompensation = robotHeadingVelocity;
                    return;

                } else if (timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS + ((RobotMasterPinpoint.isAuto) ? AUTO_HEADING_COMPENSATION_DURATION_MS : HEADING_COMPENSATION_DURATION_MS)) {
                    currentMode = CompensationMode.HEADING_COMP;

                    //figure out which direction to rotate; signum example: -0.5 = -1.0, 0 = 0, 15 = 1
                     double headingDirection = Math.signum(robotHeading - robotHeadingOnTargetLoss);

                     errorToUse = -headingDirection * headingVelocityAtEndOfVeloCompensation * HEADING_VELOCITY_COMPENSATION_FACTOR;

                    // Use far controller (more damped)
                    if (activeAutoAimController != autoAimClose) {
                        autoAimFar.reset();
                    }
                    activeAutoAimController = autoAimClose;

                } else if (timeSinceLost < VELOCITY_COMPENSATION_DURATION_MS +
                        HEADING_COMPENSATION_DURATION_MS + RETURN_TO_CENTER_DURATION_MS) {
                    // --- STAGE 3: RETURN TO CENTER ---
                    currentMode = CompensationMode.RETURN_TO_CENTER;

                    // Error is current turret angle - we want to go to 0
                    errorToUse = -turretDeg;

                    // Use far controller for smooth return
                    if (activeAutoAimController != autoAimFar) {
                        autoAimClose.reset();
                    }
                    activeAutoAimController = autoAimFar;

                } else {
                    // --- STAGE 4: IDLE ---
                    currentMode = CompensationMode.IDLE;
                    calculatedPower = 0.0;
                    applyTurretPower(calculatedPower);
                    wasTargetVisibleLastLoop = hasValidTarget;
                    return;
                }
            }

            calculatedPower = -activeAutoAimController.calculatePIDF(errorToUse);

        } else {
            // Manual mode
            currentMode = CompensationMode.IDLE;
            calculatedPower = manualTurnInput;
            autoAimClose.reset();
            autoAimFar.reset();
        }

        applyTurretPower(calculatedPower);
        wasTargetVisibleLastLoop = hasValidTarget;
    }

    /**
     * Apply power to turret with slew rate limiting and soft limits
     */
    private void applyTurretPower(double calculatedPower) {
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
            lastTurretPower = calculatedPower;
        } else if (currentPosition <= TURRET_MIN_LIMIT_TICKS && calculatedPower < 0) {
            turret.setPower(0);
            lastTurretPower = calculatedPower;
        } else {
            turret.setPower(calculatedPower);
        }
    }

    /**
     * Normalize angle to -180 to 180 range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Reset tracking state (call at start of match or when switching modes)
     */
    public void resetTracking() {
        hasSeenTargetThisSession = false;
        currentMode = CompensationMode.IDLE;
        compensationTimer.reset();
        robotHeadingOnTargetLoss = 0;
        wasTargetVisibleLastLoop = false;
    }

    /**
     * Get current compensation mode for telemetry
     */
    public String getTrackingModeForTelemetry() {
        switch (currentMode) {
            case VISION: return "VISION";
            case VELOCITY_COMP:
                return String.format("VELOCITY (%.0fms)", compensationTimer.milliseconds());
            case HEADING_COMP:
                return String.format("HEADING (%.0fms)", compensationTimer.milliseconds());
            case RETURN_TO_CENTER:
                return String.format("CENTERING (%.0fms)", compensationTimer.milliseconds());
            case IDLE: return "IDLE";
            default: return "UNKNOWN";
        }
    }

    public void setFlywheelVelocity(double targetRPM)
    {
        double targetTicksPerSecond = targetRPM * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);
        flywheelRight.setVelocity(targetTicksPerSecond);

        double feedforwardPower = targetRPM / 6000.0;
        feedforwardPower = Range.clip(feedforwardPower, -1.0, 1.0);


        flywheelLeft.setPower(feedforwardPower);
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

                return Range.clip((lowerBound[2] + (distanceRatio * powerRange))
                   , 0.40, 1);
            }
        }
        return 0.9;
    }

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Aim ---");
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Degrees", turretDeg);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Interpolated Servo Position", "%.2f",
                getServoPositionFromDistance(Limelight.getDistance()));
        telemetry.addData("Flywheel RPM", flywheelRight.getVelocity() /
                FLYWHEEL_TICKS_PER_REVOLUTION * 60);
        telemetry.addData("Target Power", targetPower);
        telemetry.addData("Motor Power (L/R)", String.format(Locale.US,"%.2f / %.2f",
                flywheelLeft.getPower(), flywheelRight.getPower()));
        telemetry.addData("Current Voltage", currVoltage);
        telemetry.addData("Turret Tracking Mode", getTrackingModeForTelemetry());
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
        autoAimSlow.reset();
    }
}