package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

public class Turret {

    // Hardware Constants
    private final double TURRET_TICKS_PER_DEGREE = 67.06899;
    private final double TURRET_MIN_LIMIT_TICKS = -10000;
    private final double TURRET_MAX_LIMIT_TICKS = 10000;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 28;
    private final double TURRET_SLEW_RATE = 0.12;

    private double lastValidCalculatedPower = 0.0;
    private final CRServo turretLeft, turretRight;
    private final DcMotorEx flywheelLeft, flywheelRight;
    private final DcMotorEx turretEncoder;
    private final Servo hood;
    private double turretDeg;
    private double timeSinceLost = 0;
    private boolean isFlywheelOn;
    public static boolean isFlywheelRunning = false;
    private String currentMode = "IDLE";

    private final PIDFController visionAimClose = new PIDFController(0.009, 0.0, 0.0029, 0.0);
    private final PIDFController visionAimCloseStationary = new PIDFController(0.0105, 0, 0.00019, 0.0);

    private final PIDFController visionAimFar = new PIDFController(0.0088, 0, 0.0012, 0);
    private final PIDFController visionAimFarStationary = new PIDFController(0.0097, 0, 0.0019, 0);
    private final PIDFController chassisCenterController = new PIDFController(0.2, 0, 0.1, 0.03);
    private final ElapsedTime timeSinceLostTimer = new ElapsedTime();

    private final double MAX_FLYWHEEL_TPS = 5800 * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);
    private double lastTPS = 0;
    private double rpmCompensation = 0;
    private static final double SHOT_DROP_THRESHOLD_TPS = 80;
    private static final double RPM_BOOST_CLOSE = 450;  // within close launch zone
    private static final double RPM_BOOST_FAR   = 1400;  // fuck it we ball
    private static final double BOOST_DISTANCE_CLOSE = 40.0;
    private static final double BOOST_DISTANCE_FAR   = 80.0;
    private double desiredRPM;
    private double currentTPS;
    private double lastTargetTPS;
    private double maxTPSDrop;
    private int boostCounter = 0;


    private final ElapsedTime stationaryTimer = new ElapsedTime();
    private static final double STATIONARY_DEBOUNCE_MS = 200;

    // [Distance (Inches), Hood Position, Flywheel RPM]
    private final double[][] launchAngleLookupTable = {
            {21.1, .580, 2700},
            {30.1, .67, 2800},
            {40.4, .7, 2950},
            {51, .75, 3200},
            {60.3, .82, 3300},
            {70.1, .85, 3800},
            {80.5, .9, 3950},
            {90.7, .96, 4100},
            {100.6, .98, 4200},
            {111.8, 1, 4350},
            {120.3, 1, 4450},
            {130.1, 1, 4500},
            {140.2, 1, 4600}
    };

    public Turret(HardwareMap hwMap) {
        turretLeft = hwMap.get(CRServo.class, "turretLeft");
        turretRight = hwMap.get(CRServo.class, "turretRight");
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");
        turretEncoder = hwMap.get(DcMotorEx.class, "transfer");

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setVelocityPIDFCoefficients(14.5, 0, 3, 13.5);

        chassisCenterController.setReference(0);
        visionAimClose.setReference(0);
        visionAimCloseStationary.setReference(0);
        visionAimFar.setReference(0);
        visionAimFarStationary.setReference(0);
        timeSinceLostTimer.reset();
        stationaryTimer.reset();
    }

    public void updateTurret() {
        turretDeg = turretEncoder.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        double currentDistance = Limelight.getDistance();
        currentTPS = flywheelLeft.getVelocity();
        hood.setPosition(getServoPositionFromDistance(currentDistance));


        if (isFlywheelOn) {
            isFlywheelRunning = true;

            applyFlywheelPower(getRPMFromDistance(currentDistance));
        } else {
            isFlywheelRunning = false;
            applyFlywheelPower(0);
        }
    }

    public void aimTurret(boolean hasValidTarget, double limelightError, double manualTurnInput,
                          boolean isUsingModifiedCenter, double manualTargetAngle, boolean isChassisStationary) {
        double calculatedPower;

        if (Math.abs(manualTurnInput) > 0.05) {
            currentMode = "MANUAL";
            calculatedPower = manualTurnInput;
            timeSinceLostTimer.reset();
            timeSinceLost = 0;
        }
        else if (isUsingModifiedCenter) {
            currentMode = "HOLDING_POSITION";
            chassisCenterController.setReference(manualTargetAngle);
            calculatedPower = chassisCenterController.calculatePIDF(turretDeg);
            lastValidCalculatedPower = 0;
        }
        else {
            if (hasValidTarget) {

                timeSinceLostTimer.reset();
                timeSinceLost = 0;

                double errorToUse = wrapAngle(limelightError);


//                if (consecutiveValidFrames > 1 && Math.abs(rawError - lastRawError) > MAX_ERROR_JUMP_DEG) {
//                    rawError = lastRawError;
//                }
//                lastRawError = rawError;
//
//                // EMA smoothing. On the first valid frame after a gap, seed the
//                // filter directly so we don't converge from a stale value.
//                if (consecutiveValidFrames == 1) {
//                    filteredError = rawError;
//                } else {
//                    double alpha = isChassisStationary ? ERROR_ALPHA_STATIONARY : ERROR_ALPHA_MOVING;
//                    filteredError = alpha * rawError + (1.0 - alpha) * filteredError;
//                }
//
//                // Don't act until the target has been consistently visible for
//                // MIN_VALID_FRAMES — avoids reacting to a single-frame phantom detection.
//                if (consecutiveValidFrames < MIN_VALID_FRAMES) {
//                    currentMode = "ACQUIRING";
//                    applyTurretPower(lastValidCalculatedPower * 0.2);
//                    return;
//                }


                double currentDistance = Limelight.getDistance();

                if (!isChassisStationary) stationaryTimer.reset();
                boolean useStationary = isChassisStationary
                        && stationaryTimer.milliseconds() > STATIONARY_DEBOUNCE_MS;

                if (currentDistance > 70.0) {
                    double farPowerMoving     = visionAimFar.calculatePIDF(errorToUse);
                    double farPowerStationary = Math.abs(errorToUse) < 3.0
                            ? 0.0
                            : visionAimFarStationary.calculatePIDF(errorToUse);

                    if (useStationary) {
                        currentMode     = "VISION_TRACKING_FAR_STATIONARY";
                        calculatedPower = farPowerStationary;
                    } else {
                        currentMode     = "VISION_TRACKING_FAR";
                        calculatedPower = farPowerMoving;
                    }
                } else {
                    double closePowerMoving     = visionAimClose.calculatePIDF(errorToUse);
                    double closePowerStationary = Math.abs(errorToUse) < 1.5
                            ? 0.0
                            : visionAimCloseStationary.calculatePIDF(errorToUse);

                    if (useStationary) {
                        currentMode     = "VISION_TRACKING_CLOSE_STATIONARY";
                        calculatedPower = closePowerStationary;
                    } else {
                        currentMode     = "VISION_TRACKING_CLOSE";
                        calculatedPower = closePowerMoving;
                    }
                }

                lastValidCalculatedPower = calculatedPower;
            }
            else {
                timeSinceLost = timeSinceLostTimer.milliseconds();

                if (timeSinceLost < 300 && timeSinceLost != 0) {
                    currentMode     = "DEBOUNCE";
                    calculatedPower = 0;
                } else {
                    currentMode = "RETURN_TO_CENTER";
                    double errorToUse = wrapAngle(0.0 - turretDeg);
                    visionAimFarStationary.reset();
                    visionAimClose.reset();
                    visionAimFar.reset();
                    visionAimCloseStationary.reset();
                    calculatedPower = (Math.abs(errorToUse) < 3) ? 0 : -chassisCenterController.calculatePIDF(errorToUse);
                    lastValidCalculatedPower = 0;
                }
            }
        }

        applyTurretPower(calculatedPower);
    }
    private void applyTurretPower(double targetPower) {
        double delta = targetPower - lastValidCalculatedPower;
        if (Math.abs(delta) > TURRET_SLEW_RATE) {
            targetPower = lastValidCalculatedPower + Math.signum(delta) * TURRET_SLEW_RATE;
        }

        int currentTicks = turretEncoder.getCurrentPosition();
        if ((currentTicks >= TURRET_MAX_LIMIT_TICKS && targetPower > 0) || (currentTicks <= TURRET_MIN_LIMIT_TICKS && targetPower < 0)) {
            turretLeft.setPower(0);
            turretRight.setPower(0);
            lastValidCalculatedPower = 0;
        } else {
            turretLeft.setPower(targetPower);
            turretRight.setPower(targetPower);
            lastValidCalculatedPower = targetPower;
        }
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double getServoPositionFromDistance(double distance) {
        if (distance <= launchAngleLookupTable[0][0]) return launchAngleLookupTable[0][1];
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) return launchAngleLookupTable[launchAngleLookupTable.length - 1][1];

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lower = launchAngleLookupTable[i];
            double[] upper = launchAngleLookupTable[i+1];
            if (distance >= lower[0] && distance <= upper[0]) {
                return lower[1] + ((distance - lower[0]) / (upper[0] - lower[0])) * (upper[1] - lower[1]);
            }
        }
        return 0.5;
    }

    public double getRPMFromDistance(double distance) {
        if (distance == 0) return 2000;
        if (distance <= launchAngleLookupTable[0][0]) return launchAngleLookupTable[0][2];
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) return launchAngleLookupTable[launchAngleLookupTable.length - 1][2];

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lower = launchAngleLookupTable[i];
            double[] upper = launchAngleLookupTable[i+1];
            if (distance >= lower[0] && distance <= upper[0]) {
                return lower[2] + ((distance - lower[0]) / (upper[0] - lower[0])) * (upper[2] - lower[2]);
            }
        }
        return 2500;
    }
    private double getRPMCompensationFromDistance(double distance) {
        if (distance <= BOOST_DISTANCE_CLOSE) return RPM_BOOST_CLOSE;
        if (distance >= BOOST_DISTANCE_FAR)   return RPM_BOOST_FAR;
        double t = (distance - BOOST_DISTANCE_CLOSE) / (BOOST_DISTANCE_FAR - BOOST_DISTANCE_CLOSE);
        return RPM_BOOST_CLOSE + t * (RPM_BOOST_FAR - RPM_BOOST_CLOSE);
    }

    public void applyFlywheelPower(double targetRPM) {
        currentTPS = flywheelLeft.getVelocity();
        double targetTPS = targetRPM * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);

        double tpsDrop = lastTPS - currentTPS;

        double expectedDrop = lastTargetTPS - targetTPS;
        double unexplainedDrop = tpsDrop - Math.max(0, expectedDrop);

        if (unexplainedDrop > maxTPSDrop) maxTPSDrop = unexplainedDrop;

        if (isFlywheelOn && unexplainedDrop > SHOT_DROP_THRESHOLD_TPS && boostCounter == 0) {
            rpmCompensation += getRPMCompensationFromDistance(Limelight.getDistance());
            boostCounter++;
        }

        if (Math.abs(currentTPS - targetTPS) < 100) {
            rpmCompensation = 0;
            boostCounter = 0;
        }

        lastTPS = currentTPS;
        lastTargetTPS = targetTPS;

        double compensatedRPM = targetRPM + rpmCompensation;
        desiredRPM = compensatedRPM;
        double compensatedTPS = compensatedRPM * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);

        flywheelLeft.setVelocity(compensatedTPS);
        if (currentTPS < compensatedTPS - SHOT_DROP_THRESHOLD_TPS && ShooterSubsystem.shotStartTime > 100) {
            flywheelRight.setPower(1);
        } else {
            flywheelRight.setPower(compensatedTPS / MAX_FLYWHEEL_TPS);
        }
    }

    public void turnOnFlywheel() { isFlywheelOn = true; }
    public void turnOffFlywheel() { isFlywheelOn = false; }
    public double getTurretDeg() { return turretDeg; }

    //use theses when we aren't using the lookuptable for power
    public void forceOnFlywheel(double rpm) {
       applyFlywheelPower(rpm);
    }
    public void forceOffFlywheel()
    {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    public double getRealRPM()
    {
        return (flywheelLeft.getVelocity() * 60.0) / FLYWHEEL_TICKS_PER_REVOLUTION;
    }
    public double getRealPower()
    {
        return flywheelLeft.getPower();
    }

    public void resetEncoder() {

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret Mode", currentMode);
        telemetry.addData("Time Since Lost", "%.0f ms", timeSinceLost);
        telemetry.addData("Turret Angle", "%.2f°", turretDeg);
        telemetry.addData("Distance", Limelight.getDistance());
        telemetry.addData("Turret Power", turretLeft.getPower());
        telemetry.addData("Target RPM", desiredRPM);
        telemetry.addData("Real RPM", getRealRPM());
        telemetry.addData("Real Power Left", getRealPower());
        telemetry.addData("Real Power Right", flywheelRight.getPower());
        telemetry.addData("RPM Compensation", "%.0f", rpmCompensation);
        telemetry.addData("Max TPS Drop", "%.0f", maxTPSDrop);
    }
}