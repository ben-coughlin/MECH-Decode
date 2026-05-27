package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

public class Turret {

    // Hardware Constants
    private final double TURRET_TICKS_PER_DEGREE = 67.06899;
    private final double TURRET_MIN_LIMIT_TICKS = -12009;
    private final double TURRET_MAX_LIMIT_TICKS = 12009;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 28;
    private final double TURRET_SLEW_RATE = 0.12;

    private double lastValidCalculatedPower = 0.0;
    private final CRServo turretLeft, turretRight;
    private final DcMotorEx flywheelLeft, flywheelRight;
    private final DcMotorEx turretEncoderModule;
    private final Servo hood;

    private double turretDeg;
    private double timeSinceLost = 0;
    private double targetRPM;
    private boolean isFlywheelOn;
    public static boolean isFlywheelRunning = false;
    private String currentMode = "IDLE";

    private final PIDFController visionAimClose = new PIDFController(0.015, 0.0, 0.001, 0.0);
    private final PIDFController visionAimFar = new PIDFController(0.017, 0.0, 0.001, 0.0);
    private final PIDFController chassisCenterController = new PIDFController(0.015, 0.0, 0.0008, 0.0);
    private final ElapsedTime compensationTimer = new ElapsedTime();

    // [Distance (Inches), Hood Position, Flywheel RPM]
    private final double[][] launchAngleLookupTable = {
            { 23, .600, 2500},
            { 29.68, .630, 2550},
            { 39.58, .7, 2700},
            { 50.28, .73, 2900},
            { 60.63, .82, 3000},
            { 71, .86, 3100},
            { 80, .92, 3250},
            { 89.99, .94, 3450},
            { 103, .99, 3600},
            { 112, 1,  3700},
            {122, 1, 3850}
    };

    public Turret(HardwareMap hwMap) {
        turretLeft = hwMap.get(CRServo.class, "turretLeft");
        turretRight = hwMap.get(CRServo.class, "turretRight");
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");
        turretEncoderModule = hwMap.get(DcMotorEx.class, "transfer");

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setVelocityPIDFCoefficients(10.0, 0.15, 1.2, 11.0);

        chassisCenterController.setReference(0);
        visionAimClose.setReference(0);
        visionAimFar.setReference(0);
        compensationTimer.reset();
    }

    public void updateTurret() {
        turretDeg = turretEncoderModule.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        double currentDistance = Limelight.getDistance();

        hood.setPosition(getServoPositionFromDistance(currentDistance));
        targetRPM = getRPMFromDistance(currentDistance);

        if (isFlywheelOn) {
            isFlywheelRunning = true;
            double targetTPS = targetRPM * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);
            flywheelLeft.setVelocity(targetTPS);
            flywheelRight.setPower(flywheelLeft.getPower());
        } else {
            isFlywheelRunning = false;
            flywheelLeft.setVelocity(0);
            flywheelRight.setPower(0);
        }
    }

    public void aimTurret(boolean hasValidTarget, double limelightError, double manualTurnInput) {
        double calculatedPower;

        if (Math.abs(manualTurnInput) > 0.05) {
            currentMode = "MANUAL";
            calculatedPower = manualTurnInput;
            compensationTimer.reset();
            timeSinceLost = 0;
        }
        else {
            if (hasValidTarget) {
                compensationTimer.reset();
                timeSinceLost = 0;

                double errorToUse = wrapAngle(limelightError);
                double currentDistance = Limelight.getDistance();


                if (currentDistance > 60.0) {
                    currentMode = "VISION_TRACKING_FAR";
                    calculatedPower = visionAimFar.calculatePIDF(errorToUse);
                } else {
                    currentMode = "VISION_TRACKING_CLOSE";
                    calculatedPower = visionAimClose.calculatePIDF(errorToUse);
                }

                if (Math.abs(errorToUse) > 1.0) {
                    double kF = 0.04;
                    calculatedPower += Math.signum(errorToUse) * kF;
                }

                lastValidCalculatedPower = calculatedPower;
            }
            else {
                timeSinceLost = compensationTimer.milliseconds();

                if (timeSinceLost < 300) {
                    currentMode = "COAST_DEBOUNCE";
                    calculatedPower = lastValidCalculatedPower * 0.75;
                }
                else {
                    currentMode = "RETURN_TO_CENTER";
                    double errorToUse = wrapAngle(0.0 - turretDeg);

                    calculatedPower = -chassisCenterController.calculatePIDF(errorToUse);
                    lastValidCalculatedPower = 0;
                }
            }
        }

        applyTurretPower(Range.clip(calculatedPower, -0.65, 0.65));
    }
    private void applyTurretPower(double targetPower) {
        // Slew rate implementation prevents instant violent direction changes
        double delta = targetPower - lastValidCalculatedPower;
        if (Math.abs(delta) > TURRET_SLEW_RATE) {
            targetPower = lastValidCalculatedPower + Math.signum(delta) * TURRET_SLEW_RATE;
        }

        int currentTicks = turretEncoderModule.getCurrentPosition();
        if ((currentTicks >= TURRET_MAX_LIMIT_TICKS && targetPower > 0) || (currentTicks <= TURRET_MIN_LIMIT_TICKS && targetPower < 0)) {
            turretLeft.setPower(0);
            turretRight.setPower(0);
            lastValidCalculatedPower = 0;
        } else {
            turretLeft.setPower(targetPower);
            turretRight.setPower(targetPower);
            lastValidCalculatedPower = targetPower; // Updates frame anchor
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
        if (distance == 0) return 4000;
        if (distance <= launchAngleLookupTable[0][0]) return launchAngleLookupTable[0][2];
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) return launchAngleLookupTable[launchAngleLookupTable.length - 1][2];

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lower = launchAngleLookupTable[i];
            double[] upper = launchAngleLookupTable[i+1];
            if (distance >= lower[0] && distance <= upper[0]) {
                return Range.clip(lower[2] + ((distance - lower[0]) / (upper[0] - lower[0])) * (upper[2] - lower[2]), 3000, 6000);
            }
        }
        return 4500;
    }

    public void turnOnFlywheel() { isFlywheelOn = true; }
    public void turnOffFlywheel() { isFlywheelOn = false; }
    public double getTurretDeg() { return turretDeg; }

    //use theses when we aren't using the lookuptable for power
    public void forceOnFlywheel(double rpm) {
        double targetTPS = rpm * (FLYWHEEL_TICKS_PER_REVOLUTION / 60.0);
        flywheelLeft.setVelocity(targetTPS);
        flywheelRight.setPower(flywheelLeft.getPower());
    }
    public void forceOffFlywheel()
    {
        flywheelLeft.setVelocity(0);
        flywheelRight.setPower(0);
    }

    public double getRealRPM()
    {
        return flywheelLeft.getVelocity() / (FLYWHEEL_TICKS_PER_REVOLUTION * 60);
    }




    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret Mode", currentMode);
        telemetry.addData("Time Since Lost", "%.0f ms", timeSinceLost);
        telemetry.addData("Turret Angle", "%.2f°", turretDeg);
        telemetry.addData("Turret Power", turretLeft.getPower());
        telemetry.addData("Target RPM", targetRPM);
    }
}