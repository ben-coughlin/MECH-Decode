package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret
{
    private final double TURRET_TICKS_PER_DEGREE = 4.27; //gobilda 435 rpm - 5203-2402-0014
    private final double TURRET_MIN_LIMIT_TICKS = -450;
    private final double TURRET_MAX_LIMIT_TICKS = 450;

    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 8192; //rev throughbore

    private final DcMotorEx turret;
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final Servo hood;
    private final PIDFController autoaimClose = new PIDFController(0.012, 0, 0.001, 0.05);

    private final PIDFController autoAimFar = new PIDFController(0.008, 0, 0, 0.05); // Lower P-gain and zero D-gain are key

    private final PIDFController activeAutoAimController;
    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 72.0;

    private int turretPos;
    private double turretPower;
    private double flywheelLeftRPM;
    private double llError = 0;
    private double flywheelRightRPM;
    private double flywheelLeftPower;
    private double flywheelRightPower;
    private double hoodPos;
    // The lookup table now includes [distance, hood_position, target_rpm]
    private final double[][] launchAngleLookupTable = {
            { 24, 0.3, 3800 },   // At 24 inches, servo is 0.3, RPM is 3800
            { 36, 0.4, 4000 },   // At 36 inches, servo is 0.4, RPM is 4000
            { 48, 0.55, 4200 },  // At 48 inches, servo is 0.55, RPM is 4200
            { 60, 0.7, 4500 }    // At 60 inches, servo is 0.7, RPM is 4500 //todo: tune
    };

    public Turret(HardwareMap hwMap)
    {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        activeAutoAimController = autoaimClose;
        autoAimFar.setReference(0);
        autoaimClose.setReference(0);
    }

    public void updateTurret()
    {
        turretPos = turret.getCurrentPosition();
        turretPower = turret.getPower();

        double flywheelLeftVelocity = flywheelLeft.getVelocity();
        double flywheelRightVelocity = flywheelRight.getVelocity();

        hoodPos = hood.getPosition();
        flywheelLeftPower = flywheelLeft.getPower();
        flywheelRightPower = flywheelRight.getPower();

        flywheelLeftRPM = (flywheelLeftVelocity / FLYWHEEL_TICKS_PER_REVOLUTION) * 60;
        flywheelRightRPM = (flywheelRightVelocity / FLYWHEEL_TICKS_PER_REVOLUTION) * 60;


        /*
        if (Limelight.getDistance() > 0) {
            double servoPosition = getServoPositionFromDistance(Limelight.getDistance());
            hood.setPosition(servoPosition);
        }
        */
    }

    /**
     * Sets the target velocity of both flywheel motors.
     * @param rpm The desired target RPM.
     */
    public void setFlywheelRPM(double rpm) {
        double ticksPerSecond = (rpm * FLYWHEEL_TICKS_PER_REVOLUTION) / 60.0;
        flywheelLeft.setVelocity(ticksPerSecond);
        flywheelRight.setVelocity(ticksPerSecond);
    }

    // This method is now primarily for stopping the flywheel or for manual overrides.
    public void setFlywheelPower(double flywheelPower)
    {
        // If stopping, use setVelocity(0) for a more definitive stop in RUN_USING_ENCODER mode.
        if (flywheelPower == 0) {
            setFlywheelRPM(0);
        } else {

            flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelLeft.setPower(flywheelPower);
            flywheelRight.setPower(flywheelPower);
        }
    }


    /**
     * Aims the turret using either auto-aim or manual input, and selects PIDF gains based on distance.
     * @param useAutoAim        Boolean to enable/disable auto-aim.
     * @param limelightError    The 'tx' error from the Limelight.
     * @param manualTurnInput   Manual joystick input.
     * @param distance          The distance to the target, used for gain scheduling.
     */
    public void aimTurret(boolean useAutoAim, double limelightError, double manualTurnInput, double distance)
    {
        double calculatedPower;

        if(useAutoAim)
        {

            PIDFController activeAutoAimController;
            if (distance < GAIN_SCHEDULING_DISTANCE_THRESHOLD && distance > 0) {
                activeAutoAimController = autoaimClose;
            } else {
                activeAutoAimController = autoAimFar;
            }

            llError = limelightError;

            if (llError > 180) llError -= 360;
            else if (llError < -180) llError += 360;

            calculatedPower = -activeAutoAimController.calculatePIDF(llError);
        }
        else
        {
            calculatedPower = manualTurnInput;
            autoaimClose.reset();
            autoAimFar.reset();
        }

        int currentPosition = turret.getCurrentPosition();
        if (currentPosition >= TURRET_MAX_LIMIT_TICKS && calculatedPower > 0) {
            turret.setPower(0);
        }
        else if (currentPosition <= TURRET_MIN_LIMIT_TICKS && calculatedPower < 0) {
            turret.setPower(0);
        }
        else {
            turret.setPower(calculatedPower);
        }
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
     * Uses the lookup table and linear interpolation to find the correct target RPM.
     * @param distance The current distance to the target.
     * @return The calculated target RPM.
     */
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

                return lowerBound[2] + (distanceRatio * rpmRange);
            }
        }

        return 6000; //full send if this fails
    }

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Position", turretPos);
        telemetry.addData("LLError", llError);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Servo Position", "%.2f", hood.getPosition());
        telemetry.addData("Flywheel Left RPM", "%.2f", getFlywheelLeftRPM());
        telemetry.addData("Flywheel Right RPM", "%.2f", getFlywheelRightRPM());
    }

    public int getTurretPos() { return turretPos; }
    public void setTurretPos(int turretPos) { this.turretPos = turretPos; }
    public double getTurretPower() { return turretPower; }
    public void setTurretPower(double turretPower) { turret.setPower(turretPower); }
    public double getFlywheelLeftRPM() { return flywheelLeftRPM; }
    public double getFlywheelLeftPower() { return flywheelLeftPower; }
    public void setFlywheelLeftPower(double flywheelLeftPower) { flywheelLeft.setPower(flywheelLeftPower); }
    public double getFlywheelRightRPM() { return flywheelRightRPM; }
    public void setFlywheelRightRPM(double flywheelRightRPM) { this.flywheelRightRPM = flywheelRightRPM; }
    public double getFlywheelRightPower() { return flywheelRightPower; }
    public void setFlywheelRightPower(double flywheelRightPower) { flywheelRight.setPower(flywheelRightPower); }
    public double getHoodPos() { return hoodPos; }
    public void setHoodPos(double hoodPos) { hood.setPosition(hoodPos); }
}
