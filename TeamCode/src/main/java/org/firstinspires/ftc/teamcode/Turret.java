package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret
{
    private final double TURRET_TICKS_PER_DEGREE = 2.563; // 384.5 PPR encoder with 2.4:1 gear reduction (48/20 gears)
    private final double TURRET_MIN_LIMIT_TICKS = -450;
    private final double TURRET_MAX_LIMIT_TICKS = 450;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 8192; //rev throughbore


    private final DcMotorEx turret;
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final Servo hood;
    private final VoltageSensor voltageSensor;
    private final PIDFController autoAimClose = new PIDFController(0.012, 0, 0.0012, 0.06);
    private final PIDFController autoAimFar = new PIDFController(0.0065, 0, 0.0008, 0.05);
    private final PIDFController flywheelPIDF = new PIDFController(0.0008, 0, 0.0001, 0.00018); //todo: tuneeee
    private  PIDFController activeAutoAimController;

    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 72.0;

    private double turretDeg;
    private double turretPower;
    private double llError = 0;
    private double flywheelPower;
    private double hoodPos;
    private double currRPM;
    private double targetRPM;


    private final double[][] launchAngleLookupTable = {
            { 20, 0.01, 4200},   // At 20 inches, servo is 0.01, % power is 80
            { 36, 0.08, 4680},
            { 48, 0.105, 5550 },
            { 60, 0.155, 6000 },
            { 65, 0.210, 6000 },
            { 110, 0.83, 6000 }   //todo: tune
    };

    public Turret(HardwareMap hwMap)
    {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft"); //todo: REMEMBER WHICH PORT THE ENCODER IS IN!!! also check that voltage sensor
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");

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

        activeAutoAimController = autoAimClose;
        autoAimFar.setReference(0);
        autoAimClose.setReference(0);

    }

    public void updateTurret()
    {
        turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        turretPower = turret.getPower();

        double ticksPerSecond = flywheelLeft.getVelocity();
        currRPM = (ticksPerSecond / FLYWHEEL_TICKS_PER_REVOLUTION) * 60.0;


        double currentDistance = Limelight.getDistance();
        if (currentDistance > 0) {
            hood.setPosition(getServoPositionFromDistance(currentDistance));
            targetRPM = getRPMFromDistance(currentDistance);
        }
        updateFlywheel();

        hoodPos = hood.getPosition();
        flywheelPower = flywheelLeft.getPower();

    }


    public double setPowerFromRPM(double rpm)
    {
        return rpm / 6000;
    }

    public void setFlywheelPower(double flywheelPower)
    {
        flywheelLeft.setPower(flywheelPower);
        flywheelRight.setPower(flywheelPower);
    }

    public void turnOnFlywheel()
    {
        setFlywheelPower(flywheelPower);
    }
    public void turnOffFlywheel()
    {
        setFlywheelPower(0);
    }


    public void updateFlywheel() {
        double voltage = voltageSensor.getVoltage();

        double kF_comp = flywheelPIDF.getF() * (12.0 / voltage);
        flywheelPIDF.setF(kF_comp);

        flywheelPIDF.setReference(targetRPM);

        flywheelPower = flywheelPIDF.calculatePIDF(currRPM);

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


            if (distance < GAIN_SCHEDULING_DISTANCE_THRESHOLD && distance > 0) {
                activeAutoAimController = autoAimClose;
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
            autoAimClose.reset();
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
        telemetry.addLine("--- Aim ---");
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Degrees", turretDeg);
        telemetry.addData("LLError", llError);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Target RPM", getRPMFromDistance(Limelight.getDistance()));
        telemetry.addData("Interpolated Servo Position", "%.2f", getServoPositionFromDistance(Limelight.getDistance()));
        telemetry.addData("Flywheel Current RPM", "%.2f", currRPM);
       // telemetry.addData("Flywheel Velocity" , "%.2f", flywheelLeft.getVelocity());

    }

    public double getTurretDeg() { return turretDeg; }
    public double getTurretPower() { return turretPower; }
    public void setTurretPower(double turretPower) { turret.setPower(turretPower); }
    public double getFlywheelRPM() { return currRPM; }
    public void setFlywheelLeftPower(double flywheelLeftPower) { flywheelLeft.setPower(flywheelLeftPower); }
    public void setFlywheelRightPower(double flywheelRightPower) { flywheelRight.setPower(flywheelRightPower); }
    public double getHoodPos() { return hoodPos; }
    public void setHoodPos(double hoodPos) { hood.setPosition(hoodPos); }
}
