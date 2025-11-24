package org.firstinspires.ftc.teamcode;


import android.graphics.PixelFormat;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Base64;

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
    private final PIDFController autoaimClose = new PIDFController(0.012, 0, 0.001, 0.06);

    private final PIDFController autoAimFar = new PIDFController(0.006, 0, 0, 0.05);
    private final PIDFController flywheel = new PIDFController(0.0005, 0, 0, 0.00017);


    private  PIDFController activeAutoAimController;
    private final VoltageSensor battVoltage;
    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 72.0;

    private double turretDeg;
    private double turretPower;
    private double llError = 0;
    private double flywheelLeftPower;
    private double flywheelRightPower;

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
        flywheelLeft = hwMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hwMap.get(DcMotorEx.class, "flywheelRight");
        hood = hwMap.get(Servo.class, "hood");


        battVoltage = hwMap.voltageSensor.iterator().next();

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
        flywheel.setOutputLimits(0, 1);


        activeAutoAimController = autoaimClose;
        autoAimFar.setReference(0);
        autoaimClose.setReference(0);

    }


    public void updateTurret()
    {
        turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        turretPower = turret.getPower();


      //  double degreesPerSecond = flywheelLeft.getVelocity();
        currRPM = 4000;

        // 2. Set the PIDF reference to our target
        flywheel.setReference(targetRPM);

        // 3. Calculate the power needed using your PIDF controller
        double calculatedPower = flywheel.calculatePIDF(currRPM);

        flywheelLeft.setPower(calculatedPower);
        flywheelRight.setPower(calculatedPower);



       double currentDistance = Limelight.getDistance();
        if (currentDistance > 0) {
            hood.setPosition(getServoPositionFromDistance(currentDistance));
        }


        hoodPos = hood.getPosition();
        flywheelLeftPower = flywheelLeft.getPower();
        flywheelRightPower = flywheelRight.getPower();
    }




    public void turnOnFlywheel() {
        targetRPM = getRPMFromDistance(Limelight.getDistance());
    }


    public void turnOffFlywheel()
    {
        // To turn off, just set the target RPM to 0. The PIDF will spin it down.
        targetRPM = 0;
        flywheelLeft.setPower(0); // Also command power to 0 for an immediate stop.
        flywheelRight.setPower(0);
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

        return 0.5; //safety if we have issues
    }

    /**
     * Uses the lookup table and linear interpolation to find the correct target RPM.
     * @param distance The current distance to the target.
     * @return The calculated target RPM.
     */
    public double getRPMFromDistance(double distance) {
        double rpm;

        if (distance <= launchAngleLookupTable[0][0]) {
            rpm = launchAngleLookupTable[0][2];
        } else if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) {
            rpm = launchAngleLookupTable[launchAngleLookupTable.length - 1][2];
        } else {
            rpm = 6000;

            for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
                double[] lowerBound = launchAngleLookupTable[i];
                double[] upperBound = launchAngleLookupTable[i+1];

                if (distance >= lowerBound[0] && distance <= upperBound[0]) {
                    double distanceRange = upperBound[0] - lowerBound[0];
                    double powerRange = upperBound[2] - lowerBound[2];
                    double distanceRatio = (distance - lowerBound[0]) / distanceRange;

                    rpm = lowerBound[2] + (distanceRatio * powerRange);
                    break;
                }
            }
        }

        double currentVoltage = battVoltage.getVoltage();

        if (currentVoltage < 10.0) {
            return rpm;
        }

        final double NOMINAL_VOLTAGE = 12.0;
        double compensationFactor = NOMINAL_VOLTAGE / currentVoltage;

        return rpm * compensationFactor;
    }

    public double convertRPMToDegreesPerSec(double rpm)
    {
        return rpm * 6;
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
    public double getFlywheelLeftPower() { return flywheelLeftPower; }
    public void setFlywheelLeftPower(double flywheelLeftPower) { flywheelLeft.setPower(flywheelLeftPower); }
    public double getFlywheelRightPower() { return flywheelRightPower; }
    public void setFlywheelRightPower(double flywheelRightPower) { flywheelRight.setPower(flywheelRightPower); }
    public double getHoodPos() { return hoodPos; }
    public void setHoodPos(double hoodPos) { hood.setPosition(hoodPos); }
}
