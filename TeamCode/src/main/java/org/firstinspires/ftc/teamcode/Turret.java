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

    private final DcMotorEx turret;
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final Servo hood;
    final PIDFController autoAim = new PIDFController(0.01, 0, 0.001, 0.05);
    private int turretPos;
    private double turretPower;
    private double flywheelLeftRPM;

    private double llError = 0;


    private double flywheelRightRPM;

    private double flywheelLeftPower;
    private double flywheelRightPower;
    private double hoodPos;
    private final double[][] launchAngleLookupTable = {
            { 24, 0.3 },   // At 24 inches, servo position is 0.3
            { 36, 0.4 },   // At 36 inches, servo position is 0.4
            { 48, 0.55 },  // At 48 inches, servo position is 0.55
            { 60, 0.7 }    // At 60 inches, servo position is 0.7 //todo: tune
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
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        autoAim.setReference(0);

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

        int flywheelCountsPerRev = 8192; //REV-11-127 through bore encoder
        flywheelLeftRPM = (flywheelLeftVelocity / flywheelCountsPerRev) * 60;
        flywheelRightRPM = (flywheelRightVelocity / flywheelCountsPerRev) * 60;

        if (Limelight.getDistance() > 0) {
            double servoPosition = getServoPositionFromDistance(Limelight.getDistance());
            hood.setPosition(servoPosition);
        }

    }

    public void aimTurret(boolean useAutoAim, double limelightError, double manualTurnInput)
    {
        double calculatedPower;

        if(useAutoAim)
        {
            llError = limelightError;

            // If the error is > 180 degrees, it's shorter to go the other way.
            if (llError > 180) {
                llError -= 360;
            } else if (llError < -180) {
                llError += 360;
            }

            calculatedPower = -autoAim.calculatePIDF(llError);
        }
        else
        {

            calculatedPower = manualTurnInput;
            autoAim.reset();
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

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Position", turretPos);
        telemetry.addData("LLError", llError);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Servo Position", "%.2f", hood.getPosition());

    }

    public int getTurretPos()
    {
        return turretPos;
    }

    public void setTurretPos(int turretPos)
    {
        this.turretPos = turretPos;
    }
    public double getTurretPower()
    {
        return turretPower;
    }
    public void setTurretPower(double turretPower)
    {
        turret.setPower(turretPower);
    }

    public double getFlywheelLeftRPM()
    {
        return flywheelLeftRPM;
    }

    public double getFlywheelLeftPower()
    {
        return flywheelLeftPower;
    }

    public void setFlywheelLeftPower(double flywheelLeftPower)
    {
        flywheelLeft.setPower(flywheelLeftPower);
    }
    public double getFlywheelRightRPM()
    {
        return flywheelRightRPM;
    }

    public void setFlywheelRightRPM(double flywheelRightRPM)
    {
        this.flywheelRightRPM = flywheelRightRPM;
    }
    public double getFlywheelRightPower()
    {
        return flywheelRightPower;
    }

    public void setFlywheelRightPower(double flywheelRightPower)
    {
        flywheelRight.setPower(flywheelRightPower);
    }
    public double getHoodPos()
    {
        return hoodPos;
    }
    public void setHoodPos(double hoodPos)
    {
        hood.setPosition(hoodPos);
    }
    public void setFlywheelPower(double flywheelPower)
    {
        setFlywheelRightPower(flywheelPower);
        setFlywheelLeftPower(flywheelPower);
    }

}

