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
     final PIDController autoAim = new PIDController(0.023, 0.001, 0.3);
    private int turretPos;
    private double turretPower;
    private double flywheelLeftRPM;

    private double llError = 0;


    private double flywheelRightRPM;

    private double flywheelLeftPower;
    private double flywheelRightPower;
    private double hoodPos;


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
        flywheelLeftRPM = (flywheelLeftVelocity * 60 / flywheelCountsPerRev);
        flywheelRightRPM = (flywheelRightVelocity * 60 / flywheelCountsPerRev);

    }

    public void aimTurret(boolean useAutoAim, double limelightError, double manualTurnInput)
    {
        double turretPower;

        if(useAutoAim)
        {
            llError = limelightError;
            turretPower = -autoAim.calculatePID(limelightError);
        }
        else
        {
            turretPower = manualTurnInput;
            autoAim.reset();
        }

        turret.setPower(turretPower);
    }

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Position", turretPos);
        telemetry.addData("LLError", llError);
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

