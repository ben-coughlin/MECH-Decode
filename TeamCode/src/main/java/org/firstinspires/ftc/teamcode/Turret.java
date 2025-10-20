package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret
{
    private final DcMotorEx turret;
    private final DcMotorEx flywheel;
    private final Servo hood;
    private final PIDController autoAim = new PIDController(0.005, 0, 0);
    private int turretPos;
    private double flywheelRPM;
    private double flywheelPower;
    private double hoodPos;

    public Turret(HardwareMap hwMap)
    {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        hood = hwMap.get(Servo.class, "hood");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void updateTurret()
    {
        turretPos = turret.getCurrentPosition();
        double flywheelVelocity = flywheel.getVelocity();

        hoodPos = hood.getPosition();
        flywheelPower = flywheel.getPower();

        int flywheelCountsPerRev = 8192; //REV-11-127 through bore encoder
        flywheelRPM = (flywheelVelocity * 60 / flywheelCountsPerRev);

    }

    public void aimTurret(boolean useAutoAim)
    {
        if(!useAutoAim)
        {
            return; //deal with this later!!
        }
        else
        {

        }


    }




    public int getTurretPos()
    {
        return turretPos;
    }

    public void setTurretPos(int turretPos)
    {
        this.turretPos = turretPos;
    }

    public double getFlywheelRPM()
    {
        return flywheelRPM;
    }

    public double getFlywheelPower()
    {
        return flywheelPower;
    }

    public void setFlywheelPower(double flywheelPower)
    {
        this.flywheelPower = flywheelPower;
    }

    public double getHoodPos()
    {
        return hoodPos;
    }

    public void setHoodPos(double hoodPos)
    {
        this.hoodPos = hoodPos;
    }
}
