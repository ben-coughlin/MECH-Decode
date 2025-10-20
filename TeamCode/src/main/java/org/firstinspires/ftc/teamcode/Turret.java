package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret
{
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private Servo hood;
    private int turretPos;
    private int flywheelRPM;
    private double flywheelPower;
    private double hoodPos;
    public Turret(HardwareMap hwMap)
    {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        hood = hwMap.get(Servo.class, "hood");


    }

    public void updateTurret()
    {
        turretPos = turret.getCurrentPosition();
        flywheelRPM = flywheel.getCurrentPosition();
        hoodPos = hood.getPosition();
        flywheelPower = flywheel.getPower();

    }

    public int getTurretPos()
    {
        return turretPos;
    }

    public void setTurretPos(int turretPos)
    {
        this.turretPos = turretPos;
    }

    public int getFlywheelRPM()
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
