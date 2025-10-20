package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret
{
    private final DcMotorEx turret;
    private final DcMotorEx flywheel;
    private final Servo hood;
    private final PIDController autoAim = new PIDController(0.005, 0, 0);
    private int turretPos;
    private double turretPower;
    private double flywheelRPM;
    private double flywheelPower;
    private double hoodPos;

    public Turret(HardwareMap hwMap)
    {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        hood = hwMap.get(Servo.class, "hood");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        autoAim.setReference(0);


    }

    public void updateTurret()
    {
        turretPos = turret.getCurrentPosition();
        turretPower = turret.getPower();
        double flywheelVelocity = flywheel.getVelocity();

        hoodPos = hood.getPosition();
        flywheelPower = flywheel.getPower();

        int flywheelCountsPerRev = 8192; //REV-11-127 through bore encoder
        flywheelRPM = (flywheelVelocity * 60 / flywheelCountsPerRev);

    }

    public void aimTurret(boolean useAutoAim, double limelightError, double manualTurnInput)
    {
        double turretPower;

        if(!useAutoAim)
        {
            turretPower = manualTurnInput;
            autoAim.reset();
        }
        else
        {
            turretPower = autoAim.calculatePID(limelightError);
        }

        turret.setPower(turretPower);

    }
    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Position", turretPos);
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
    public void setTurretPower(double v)
    {
        this.turretPower = turretPower;
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
