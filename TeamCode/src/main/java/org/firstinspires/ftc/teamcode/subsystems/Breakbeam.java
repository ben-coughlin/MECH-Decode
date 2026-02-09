package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Breakbeam {
    private final DigitalChannel intake;
    private final DigitalChannel turret;
    public static boolean intakeState = false;
    public static boolean turretState = false;


    public Breakbeam(HardwareMap hwMap)
    {
        intake = hwMap.get(DigitalChannel.class, "intakeBreakbeam");
        turret = hwMap.get(DigitalChannel.class, "turretBreakbeam");
    }

    //statuses are flipped for some reason
    public boolean getIntakeBreakbeamStatus()
    {
        intakeState = !intake.getState();
        return !intake.getState();
    }
    public boolean getTurretBreakbeamStatus()
    {
        turretState = !turret.getState();
        return !turret.getState();
    }



    public void displayBreakbeamTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Is intake breakbeam broken? ", intake.getState());
        telemetry.addData("Is turret breakbeam broken? ", turret.getState());
    }



}
