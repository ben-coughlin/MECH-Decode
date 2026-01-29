package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Breakbeam {
    private final DigitalChannel breakbeam;

    public Breakbeam(HardwareMap hwMap)
    {
        breakbeam = hwMap.get(DigitalChannel.class, "breakbeam");
    }

    public boolean getBreakbeamStatus()
    {
        return breakbeam.getState();
    }

    public void displayBreakbeamTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Is breakbeam broken? ", breakbeam.getState());
    }



}
