package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {
    private NormalizedRGBA colors = null;
    private RevColorSensorV3 colorSensor;

    private boolean isPurple;
    private boolean isGreen;

    final double purpleThreshold = 0.003;
    final double greenThreshold = 0.002;
    final float gain = 2;


    public ColorSensor (HardwareMap hwMap) {
        colorSensor = hwMap.get(RevColorSensorV3.class, "artifactSensor");
        if(colorSensor instanceof SwitchableLight)
        {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        colorSensor.setGain(gain);
    }


    public void showColorSensorTelemetry(Telemetry telemetry)
    {
        telemetry.addData("isPurple", isPurple);
        telemetry.addData("isGreen", isGreen);
        telemetry.addData("Red Value", colors.red);
        telemetry.addData("Blue Value", colors.blue);
        telemetry.addData("Green Value", colors.green);
    }
    public void updateDetection()
    {
        colors = colorSensor.getNormalizedColors();

        if (colors.red > purpleThreshold && colors.blue > purpleThreshold) {
            isPurple = true;
            isGreen = false;
        }
        else if (colors.green > greenThreshold && colors.red < 0.0015) {
            isGreen = true;
            isPurple = false;
        }
        else {
            isGreen = false;
            isPurple = false;
        }
    }


    public boolean isPurple() {
        return isPurple;
    }

    public boolean isGreen() {
        return isGreen;
    }





}
