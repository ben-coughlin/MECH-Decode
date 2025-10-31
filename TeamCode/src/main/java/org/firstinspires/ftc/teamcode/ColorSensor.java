package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor
{
    private NormalizedRGBA colors = null;
    private final NormalizedColorSensor colorSensor;

    private boolean isPurple;
    private boolean isGreen;

    final double purpleThreshold = 0.003;
    final double greenThreshold = 0.002;
    final float gain = 2;
    private final float[] hsvValues = new float[3];


    public ColorSensor(HardwareMap hwMap)
    {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color");
        if (colorSensor instanceof SwitchableLight)
        {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        colorSensor.setGain(gain);
    }


    public void showColorSensorTelemetry(Telemetry telemetry)
    {

        Color.colorToHSV(colors.toColor(), hsvValues);

        if (colors != null)
        {
            telemetry.addData("isPurple", isPurple);
            telemetry.addData("isGreen", isGreen);
            telemetry.addData("Red Value", colors.red);
            telemetry.addData("Blue Value", colors.blue);
            telemetry.addData("Green Value", colors.green);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
        }
        else
        {
            telemetry.addData("Error", "Cannot communicate with color sensor.");
        }
    }

    public void updateDetection()
    {
        colors = colorSensor.getNormalizedColors();

        if (colors.green > colors.red * 2 && colors.green > colors.blue * 2)
        {
            isPurple = true;
            isGreen = false;
        }
        else if (colors.green > greenThreshold && colors.red < 0.0015)
        {
            isGreen = true;
            isPurple = false;
        }
        else
        {
            isGreen = false;
            isPurple = false;
        }
    }

    public float[] getHsvValues()
    {
        return hsvValues;
    }



}
