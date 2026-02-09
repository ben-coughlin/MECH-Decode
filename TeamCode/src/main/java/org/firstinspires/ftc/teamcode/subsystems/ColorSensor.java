package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Pattern;

public class ColorSensor
{
    private NormalizedRGBA colors = null;
    private final NormalizedColorSensor colorSensor;

    private boolean isPurple;
    private boolean isGreen;


    final float gain = 19;
    private final float[] hsv = new float[4];


    public ColorSensor(HardwareMap hwMap)
    {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color");
        if (colorSensor instanceof SwitchableLight)
        {
           // ((SwitchableLight) colorSensor).enableLight(false);
        }
        colorSensor.setGain(gain);
    }


    public void showColorSensorTelemetry(Telemetry telemetry)
    {



        if (colors != null)
        {
            telemetry.addLine("--- Color Sensor ---");
            telemetry.addData("isPurple", isPurple);
            telemetry.addData("isGreen", isGreen);
            telemetry.addData("Red Value", colors.red);
            telemetry.addData("Blue Value", colors.blue);
            telemetry.addData("Green Value", colors.green);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsv[0])
                    .addData("Saturation", "%.3f", hsv[1])
                    .addData("Value", "%.3f", hsv[2]);
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
        hsv[3] = (float)((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        Color.colorToHSV(colors.toColor(), hsv);
        detectBallColor();

    }

    /**
     * @return empty, purple, green or ballnotdetected if the spindexer is detected
     */
    public Pattern.Ball detectBallColor() {
        float hue = hsv[0];
        float sat = hsv[1];

       if(hue < 192 && hue > 180)
       {
           isPurple = false;
           isGreen = false;
           return Pattern.Ball.BALL_NOT_DETECTED;
       }
       else if(hue > 200 && sat > 0.4)
       {
           isPurple = true;
           isGreen = false;
           return Pattern.Ball.PURPLE;
       }
       else if(hue > 160 && hue < 175 && sat > 0.4)
       {
           isPurple = false;
           isGreen = true;
           return Pattern.Ball.GREEN;
       }
       else {
           isPurple = false;
           isGreen = false;
           return Pattern.Ball.EMPTY;
       }

    }

    /**
     * @return 0 - hue, 1 - saturation, 2 - value, 3 - distance
     */
    public float[] getHsvValues()
    {
        return hsv;
    }




}
