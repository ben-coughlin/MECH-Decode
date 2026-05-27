package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotMaster.inventory;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Pattern;

public class ColorSensor
{
    private NormalizedRGBA colorsUpper = null;
    private NormalizedRGBA colorsMiddle = null;
    private NormalizedRGBA colorsLower = null;
    private final RevColorSensorV3 upperColorSensor;
    private final RevColorSensorV3 middleColorSensor;
    private final RevColorSensorV3 lowerColorSensor;

    private boolean isUpperGreen;
    private boolean isUpperPurple;
    private boolean isMiddleGreen;
    private boolean isMiddlePurple;
    private boolean isLowerGreen;
    private boolean isLowerPurple;



    final float gain = 2;

    private final float[] hsvUpper = new float[4];
    private final float[] hsvMiddle = new float[4];
    private final float[] hsvLower = new float[4];



    public ColorSensor(HardwareMap hwMap)
    {
        upperColorSensor = hwMap.get(RevColorSensorV3.class, "upperColorSensor");
        middleColorSensor = hwMap.get(RevColorSensorV3.class, "middleColorSensor");
        lowerColorSensor = hwMap.get(RevColorSensorV3.class, "lowerColorSensor");

        upperColorSensor.setGain(gain);
        middleColorSensor.setGain(gain);
        lowerColorSensor.setGain(gain);

    }

    public void updateDetection()
    {
        colorsUpper = upperColorSensor.getNormalizedColors();
        hsvUpper[3] = (float) upperColorSensor.getDistance(DistanceUnit.CM);
        Color.colorToHSV(colorsUpper.toColor(), hsvUpper);


        colorsMiddle = middleColorSensor.getNormalizedColors();
        hsvMiddle[3] = (float) middleColorSensor.getDistance(DistanceUnit.CM);
        Color.colorToHSV(colorsMiddle.toColor(), hsvMiddle);

        colorsLower = lowerColorSensor.getNormalizedColors();
        hsvLower[3] = (float) lowerColorSensor.getDistance(DistanceUnit.CM);
        Color.colorToHSV(colorsLower.toColor(), hsvLower);

        inventory.updatePattern(detectBallColor(hsvUpper), detectBallColor(hsvMiddle), detectBallColor(hsvLower));
        updateColorBooleans();
    }


    public void showColorSensorTelemetry(Telemetry telemetry)
    {


        if (colorsLower != null)
        {
            telemetry.addLine("--- Lower Color Sensor ---");
            telemetry.addData("isPurple", isLowerPurple);
            telemetry.addData("isGreen", isLowerGreen);
            telemetry.addData("Red Value", colorsLower.red);
            telemetry.addData("Blue Value", colorsLower.blue);
            telemetry.addData("Green Value", colorsLower.green);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvLower[0])
                    .addData("Saturation", "%.3f", hsvLower[1])
                    .addData("Value", "%.3f", hsvLower[2]);
            telemetry.addData("Alpha", "%.3f", colorsLower.alpha);
        }
        else
        {
            telemetry.addData("Error", "Cannot communicate with lower color sensor.");
        }
        if (colorsMiddle != null)
        {
            telemetry.addLine("--- Middle Color Sensor ---");
            telemetry.addData("isPurple", isMiddlePurple);
            telemetry.addData("isGreen", isMiddleGreen);
            telemetry.addData("Red Value", colorsMiddle.red);
            telemetry.addData("Blue Value", colorsMiddle.blue);
            telemetry.addData("Green Value", colorsMiddle.green);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvMiddle[0])
                    .addData("Saturation", "%.3f", hsvMiddle[1])
                    .addData("Value", "%.3f", hsvMiddle[2]);
            telemetry.addData("Alpha", "%.3f", colorsMiddle.alpha);
        }
        else
        {
            telemetry.addData("Error", "Cannot communicate with middle color sensor.");
        }
        if (colorsUpper != null)
        {
            telemetry.addLine("--- Upper Color Sensor ---");
            telemetry.addData("isPurple", isUpperPurple);
            telemetry.addData("isGreen", isUpperGreen);
            telemetry.addData("Red Value", colorsUpper.red);
            telemetry.addData("Blue Value", colorsUpper.blue);
            telemetry.addData("Green Value", colorsUpper.green);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvUpper[0])
                    .addData("Saturation", "%.3f", hsvUpper[1])
                    .addData("Value", "%.3f", hsvUpper[2]);
            telemetry.addData("Alpha", "%.3f", colorsUpper.alpha);
        }
        else
        {
            telemetry.addData("Error", "Cannot communicate with upper color sensor.");
        }
    }



    /**
     * @return empty, purple, green or ballnotdetected if the spindexer is detected
     */
    public Pattern.Ball detectBallColor(float[] hsv) {
        float hue = hsv[0];
        float sat = hsv[1];


       if(hue < 192 && hue > 180)
       {
           return Pattern.Ball.BALL_NOT_DETECTED;
       }
       else if(hue > 200 && sat > 0.4)
       {
           return Pattern.Ball.PURPLE;
       }
       else if(hue > 115 && hue < 185 && sat > 0.4)
       {

           return Pattern.Ball.GREEN;
       }
       else {

           return Pattern.Ball.EMPTY;
       }

    }

    private void updateColorBooleans()
    {
        //ugly :(
        if(inventory.getUpper().equals(Pattern.Ball.GREEN))
        {
            isUpperGreen = true;
            isUpperPurple = false;
        }
        else if(inventory.getUpper().equals(Pattern.Ball.PURPLE))
        {
            isUpperGreen = false;
            isUpperPurple = true;
        }
        else {
            isUpperGreen = false;
            isUpperPurple = false;
        }

        // -
        if(inventory.getMiddle().equals(Pattern.Ball.GREEN))
        {
            isMiddleGreen = true;
            isMiddlePurple = false;
        }
        else if(inventory.getMiddle().equals(Pattern.Ball.PURPLE))
        {
            isMiddleGreen = false;
            isMiddlePurple = true;
        }
        else
        {
            isMiddleGreen = false;
            isMiddlePurple = false;
        }

        // -
        if(inventory.getLower().equals(Pattern.Ball.GREEN))
        {
            isLowerGreen = true;
            isLowerPurple = false;
        }
        else if(inventory.getLower().equals(Pattern.Ball.PURPLE))
        {
            isLowerGreen = false;
            isLowerPurple = true;
        }
        else
        {
            isLowerGreen = false;
            isLowerPurple = false;
        }

    }



    /**
     * @return 0 - hue, 1 - saturation, 2 - value, 3 - distance
     */
    public float[] getHsvValues()
    {
        return hsvUpper;
    }




}
