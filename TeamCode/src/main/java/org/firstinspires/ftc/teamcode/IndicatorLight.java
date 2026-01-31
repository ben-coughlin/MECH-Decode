package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IndicatorLight {
    private static Servo indicatorLight;


    public IndicatorLight(HardwareMap hwMap)
    {
        indicatorLight = hwMap.get(Servo.class, "light");
    }

    // all these positions are from the gobilda website: https://www.gobilda.com/rgb-indicator-light-pwm-controlled
    public static void setLightRed(){indicatorLight.setPosition(0.277);}
    public static void setLightOrange(){indicatorLight.setPosition(0.333);}
    public static void setLightYellow(){indicatorLight.setPosition(0.388);}
    public static void setLightGreen(){indicatorLight.setPosition(0.500);}
    public static void setLightAzure(){indicatorLight.setPosition(0.555);}
    public static void setLightBlue(){indicatorLight.setPosition(0.611);}
    public static void setLightIndigo(){indicatorLight.setPosition(0.666);}
    public static void setLightViolet(){indicatorLight.setPosition(0.722);}
    public static void setLightWhite(){indicatorLight.setPosition(1);}
    public static void turnLightOff(){indicatorLight.setPosition(0);}

}
