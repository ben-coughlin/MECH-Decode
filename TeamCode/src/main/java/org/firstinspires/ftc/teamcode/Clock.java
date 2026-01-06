package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Clock {
    private Servo clockSpinner;
    private Servo ramp;
    private final double CLOCK_INIT = 0;
    private final double CLOCK_SHOOT = 1;
    private final double RAMP_INIT = 0 ;
    private final double RAMP_SHOOT = 1;

    public Clock(HardwareMap hwMap) {
        clockSpinner = hwMap.get(Servo.class, "clock");
        ramp = hwMap.get(Servo.class, "ramp");
        clockSpinner.setPosition(CLOCK_INIT);
        ramp.setPosition(RAMP_INIT);

    }

    private void spinClock()
    {
        clockSpinner.setPosition(CLOCK_SHOOT);
    }
    private void moveRampForShoot()
    {
        ramp.setPosition(RAMP_SHOOT);
    }





}
