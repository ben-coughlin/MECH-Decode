package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Clock {
    private Servo clockSpinner;
    private Servo ramp;
    private DcMotorEx encoder;
    private final double CLOCK_INIT = 0;
    private final double CLOCK_SHOOT = 1;
    private final double RAMP_INIT = 0 ;
    private final double RAMP_SHOOT = 1;
    private final int ENCODER_INIT = 0;
    private final int ENCODER_SHOOT = 100;
    private double currentClockPosition = 0;
    private double currentRampPosition = 0;
    private int currentEncoderPosition = 0;
    private int POSITION_TOLERANCE = 20;

    public Clock(HardwareMap hwMap) {
        clockSpinner = hwMap.get(Servo.class, "clock");
        ramp = hwMap.get(Servo.class, "ramp");
        encoder = hwMap.get(DcMotorEx.class, "intake");
        clockSpinner.setPosition(CLOCK_INIT);
        ramp.setPosition(RAMP_INIT);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        currentClockPosition = clockSpinner.getPosition();
        currentRampPosition = ramp.getPosition();
        currentEncoderPosition = encoder.getCurrentPosition();

    }
    public void clockUpdate(){
        currentClockPosition = clockSpinner.getPosition();
        currentRampPosition = ramp.getPosition();
        currentEncoderPosition = encoder.getCurrentPosition();
    }
    public void clockShoot()
    {
        clockSpinner.setPosition(CLOCK_SHOOT);
    }
    public void rampShoot()
    {
        ramp.setPosition(RAMP_SHOOT);
    }
    public void clockInit(){clockSpinner.setPosition(CLOCK_INIT);}
    public void rampInit(){ramp.setPosition(RAMP_INIT);}


    public boolean isAtShootPosition()
    {
        int error = Math.abs(ENCODER_SHOOT - currentEncoderPosition);
        if (error <= POSITION_TOLERANCE)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
