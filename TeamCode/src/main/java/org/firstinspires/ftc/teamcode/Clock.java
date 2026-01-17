package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Clock {
    private Servo clock;
    private CRServo ramp;
    private DcMotorEx encoder;
    private final double CLOCK_INIT = 0.5;
    private final double CLOCK_PRE_SHOOT = .59;
    private final double CLOCK_SHOOT = .75;
    private final double RAMP_INIT = 0;
    private final double RAMP_SHOOT = 1;
    private final int ENCODER_INIT = 0;
    private final int ENCODER_SHOOT = 100;
    private double currentClockPosition = 0;
    private double currentRampPosition = 0;
    private int currentEncoderPosition = 0;
    private int POSITION_TOLERANCE = 20;

    public Clock(HardwareMap hwMap) {
        clock = hwMap.get(Servo.class, "clock");
        ramp = hwMap.get(CRServo.class, "ramp");
        encoder = hwMap.get(DcMotorEx.class, "intake");
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
        clock.setPosition(CLOCK_INIT);
        ramp.setPower(RAMP_INIT);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        currentClockPosition = clock.getPosition();
        currentRampPosition = ramp.getPower();
        currentEncoderPosition = encoder.getCurrentPosition();


    }
    public void clockUpdate(){
        currentClockPosition = clock.getPosition();
        currentRampPosition = ramp.getPower();
        currentEncoderPosition = encoder.getCurrentPosition();
    }
    public void moveClockToShootPosition()
    {
        clock.setPosition(CLOCK_SHOOT);
    }
    public void setClockToPreShootPosition()
    {
        clock.setPosition(CLOCK_PRE_SHOOT);
    }

    public void setRampToShootPower()
    {
        ramp.setPower(RAMP_SHOOT);
    }


    public void resetClock(){
        clock.setPosition(CLOCK_INIT);}
    public void stopRamp(){ramp.setPower(RAMP_INIT);}


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

    void setClockPos(double pos)
    {
        clock.setPosition(pos);

    }
    void setRampPower(double pwr)
    {
        ramp.setPower(pwr);
    }


}
