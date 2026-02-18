package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.subsystems.Breakbeam.intakeState;
import static org.firstinspires.ftc.teamcode.subsystems.Breakbeam.turretState;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Clock {
    private final Servo clock;
    private final CRServo ramp;
    private final DcMotorEx encoder;
    private final double CLOCK_INIT = 0.375;
    private final double CLOCK_SHOOT = .675; // 1:1 = .285 | 2:1 = .515
    private final double CLOCK_PRE_SHOOT = .475; // 1:1 = .13 | 2:1 = .2
    private final double RAMP_INIT = 0;
    private final double RAMP_SHOOT = 1;

    private final int ENCODER_SHOOT = 100;
    private boolean intakeBallDetected;
    private boolean lastIntakeDetected;
    private boolean turretBallDetected;
    private boolean lastTurretDetected;

    public int currentEncoderPosition = 0;
    private int initEncoderPostion = 0;
    private int shootEncoderPosition = 0;
    private int POSITION_TOLERANCE = 20;
    private int numBallsInClock = 0;
    public boolean isClockResetting;
    private long clockResetTimer;


    public Clock(HardwareMap hwMap) {
        clock = hwMap.get(Servo.class, "clock");
        ramp = hwMap.get(CRServo.class, "ramp");
        encoder = hwMap.get(DcMotorEx.class, "intake");
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        currentEncoderPosition = encoder.getCurrentPosition();

    }

    public void initClock()
    {
        clock.setPosition(CLOCK_INIT);
        ramp.setPower(RAMP_INIT);
    }

    public void clockUpdate() {
        currentEncoderPosition = encoder.getCurrentPosition();
        intakeNewBall();
        recordShotBall();



    }

    private void intakeNewBall() {
        if (!IntakeSubsystem.isIntakeRunning) {
            intakeBallDetected = false; // Reset when intake stops
            return;
        }

        if (!lastIntakeDetected && intakeState && !intakeBallDetected) {
            intakeBallDetected = true;
            numBallsInClock++;
        }

        // Reset on falling edge (ball passed sensor)
        if (lastIntakeDetected && !intakeState) {
            intakeBallDetected = false;
        }

        lastIntakeDetected = intakeState;
    }

    private void recordShotBall() {
        if (!Turret.isFlywheelRunning) {
            turretBallDetected = false; // Reset when flywheel stops
            return;
        }

        if (!lastTurretDetected && turretState && !turretBallDetected) {
            turretBallDetected = true;
            numBallsInClock--;
        }

        // Reset on falling edge (ball passed sensor)
        if (lastTurretDetected && !turretState) {
            turretBallDetected = false;
        }

        lastTurretDetected = turretState;
    }
    public void moveClockToShootPosition()
    {
        clock.setPosition(CLOCK_SHOOT);
    }
    public void moveClockToPreShootPosition() {clock.setPosition(CLOCK_PRE_SHOOT);}
    public void setRampToShootPower()
    {
        ramp.setPower(RAMP_SHOOT);
    }


    public void resetClock(){
        clock.setPosition(CLOCK_INIT);
        isClockResetting = true;
        clockResetTimer = SystemClock.uptimeMillis();

    }
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

    public void setClockPos(double pos)
    {
        clock.setPosition(pos);

    }
    public void setRampPower(double pwr)
    {
        ramp.setPower(pwr);
    }


    public void setNumBallsInClock(int balls)
    {
        numBallsInClock = balls;
    }




}
