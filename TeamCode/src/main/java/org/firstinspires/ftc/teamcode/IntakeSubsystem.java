package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem
{

    private final Servo kicker;
    private final DcMotorEx intake;


    public IntakeSubsystem(HardwareMap hwMap)
    {
        kicker = hwMap.get(Servo.class, "kicker");
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kicker.setPosition(0.37);

    }



    public double normalizeAngleFromTicks(double position, int ticksPerRevolution)
    {
        double revolutions = position / ticksPerRevolution;

        double angle = revolutions * 360;

        return angle / 360;
    }




    public void turnIntakeOn()
    {
        double intakePower = 1;
        intake.setPower(intakePower);
    }
    public void outtake()
    {
        intake.setPower(-1);
    }


    public void turnIntakeOff()
    {
        intake.setPower(0);
    }

    public void moveKickerVertical()
    {
        double kickerVertical = 0.05;
      kicker.setPosition(kickerVertical);
    }

    public void moveKickerHorizontal()
    {
        double kickerHorizontal = 0.4;
        kicker.setPosition(kickerHorizontal);
    }
    public double getKickerPos()
    {
        return kicker.getPosition();
    }
    public void setKickerPos(double kickerPos)
    {
        kicker.setPosition(kickerPos);
    }




}
