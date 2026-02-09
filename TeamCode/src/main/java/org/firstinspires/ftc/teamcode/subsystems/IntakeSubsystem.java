package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem
{
    private final DcMotorEx intake;
    public static boolean isIntakeRunning;


    public IntakeSubsystem(HardwareMap hwMap)
    {

        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        isIntakeRunning = true;

    }
    public void outtake()
    {
        intake.setPower(-1);
    }


    public void turnIntakeOff()
    {
        intake.setPower(0);
        isIntakeRunning = false;
    }





}
