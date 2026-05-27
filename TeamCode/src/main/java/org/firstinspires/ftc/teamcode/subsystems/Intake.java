package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Pattern;

public class Intake
{
    private final DcMotorEx intake;
    public static boolean isIntakeRunning;
    public static boolean isOuttakeRunning;



    public Intake(HardwareMap hwMap)
    {

        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void turnIntakeOn()
    {
        double intakePower = 1;
        intake.setPower(intakePower);
        isIntakeRunning = true;
        isOuttakeRunning = false;

    }
    public void turnOuttakeOn()
    {
        isIntakeRunning = false;
        isOuttakeRunning = true;
        intake.setPower(-1);
    }


    public void turnIntakeOff()
    {
        intake.setPower(0);
        isIntakeRunning = false;
        isOuttakeRunning = false;
    }









}
