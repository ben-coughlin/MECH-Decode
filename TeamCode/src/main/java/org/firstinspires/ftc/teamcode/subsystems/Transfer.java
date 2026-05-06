package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotMaster.inventory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Pattern;

public class Transfer {
    private final DcMotorEx transfer;
    private final Servo triggerServo;
    private final ElapsedTime timer =  new ElapsedTime();
    private final double triggerShoot = 0;
    private final double triggerBlock = 0.5;
    public static boolean isTransferRunning;
    private boolean isShotCommanded = false;



    public Transfer(HardwareMap hwMap)
    {
        transfer = hwMap.get(DcMotorEx.class, "transfer");
        triggerServo = hwMap.get(Servo.class, "triggerServo");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void updateTransfer()
    {
       handleIndicatorLightForBalls();

        if(ShooterSubsystem.isShotInProgress)
        {
            triggerServo.setPosition(triggerShoot);
            turnTransferOn();
            timer.reset();
        }
        else if(timer.seconds() > 0.5 || !ShooterSubsystem.isShotInProgress)
        {
            ShooterSubsystem.isShotInProgress = false;
        }
    }


    public void initTransfer()
    {
        triggerServo.setPosition(triggerBlock);
    }

    public void turnTransferOn()
    {
        transfer.setPower(1);
        isTransferRunning = true;
    }
    public void turnTransferOff()
    {
        transfer.setPower(0);
        isTransferRunning = false;
    }
    public void transferOuttake()
    {
        transfer.setPower(-1);
    }
    public void setTriggerToShoot()
    {
        triggerServo.setPosition(triggerShoot);
    }
    public void setTriggerToBlock()
    {
        triggerServo.setPosition(triggerBlock);
    }




    private void handleIndicatorLightForBalls()
    {
        if(inventory.getNumBalls() == 3)
        {
            IndicatorLight.setLightGreen();
        }
        else if(inventory.getNumBalls() == 2)
        {
            IndicatorLight.setLightYellow();
        }
        else if(inventory.getNumBalls() == 1)
        {
            IndicatorLight.setLightRed();
        }
        else
        {
            IndicatorLight.turnLightOff();
        }
    }

}
