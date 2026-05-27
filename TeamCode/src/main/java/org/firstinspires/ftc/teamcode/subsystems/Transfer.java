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
    private final double triggerShoot = 0.415;
    private final double triggerBlock = 0.275;
    private boolean isTransferRunning;
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

       if(isTransferRunning && (inventory.getUpper().equals(Pattern.Ball.GREEN) || inventory.getUpper().equals(Pattern.Ball.PURPLE)) && !isShotCommanded)
        {
            transfer.setPower(0);
        }
        else if(isTransferRunning)
        {
            transfer.setPower(1);
        }
        else
       {
           transfer.setPower(0);
       }
    }


    public void initTransfer()
    {
        triggerServo.setPosition(triggerBlock);
    }

    public void turnTransferOn()
    {
        isTransferRunning = true;
    }
    public void turnTransferOff()
    {
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
    public void transferShoot()
    {
        isShotCommanded = true;
        triggerServo.setPosition(triggerShoot);
        turnTransferOn();
        timer.reset();
    }
    public void transferEndShoot()
    {
        isShotCommanded = false;
        triggerServo.setPosition(triggerBlock);
        turnTransferOff();
    }


}
