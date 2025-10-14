package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    private CRServo spindexer = null;
    private Servo kicker = null;
    private DcMotorEx intake = null;
    private final int spindexCountsPerRev = 8192;
    private final int spindexCountsPerSlot = 2730; //TODO: tune this number
    private int spindexerPosition = 0;



    public IntakeSubsystem(HardwareMap hwMap)
    {
        spindexer = hwMap.get(CRServo.class, "spindexer");
        kicker = hwMap.get(Servo.class, "kicker");
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kickerInit = 0.52;
        kicker.setPosition(kickerInit);
    }

    public void updateIntakeSubsystem()
    {
        spindexerPosition = intake.getCurrentPosition();
    }
    public void showSpindexerTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Intake Encoder Ticks ", spindexerPosition);
        telemetry.addData("Intake Encoder Angle ", normalizeAngleFromTicks(spindexerPosition, spindexCountsPerRev));
    }


    public double normalizeAngleFromTicks(double position, int ticksPerRevolution)
    {
        double revolutions = position / ticksPerRevolution;

        double angle = revolutions * 360;

        return angle / 360;
    }
    public void rotateSpindexerOneSlot()
    {
        double desiredSpindexerTicks = spindexerPosition + spindexCountsPerSlot;

        while(spindexerPosition < desiredSpindexerTicks)
        {
            spindexer.setPower(1);

        }
        spindexer.setPower(0);

    }


    public void turnIntakeOn()
    {
        double intakePower = 1;
        intake.setPower(intakePower);
    }
    public void turnIntakeOff()
    {
        intake.setPower(0);
    }
    public void turnKickerOn()
    {
        double kickerMax = .48;
        kicker.setPosition(kickerMax);
    }
    public void turnKickerOff()
    {
        double kickerMin = .52;
        kicker.setPosition(kickerMin);
    }







}
