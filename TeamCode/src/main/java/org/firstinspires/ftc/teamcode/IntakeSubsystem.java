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
    private final CRServo spindexer;
    private final Servo kicker;
    private final DcMotorEx intake;
    private final int spindexCountsPerRev = 8192; //REV-11-127 through bore encoder
    private final int spindexCountsPerSlot = 2500; //TODO: tune this number
    private int spindexerPosition = 0;

    // State machine for spindexer rotation
    private boolean isRotating = false;
    private double desiredSpindexerTicks = 0;
    private static final double SPINDEXER_POWER = 1.0;


    public IntakeSubsystem(HardwareMap hwMap)
    {
        spindexer = hwMap.get(CRServo.class, "spindexer");
        kicker = hwMap.get(Servo.class, "kicker");
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kicker.setPosition(0);


    }

    public void updateIntakeSubsystem()
    {
        spindexerPosition = intake.getCurrentPosition();


        if (isRotating)
        {
            if (spindexerPosition < desiredSpindexerTicks)
            {
                spindexer.setPower(SPINDEXER_POWER);
            }
            else
            {
                stopSpindexer();
            }
        }
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
        // Only start a new rotation if one isn't already in progress.
        if (!isRotating)
        {
            desiredSpindexerTicks = spindexerPosition + spindexCountsPerSlot;
            isRotating = true;
            // The actual power setting is now handled by the updateIntakeSubsystem() method.
        }
    }

    /**
     * Stops the spindexer and resets the rotation state.
     * This can be called to manually stop a rotation or when the target is reached.
     */
    public void stopSpindexer()
    {
        spindexer.setPower(0);
        isRotating = false;
        desiredSpindexerTicks = 0;
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

    public void moveKickerVertical()
    {
        double kickerVertical = .5;
      kicker.setPosition(kickerVertical);
    }

    public void moveKickerHorizontal()
    {
        double kickerHorizontal = 0;
        kicker.setPosition(kickerHorizontal);
    }


}
