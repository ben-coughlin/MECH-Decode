package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Clock;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@Disabled
@TeleOp(name = "Clock Tuning")
public class ClockTuning extends LinearOpMode {
    private double currentClockPosition = 0.5;
    private double motorPower = 1;
    private final double SERVO_INCREMENT = 0.025;


    // Previous button states for rising edge detection
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    boolean lastCircle = false; // Added for flywheel toggle if needed
    boolean rampOn = false;

    @Override
    public void runOpMode() {

        Clock clock = new Clock( hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        Turret turret = new Turret(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            clock.clockUpdate();
            turret.updateTurret();

//            limelight.updateLimelight();
//            LLResult result = Limelight.getCurrResult();
//            double distanceToTag = Limelight.getDistance();
//
//            // --- TURRET AIMING ---
//            // Note: Make sure aimTurret handles null results gracefully if needed
//            double tx = (result != null) ? result.getTx() : 0;
//            turret.aimTurret(true, tx, gamepad1.right_stick_y, distanceToTag);
//            turret.updateTurret(); // Ensure the turret loop runs!

            // --- SERVO INCREMENT LOGIC (Rising Edge Detection) ---

            // CLOCK: Increment on DPAD UP press
            if (gamepad1.dpad_up && !lastDpadUp) {
                currentClockPosition += SERVO_INCREMENT;
            }
            lastDpadUp = gamepad1.dpad_up;

            // CLOCK: Decrement on DPAD DOWN press
            if (gamepad1.dpad_down && !lastDpadDown) {
                currentClockPosition -= SERVO_INCREMENT;
            }
            lastDpadDown = gamepad1.dpad_down;

            // RAMP: Decrement on DPAD RIGHT press
            if (gamepad1.dpad_right && !lastDpadRight) {
                 clock.setRampPower(-0.75);
                 rampOn = true;
            }
            else if (gamepad1.dpad_left && ! lastDpadRight) {
                clock.setRampPower(0);
                rampOn = false;
            }
            lastDpadRight = gamepad1.dpad_right;


            if (gamepad1.left_bumper) {
                intake.turnIntakeOn();
            } else if (gamepad1.right_bumper) {
                intake.outtake();
            } else {
                intake.turnIntakeOff();
            }

            if(gamepad1.triangle)
            {
                clock.moveClockToShootPosition();
            }
            if(gamepad1.square)
            {
                clock.stopRamp();
                clock.resetClock();
            }

            // --- BOUNDARY CHECKS & UPDATES ---
            currentClockPosition = Math.max(0.0, Math.min(1.0, currentClockPosition));

            if(gamepad1.cross)
            {
                clock.setClockPos(currentClockPosition);
            }

            if(gamepad2.dpad_up)
            {
                turret.turnOnFlywheel();
            }
            else if(gamepad2.dpad_down)
            {
                turret.turnOffFlywheel();
            }

            // --- TELEMETRY ---
            telemetry.addLine("--- Clock & Ramp Tuning ---");
            //      telemetry.addData("Distance to Tag (in)", "%.2f", distanceToTag);
            telemetry.addData("Clock Position", currentClockPosition);
            telemetry.addData("clock encoder pos", clock.currentEncoderPosition);
            telemetry.addData("Ramp On?", rampOn);
            //   telemetry.addData("Flywheel Power", "%.2f", turret.flywheelRight.getPower());

            // This is useful to see if your Limelight is actually getting data
            //  telemetry.addData("Limelight Valid", result != null);

            telemetry.update();
        }
    }
}
