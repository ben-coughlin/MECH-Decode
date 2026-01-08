package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Clock Tuning")
public class ClockTuning extends LinearOpMode {
    private double currentClockPosition = 0.5;
    private double currentRampPosition = 0.5;
    private double motorPower = 1;
    private final double SERVO_INCREMENT = 0.025;
    public Servo clock = null;
    public Servo ramp = null;

    private final ElapsedTime timer = new ElapsedTime();

    // State variables for shooting sequence
    private boolean shootingSequenceActive = false;

    // Debounce for servo controls
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    @Override
    public void runOpMode() {
        Limelight limelight = new Limelight(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        this.clock = hardwareMap.get(Servo.class, "clock");
        this.ramp = hardwareMap.get(Servo.class, "ramp");
        clock.setPosition(0.5);
        ramp.setPosition(0.5);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Place the robot in front of an AprilTag.");
        telemetry.addLine("Use DPAD UP/DOWN to adjust the launcher angle, press the circle button to toggle the flywheel");
        telemetry.addLine("Record the 'Distance' and 'Servo Position' for your lookup table.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            limelight.updateLimelight();
            LLResult result = Limelight.getCurrResult();
            double distanceToTag = limelight.getDistanceToTag(result);


            turret.aimTurret(true, result.getTx(), gamepad1.right_stick_y, distanceToTag);
            
            if (gamepad1.dpad_up && !dpadUpPressed)
            {
                currentClockPosition += SERVO_INCREMENT;
                dpadUpPressed = true;
            }
            else if (gamepad1.dpad_down && !dpadDownPressed)
            {
                currentClockPosition -= SERVO_INCREMENT;
                dpadDownPressed = true;
            }
            if (gamepad1.dpad_left && !dpadLeftPressed)
            {
                currentRampPosition += SERVO_INCREMENT;
                dpadLeftPressed = true;
            }
            else if (gamepad1.dpad_right && !dpadRightPressed)
            {
                currentRampPosition -= SERVO_INCREMENT;
                dpadRightPressed = true;
            }
            else
            {
                dpadUpPressed = false;
                dpadLeftPressed = false;
                dpadRightPressed = false;
                dpadDownPressed = false;
            }

            if (gamepad1.left_bumper) {
                intake.turnIntakeOn();
            } else if (gamepad1.right_bumper) {
                intake.outtake();
            } else {
                intake.turnIntakeOff();
            }
            // Constrain the servo position to the valid range [0.0, 1.0]
            currentClockPosition = Math.max(0.0, Math.min(1.0, currentClockPosition));
            currentRampPosition = Math.max(0.0, Math.min(1.0, currentRampPosition));
            clock.setPosition(currentClockPosition);
            ramp.setPosition(currentRampPosition);
            motorPower = Math.max(0.0, Math.min(1.0, motorPower));


            // --- Telemetry ---
            telemetry.addLine("--- Clock & Ramp Tuning ---");
            telemetry.addLine("Use DPAD UP/DOWN to change clock position.");
            telemetry.addLine("Use DPAD LEFT/RIGHT to change ramp position");
            telemetry.addData("Distance to Tag (in)", "%.2f", distanceToTag);
            telemetry.addData("Clock Servo Position", "%.3f", currentClockPosition);
            telemetry.addData("Ramp Servo Position", "%.3f", currentRampPosition);
            telemetry.addData("Motor Power", "%.3f", turret.flywheelLeft.getPower());
            telemetry.addData("Desired Power", "%.3f", motorPower);
            
            telemetry.addLine("--- Intake ---");
            telemetry.addData("Intake Active", IntakeSubsystem.isIntakeRunning);

            telemetry.update();
        }
    }
}
