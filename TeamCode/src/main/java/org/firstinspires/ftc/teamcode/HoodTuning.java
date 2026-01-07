package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Hood Tuning")
public class HoodTuning extends LinearOpMode {
    private double currentServoPosition = 0;
    private double motorPower = 1;
    private final double MOTOR_INCREMENT = 0.025;

    private final ElapsedTime timer = new ElapsedTime();

    // State variables for shooting sequence
    private boolean shootingSequenceActive = false;

    // Debounce for servo controls
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);
        Servo launchServo = hardwareMap.get(Servo.class, "hood");
        Turret turret = new Turret(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);







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

            if (gamepad1.circle && !shootingSequenceActive)
            {
                shootingSequenceActive = true;

                turret.flywheelRight.setPower(motorPower);
                turret.flywheelLeft.setPower(motorPower);
            }
            if (gamepad1.cross) {
                turret.flywheelRight.setPower(0);
                turret.flywheelLeft.setPower(0);
                shootingSequenceActive = false;
            }



            double SERVO_INCREMENT = 0.005;
            if (gamepad1.dpad_up && !dpadUpPressed) {
                currentServoPosition += SERVO_INCREMENT;

                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownPressed) {
               currentServoPosition -= SERVO_INCREMENT;

                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                motorPower += MOTOR_INCREMENT;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }
            if (gamepad1.dpad_right && !dpadRightPressed) {
                motorPower -= MOTOR_INCREMENT;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }


            // Constrain the servo position to the valid range [0.0, 1.0]
            currentServoPosition = Math.max(0.0, Math.min(1.0, currentServoPosition));
            motorPower = Math.max(0.0, Math.min(1.0, motorPower));

            // Set the servo's position
            launchServo.setPosition(currentServoPosition);


            // --- Telemetry ---
            telemetry.addLine("--- Launcher Tuning ---");
            telemetry.addLine("Use DPAD UP/DOWN to change servo position.");
            telemetry.addData("Distance to Tag (in)", "%.2f", distanceToTag);
            telemetry.addData("Servo Position", "%.3f", currentServoPosition);
            telemetry.addData("Motor Power", "%.3f", turret.flywheelLeft.getPower());
            telemetry.addData("Desired Power", "%.3f", motorPower);
            telemetry.update();
        }
    }
}
