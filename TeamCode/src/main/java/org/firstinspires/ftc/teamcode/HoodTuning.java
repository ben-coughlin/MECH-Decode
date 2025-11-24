package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Hood Tuning")
public class HoodTuning extends LinearOpMode {
    private double currentServoPosition = 0.5;
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


            final double MAX_POWER_DISTANCE = 40.0; // inches

            final double MIN_POWER_DISTANCE = 20.0; // inches

            final double MIN_POWER = 0.80;

            double compensatedPower;

            if (distanceToTag >= MAX_POWER_DISTANCE) {
                compensatedPower = 1.0;
            } else if (distanceToTag <= MIN_POWER_DISTANCE) {
                compensatedPower = MIN_POWER;
            } else {

                double range = MAX_POWER_DISTANCE - MIN_POWER_DISTANCE;
                double howFarIntoRange = (distanceToTag - MIN_POWER_DISTANCE) / range;

                double powerRange = 1.0 - MIN_POWER;
                compensatedPower = MIN_POWER + (howFarIntoRange * powerRange);
            }


            compensatedPower = Math.max(MIN_POWER, Math.min(1.0, compensatedPower));




            if (gamepad1.circle && !shootingSequenceActive)
            {
                shootingSequenceActive = true;
                timer.reset();
              //  turret.setFlywheelPower(motorPower);
            }

            if (shootingSequenceActive) {
                if (timer.seconds() > 5) {
                    intake.moveKickerVertical();
                }
                if (timer.seconds() > 6) {
                   // turret.setFlywheelPower(0);
                    intake.moveKickerHorizontal();
                    shootingSequenceActive = false;
                }
            }

            if(gamepad1.square)
            {
                currentServoPosition = 0;
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
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addData("Compensated Power", "%.3f", compensatedPower);
            telemetry.update();
        }
    }
}
