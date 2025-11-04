package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Hood Tuning")
public class HoodTuning extends LinearOpMode {
    private Servo launchServo;
    private Limelight limelight;
    private double currentServoPosition = 0.5;

    private static final double SERVO_INCREMENT = 0.005;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new Limelight(hardwareMap);
        launchServo = hardwareMap.get(Servo.class, "hood");

        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Place the robot in front of an AprilTag.");
        telemetry.addLine("Use DPAD UP/DOWN to adjust the launcher angle.");
        telemetry.addLine("Record the 'Distance' and 'Servo Position' for your lookup table.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Limelight Distance Calculation ---
            limelight.updateLimelight();
            LLResult result = limelight.getCurrResult();
            double distanceToTag = limelight.getDistanceToTag(result);

            // --- Servo Tuning Logic ---
            // Use a simple edge detector to prevent the value from changing too fast.
            if (gamepad1.dpad_up && !dpadUpPressed) {
                currentServoPosition += SERVO_INCREMENT;
            } else if (gamepad1.dpad_down && !dpadDownPressed) {
                currentServoPosition -= SERVO_INCREMENT;
            }
            dpadUpPressed = gamepad1.dpad_up;
            dpadDownPressed = gamepad1.dpad_down;

            // Constrain the servo position to the valid range [0.0, 1.0]
            currentServoPosition = Math.max(0.0, Math.min(1.0, currentServoPosition));

            // Set the servo's position
            launchServo.setPosition(currentServoPosition);


            // --- Telemetry ---
            telemetry.addLine("--- Launcher Tuning ---");
            telemetry.addLine("Use DPAD UP/DOWN to change servo position.");
            telemetry.addData("Distance to Tag (in)", "%.2f", distanceToTag);
            telemetry.addData("Servo Position", "%.3f", currentServoPosition);
            telemetry.update();
        }
    }
}
