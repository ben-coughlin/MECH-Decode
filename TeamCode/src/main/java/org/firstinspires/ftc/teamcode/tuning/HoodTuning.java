package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Clock;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Disabled
@TeleOp(name = "Hood Tuning")
public class HoodTuning extends LinearOpMode {
    private double currentServoPosition = 0.5; // Start at middle position
    private double motorPower = 0.0; // Start with motors off

    private final double SERVO_INCREMENT = 0.005;
    private final double MOTOR_INCREMENT = 50;

    private final ElapsedTime timer = new ElapsedTime();

    // State variables
    private boolean flywheelActive = false;

    // Debounce variables
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean circlePressed = false;
    private boolean crossPressed = false;
    private boolean trianglePressed = false;
    private boolean squarePressed = false;
    private boolean rightTriggerPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);
        Servo launchServo = hardwareMap.get(Servo.class, "hood");
        Turret turret = new Turret(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        Clock clock = new Clock(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("DPAD UP/DOWN: Adjust hood angle");
        telemetry.addLine("DPAD LEFT/RIGHT: Adjust motor power");
        telemetry.addLine("CIRCLE: Toggle flywheel ON");
        telemetry.addLine("CROSS: Turn flywheel OFF");
        telemetry.addLine("TRIANGLE: Shoot (move clock to shoot position)");
        telemetry.addLine("SQUARE: Reset (move clock to pre-shoot position)");
        telemetry.addLine("================");
        telemetry.update();
        clock.initClock();
        waitForStart();

        while (opModeIsActive()) {
            // Update limelight data
            limelight.updateLimelight();
            LLResult result = Limelight.getCurrResult();
            double distanceToTag = limelight.getDistanceToTag(result);


            boolean hasValidVision = result != null
                    && result.isValid();

            // ALWAYS call aimTurret - it will use odometry if vision is lost
            turret.aimTurret(
                    hasValidVision,
                    hasValidVision ? Limelight.getCurrResult().getTx() : 0,
                    gamepad2.right_stick_x,
                   999, //so we dont get annoying oscillation
                    0,
                    0,
                    0
            );

            // === HOOD SERVO CONTROLS ===
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

            // === MOTOR POWER CONTROLS ===
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

           if(gamepad1.right_trigger > 0.05)
           {
               intake.turnIntakeOn();
           }
           else {
               intake.turnIntakeOff();
           }

            // === FLYWHEEL TOGGLE CONTROLS ===
            if (gamepad1.circle && !circlePressed) {
                flywheelActive = true;
                turret.setFlywheelRPM(motorPower);
                clock.setRampToShootPower();
                circlePressed = true;
            } else if (!gamepad1.circle) {
                circlePressed = false;
            }

            if (gamepad1.cross && !crossPressed) {
                flywheelActive = false;
                turret.setFlywheelRPM(0);
                clock.stopRamp();
                clock.initClock();
                crossPressed = true;
            } else if (!gamepad1.cross) {
                crossPressed = false;
            }

            // === CLOCK (BALL HANDLER) CONTROLS ===
            if (gamepad1.triangle && !trianglePressed) {
                clock.moveClockToShootPosition();
                trianglePressed = true;
            } else if (!gamepad1.triangle) {
                trianglePressed = false;
            }

            if (gamepad1.square && !squarePressed) {
                clock.moveClockToPreShootPosition();
                squarePressed = true;
            } else if (!gamepad1.square) {
                squarePressed = false;
            }

            // Update flywheel power if active (allows live adjustment)
            if (flywheelActive) {
               turret.setFlywheelRPM(motorPower);
            }

            // Constrain values to valid ranges
            currentServoPosition = Math.max(0.0, Math.min(1.0, currentServoPosition));
            motorPower = Math.max(0.0, Math.min(6000, motorPower));

            // Set the servo position
            launchServo.setPosition(currentServoPosition);
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("DPAD UP/DOWN: Adjust hood angle");
            telemetry.addLine("DPAD LEFT/RIGHT: Adjust motor power");
            telemetry.addLine("CIRCLE: Toggle flywheel ON");
            telemetry.addLine("CROSS: Turn flywheel OFF");
            telemetry.addLine("TRIANGLE: Shoot (move clock to shoot position)");
            telemetry.addLine("SQUARE: Reset (move clock to pre-shoot position)");
            telemetry.addLine("================");

            // === TELEMETRY ===
            telemetry.addLine("=== LAUNCHER TUNING ===");
            telemetry.addData("Distance to Tag (in)", "%.2f", distanceToTag);
            telemetry.addLine();
            telemetry.addData("Hood Servo Position", "%.3f", currentServoPosition);
            telemetry.addData("Flywheel Status", flywheelActive ? "ON" : "OFF");
            telemetry.addData("Flywheel RPM", "%.3f", motorPower);
            telemetry.addData("Actual Motor RPM", "%.3f", turret.getCurrentFlywheelRPM());
            telemetry.addLine();
            telemetry.addLine("--- Record These Values ---");
            telemetry.addData("Distance", "%.2f in", distanceToTag);
            telemetry.addData("Hood Position", "%.3f", currentServoPosition);
            telemetry.addData("Power", "%.3f", motorPower);
            telemetry.update();
        }
    }
}