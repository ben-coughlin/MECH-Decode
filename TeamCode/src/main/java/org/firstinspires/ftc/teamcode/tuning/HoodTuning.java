package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Hood Tuning")
public class HoodTuning extends LinearOpMode {
    private double currentServoPosition = 0.5; // Start at middle position
    private double rpm = 0.0; // Start with motors off

    private final double SERVO_INCREMENT = 0.005;
    private final double MOTOR_INCREMENT = 50;

    private final ElapsedTime timer = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;


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
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");



        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("DPAD UP/DOWN: Adjust hood angle");
        telemetry.addLine("DPAD LEFT/RIGHT: Adjust motor power");
        telemetry.addLine("CIRCLE: Shoot");
        telemetry.addLine("CROSS: Cancel shot");
        telemetry.addLine("RIGHT TRIGGER: Intake & transfer on");
        telemetry.addLine("LEFT TRIGGER: Intake & transfer off");
        telemetry.addLine("================");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            transfer.updateTransfer();
            colorSensor.updateDetection();


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // Update limelight data
            limelight.updateLimelight();
            LLResult result = Limelight.getCurrResult();
            double distanceToTag = limelight.getDistanceToTag(result);


            boolean hasValidVision = result != null
                    && result.isValid();

            // ALWAYS call aimTurret - it will use odometry if vision is lost
//            turret.aimTurret(
//                    hasValidVision,
//                    hasValidVision ? Limelight.getCurrResult().getTx() : 0,
//                    gamepad2.right_stick_x,
//                   999, //so we dont get annoying oscillation
//                    0,
//                    0,
//                    0
//            );

            //since turret gains are fried we're just not gonna move it for now :P

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
                rpm += MOTOR_INCREMENT;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                rpm -= MOTOR_INCREMENT;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

           if(gamepad1.right_trigger > 0.05)
           {
               intake.turnIntakeOn();
               transfer.turnTransferOn();
           }
           else if(gamepad1.left_trigger > 0.05){
               intake.turnIntakeOff();
               transfer.turnTransferOff();
           }


            // === FLYWHEEL TOGGLE CONTROLS ===
            if (gamepad1.circle && !circlePressed) {
                flywheelActive = true;
                turret.forceOnFlywheel(rpm);
                transfer.transferShoot();
                intake.turnIntakeOn();
                circlePressed = true;
            } else if (!gamepad1.circle) {
                circlePressed = false;
            }

            if (gamepad1.cross && !crossPressed) {
                flywheelActive = false;
                transfer.transferEndShoot();
                intake.turnIntakeOff();
                crossPressed = true;
            } else if (!gamepad1.cross) {
                crossPressed = false;
            }

            if (gamepad1.triangle && !trianglePressed) {
                flywheelActive = false;
                turret.forceOffFlywheel();
                trianglePressed = true;
            } else if (!gamepad1.triangle) {
                trianglePressed = false;
            }

            if(flywheelActive)
            {
                turret.forceOnFlywheel(rpm);
            }



            // Constrain values to valid ranges
            currentServoPosition = Math.max(0.0, Math.min(1.0, currentServoPosition));
            rpm = Math.max(0.0, Math.min(6000, rpm));

            // Set the servo position
            launchServo.setPosition(currentServoPosition);
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("DPAD UP/DOWN: Adjust hood angle");
            telemetry.addLine("DPAD LEFT/RIGHT: Adjust motor power");
            telemetry.addLine("CIRCLE: Shoot");
            telemetry.addLine("CROSS: End shot");
            telemetry.addLine("TRIANGLE: Turn off flywheel");
            telemetry.addLine("================");

            // === TELEMETRY ===
            telemetry.addLine("=== LAUNCHER TUNING ===");
            telemetry.addData("Distance to Tag (in)", "%.2f", distanceToTag);
            telemetry.addLine();
            telemetry.addData("Hood Servo Position", "%.3f", currentServoPosition);
            telemetry.addData("Flywheel Status", flywheelActive ? "ON" : "OFF");
            telemetry.addData("Flywheel Commanded RPM", "%.3f", rpm);
            telemetry.addData("Flywheel Real RPM", "%.3f", turret.getRealRPM());

            telemetry.addLine();
            telemetry.addLine("--- Record These Values ---");
            telemetry.addData("Distance", "%.2f in", distanceToTag);
            telemetry.addData("Hood Position", "%.3f", currentServoPosition);
            telemetry.addData("Power", "%.3f", rpm);
            telemetry.update();
        }
    }
}