package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tur8ret Tuning")
//@Disabled
public class TurretTuning extends LinearOpMode {

    // You only need to declare the Turret object here.
    private Turret turret;
    private IntakeSubsystem intakeSubsystem;

    // A timer can be useful for seeing loop times.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code goes here, before waitForStart().
        // Create an instance of your Turret subsystem.
        turret = new Turret(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // Constants for slow, controlled movement.
        final double TURRET_MANUAL_SPEED = 1;  // Reduced speed for fine control
        final double HOOD_INCREMENT_SPEED = 0.05; // Small increment for servo tuning

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press Start to begin tuning.");
        telemetry.addLine("> Left Stick X: Turret Manual");
        telemetry.addLine("> Right Stick Y: Flywheel Manual");
        telemetry.addLine("> DPAD Up/Down: Kicker Manual");
        telemetry.update();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double manualTurretPower = gamepad1.left_stick_x * TURRET_MANUAL_SPEED;
            double manualFlywheelPower = -gamepad1.right_stick_y; // Joysticks are inverted; negative is up

            if (gamepad1.dpad_up) {
                intakeSubsystem.setKickerPos(intakeSubsystem.getKickerPos() + HOOD_INCREMENT_SPEED);
            } else if (gamepad1.dpad_down) {
                intakeSubsystem.setKickerPos(intakeSubsystem.getKickerPos() - HOOD_INCREMENT_SPEED);
            }



            turret.updateTurret();


            turret.aimTurret(false, 0, manualTurretPower);
            turret.setFlywheelLeftPower(manualFlywheelPower);
            turret.setFlywheelRightPower(manualFlywheelPower);


            telemetry.addData("Status", "Running");
            telemetry.addData("Loop Time (ms)", runtime.milliseconds());
            runtime.reset(); // Reset timer for next loop cycle

            telemetry.addLine("\n--- CONTROLS ---");
            telemetry.addData("Turret Manual Power", "%.2f", manualTurretPower);
            telemetry.addData("Flywheel Manual Power", "%.2f", manualFlywheelPower);

            telemetry.addLine("\n--- READOUTS ---");
            telemetry.addData("Turret Encoder", turret.getTurretPos());
            telemetry.addData("Flywheel Left", turret.getFlywheelLeftPower());
            telemetry.addData("Flywheel Right", turret.getFlywheelRightPower());
            telemetry.addData("kicker Servo Position", "%.3f", intakeSubsystem.getKickerPos());

            telemetry.update();
        }
    }
}
