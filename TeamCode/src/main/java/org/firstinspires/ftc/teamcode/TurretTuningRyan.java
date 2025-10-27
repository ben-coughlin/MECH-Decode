package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Tuning Ryan")
@Disabled
public class TurretTuningRyan extends OpMode
{
    private Turret turret;
    private IntakeSubsystem intakeSubsystem;

    Toggle kickerToggle = new Toggle(false);
    double turretPower = 0;
    double flyLeftPower = 0;
    double flyRightPower = 0;
    double kickerPos = 0;
    boolean hasBeenActivated = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {

        turret.updateTurret();
        kickerToggle.updateToggle(gamepad1.square);


        if (gamepad1.right_bumper && !hasBeenActivated) {
            turretPower += 0.05;
            hasBeenActivated = true;
        }
        else if (gamepad1.left_bumper && !hasBeenActivated) {
            turretPower -= 0.05;
            hasBeenActivated = true;
        }
        if (gamepad1.dpad_up && !hasBeenActivated) {
            flyLeftPower += 0.05;
            hasBeenActivated = true;
        }
        else if (gamepad1.dpad_down && !hasBeenActivated) {
            flyLeftPower -= 0.05;
            hasBeenActivated = true;
        }
        if(gamepad1.triangle && !hasBeenActivated)
        {
            flyRightPower += 0.05;
            hasBeenActivated = true;
        }
        else if(gamepad1.cross && !hasBeenActivated)
        {
            flyRightPower -= 0.05;
            hasBeenActivated = true;
        }
        if (gamepad1.dpad_right && !hasBeenActivated) {
            turret.setHoodPos(turret.getHoodPos() + 0.05);
            hasBeenActivated = true;
        }
        else if (gamepad1.dpad_left && !hasBeenActivated) {
            turret.setHoodPos(turret.getHoodPos() - 0.05);
            hasBeenActivated = true;
        }

        if (gamepad1.square && !hasBeenActivated)
        {
            kickerPos += 0.05;
            hasBeenActivated = true;
        }
        else if(gamepad1.circle && !hasBeenActivated)
        {
            kickerPos -= 0.05;
            hasBeenActivated = true;
        }
        intakeSubsystem.showSpindexerTelemetry(telemetry);

        turret.setTurretPower(turretPower);
        turret.setFlywheelLeftPower(flyLeftPower);
        turret.setFlywheelRightPower(flyRightPower);
        intakeSubsystem.setKickerPos(kickerPos);

        telemetry.addData("Current Turret Position", turret.getTurretPos());
        telemetry.addData("Current Flywheel Left Power", turret.getFlywheelLeftPower());
        telemetry.addData("Current Flywheel Right Power", turret.getFlywheelRightPower());
        telemetry.addData("Current Hood Position", turret.getHoodPos());
        telemetry.addData("Press right bumper on Gamepad 1 to increase Turret Power", "Press Left Bumper on Gamepad 1 to decrease Turret Power");
        telemetry.addData("Press Dpad Up on Gamepad 1 to increase Flywheel Left Power", "Press Dpad Down on Gamepad 1 to decrease Flywheel Left Power");
        telemetry.addData("Press Triangle on Gamepad 1 to increase Flywheel Right Power", "Press X on Gamepad 1 to decrease Flywheel Right Power");
        telemetry.addData("Press Dpad Right on Gamepad 1 to increase Hood Position", "Press Dpad Left on Gamepad 1 to decrease Hood Position");
        telemetry.update();

        hasBeenActivated = false;
    }
    @Override
    public void stop(){
    }
}
