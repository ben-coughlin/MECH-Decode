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
    boolean turretActivated = false;
    boolean flyRightActivated = false;
    boolean flyLeftActivated = false;
    boolean hoodActivated = false;
    boolean kickerActivated = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeSubsystem.setKickerPos(0.4);
    }

    public void start() {
        intakeSubsystem.setKickerPos(0.4);
    }

    @Override
    public void loop() {

        turret.updateTurret();
        kickerToggle.updateToggle(gamepad1.square);


        if (gamepad1.right_bumper && !turretActivated) {
            turretPower += 0.05;
            turretActivated = true;
        }
        else if (gamepad1.left_bumper && !turretActivated) {
            turretPower -= 0.05;
            turretActivated = true;
        }
        else if (!gamepad1.right_bumper && !gamepad1.right_bumper) {
            turretActivated = false;
        }
        if (gamepad1.dpad_up && !flyLeftActivated) {
            flyLeftPower += 0.05;
            flyLeftActivated = true;
        }
        else if (gamepad1.dpad_down && !flyLeftActivated) {
            flyLeftPower -= 0.05;
            flyLeftActivated = true;
        }
        else if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
            flyLeftActivated = false;
        }
        if(gamepad1.triangle && !flyRightActivated)
        {
            flyRightPower += 0.05;
            flyRightActivated = true;
        }
        else if(gamepad1.cross && !flyRightActivated)
        {
            flyRightPower -= 0.05;
            flyRightActivated = true;
        }
        else if(!gamepad1.triangle && !gamepad1.cross) {
            flyRightActivated = false;
        }
        if(gamepad1.dpad_right && !hoodActivated) {
            turret.setHoodPos(turret.getHoodPos() + 0.05);
            hoodActivated = true;
        }
        else if(gamepad1.dpad_left && !hoodActivated) {
            turret.setHoodPos(turret.getHoodPos() - 0.05);
            hoodActivated = true;
        }
        else if(!gamepad1.dpad_right && !gamepad1.dpad_left) {
            hoodActivated = false;
        }
        if(gamepad1.square && !kickerActivated)
        {
            kickerPos += 0.05;
            kickerActivated = true;
        }
        else if(gamepad1.circle && !kickerActivated)
        {
            kickerPos -= 0.05;
            kickerActivated = true;
        }
        else if(!gamepad1.square && !gamepad1.circle) {
            kickerActivated = false;
        }


        turret.setTurretPower(turretPower);
        turret.setFlywheelLeftPower(flyLeftPower);
        turret.setFlywheelRightPower(flyRightPower);
        intakeSubsystem.setKickerPos(kickerPos);

        telemetry.addData("Current Turret Position", turret.getTurretPos());
        telemetry.addData("Current Flywheel Left Power", turret.getFlywheelLeftPower());
        telemetry.addData("Current Flywheel Right Power", turret.getFlywheelRightPower());
        telemetry.addData("Current Hood Position", turret.getHoodPos());
        telemetry.addData("Current Kicker Position", kickerPos);
        telemetry.addData("Press right bumper on Gamepad 1 to increase Turret Power", "Press Left Bumper on Gamepad 1 to decrease Turret Power");
        telemetry.addData("Press Dpad Up on Gamepad 1 to increase Flywheel Left Power", "Press Dpad Down on Gamepad 1 to decrease Flywheel Left Power");
        telemetry.addData("Press Triangle on Gamepad 1 to increase Flywheel Right Power", "Press X on Gamepad 1 to decrease Flywheel Right Power");
        telemetry.addData("Press Dpad Right on Gamepad 1 to increase Hood Position", "Press Dpad Left on Gamepad 1 to decrease Hood Position");
        telemetry.update();
    }
}
