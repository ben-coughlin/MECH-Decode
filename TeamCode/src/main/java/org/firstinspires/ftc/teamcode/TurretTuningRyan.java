package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret TuningRyan")
@Disabled
public class TurretTuningRyan extends RobotMasterPinpoint
{
    Toggle kickerToggle = new Toggle(false);
    double turretPower = 0;
    double flyLeftPower = 0;
    double flyRightPower = 0;
    double kickerPos = 0;

    @Override
    public void init() {
        super.init();
    }
    @Override
    public void init_loop() {
        super.init_loop();
    }
    @Override
    public void start() {
        super.start();
    }
    @Override
    public void mainLoop() {

        turret.updateTurret();
        kickerToggle.updateToggle(gamepad1.square);


        if (gamepad1.right_bumper) {
            turretPower += 0.05;
        }
        else if (gamepad1.left_bumper) {
            turretPower -= 0;
        }
        if (gamepad1.dpad_up) {
            flyLeftPower += 0.05;
        }
        else if (gamepad1.dpad_down) {
            flyLeftPower -= 0.05;
        }
        if(gamepad1.triangle)
        {
            flyRightPower += 0.05;
        }
        else if(gamepad1.cross)
        {
            flyRightPower -= 0.05;
        }
        if (gamepad1.dpad_right) {
            turret.setHoodPos(turret.getHoodPos() + 0.05);
        }
        else if (gamepad1.dpad_left) {
            turret.setHoodPos(turret.getHoodPos() - 0.05);
        }

        if (gamepad1.square)
        {
            kickerPos += 0.05;
        }
        else if(gamepad1.circle)
        {
            kickerPos += 0.05;
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
    }
    @Override
    public void stop(){
        super.stop();
    }
}
