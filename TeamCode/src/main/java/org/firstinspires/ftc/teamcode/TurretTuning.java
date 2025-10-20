package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Tuning")
public class TurretTuning extends RobotMasterPinpoint
{
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
        super.mainLoop();
        if (gamepad1.right_bumper) {
            turret.setTurretPower(turret.getTurretPower() + 1);
        }
        else if (gamepad1.left_bumper) {
            turret.setTurretPower(turret.getTurretPower() - 1);
        }
        if (gamepad1.dpad_up) {
            turret.setFlywheelPower(turret.getFlywheelRPM() + 0.01);
        }
        else if (gamepad1.dpad_down) {
            turret.setFlywheelPower(turret.getFlywheelRPM() - 0.01);
        }
        if (gamepad1.dpad_right) {
            turret.setHoodPos(turret.getHoodPos() + 0.05);
        }
        else if (gamepad1.dpad_left) {
            turret.setHoodPos(turret.getHoodPos() - 0.05);
        }
        telemetry.addData("Current Turret Position", turret.getTurretPos());
        telemetry.addData("Current Flywheel RPM", turret.getFlywheelRPM());
        telemetry.addData("Current Hood Position", turret.getHoodPos());
        telemetry.addData("Press right bumper on Gamepad 1 to increase Turret Power", "Press Left Bumper on Gamepad 1 to decrease Turret Power");
        telemetry.addData("Press Dpad Up on Gamepad 1 to increase Flywheel Power", "Press Dpad Down on Gamepad 1 to decrease Flywheel Power");
        telemetry.addData("Press Dpad Right on Gamepad 1 to increase Hood Position", "Press Dpad Left on Gamepad 1 to decrease Hood Position");
        telemetry.update();
    }
    @Override
    public void stop(){
        super.stop();
    }
}
