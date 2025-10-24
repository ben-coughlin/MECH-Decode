package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Tuning")
public class TurretTuning extends RobotMasterPinpoint
{
    public boolean hasBeenActivated = false;
    @Override
    public void init() {}
    @Override
    public void init_loop() {}
    @Override
    public void start() {}
    @Override
    public void mainLoop() {
        super.mainLoop();
        if (gamepad1.right_bumper && !hasBeenActivated) {
            turret.setTurretPower(turret.getTurretPower() + 1);
            hasBeenActivated = true;
        }
        else if (gamepad1.left_bumper && !hasBeenActivated) {
            turret.setTurretPower(turret.getTurretPower() - 1);
            hasBeenActivated = true;
        }
        if (gamepad1.dpad_up && !hasBeenActivated) {
            turret.setFlywheelPower(turret.getFlywheelRPM() + 0.01);
            hasBeenActivated = true;
        }
        else if (gamepad1.dpad_down && !hasBeenActivated) {
            turret.setFlywheelPower(turret.getFlywheelRPM() - 0.01);
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
        telemetry.addData("Current Turret Position", turret.getTurretPos());
        telemetry.addData("Current Flywheel RPM", turret.getFlywheelRPM());
        telemetry.addData("Current Hood Position", turret.getHoodPos());
        telemetry.addData("Press right bumper on Gamepad 1 to increase Turret Power", "Press Left Bumper on Gamepad 1 to decrease Turret Power");
        telemetry.addData("Press Dpad Up on Gamepad 1 to increase Flywheel Power", "Press Dpad Down on Gamepad 1 to decrease Flywheel Power");
        telemetry.addData("Press Dpad Right on Gamepad 1 to increase Hood Position", "Press Dpad Left on Gamepad 1 to decrease Hood Position");
        telemetry.update();
    }
    @Override
    public void stop(){}
}
