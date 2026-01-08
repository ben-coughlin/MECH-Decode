package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed extends TeleOpMaster {

    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagBlueGoal(tagId);
    }

    // Optional: Override timing methods if blue alliance needs different tuning
    // @Override
    // protected double getShootPrepTime() {
    //     return 2600; // Custom value for blue
    // }
}