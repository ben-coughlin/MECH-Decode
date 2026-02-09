package org.firstinspires.ftc.teamcode.teleops;

import org.firstinspires.ftc.teamcode.TeleOpMaster;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed extends TeleOpMaster {

    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }

    // Example: Override timing methods if blue alliance needs different tuning
    // @Override
    // protected double getShootPrepTime() {
    //     return 2600; // Custom value for red
    // }
}