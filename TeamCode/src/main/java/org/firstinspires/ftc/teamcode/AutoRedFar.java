package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRedFar extends AutoMasterFar {
    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }

    @Override
    protected CurvePoint getDriveOutsideShootZoneEndPoint() {
        return new CurvePoint(10, 0, 0.8, 0.8, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getDriveOutsideShootZoneHeading() {
        return Math.toRadians(180);
    }

    @Override
    protected int getDriveOutsideShootZoneFollowCurveTolerance() {
        return 1;
    }
}
