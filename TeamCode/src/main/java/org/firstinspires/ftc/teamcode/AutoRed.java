package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRed extends AutoMaster {

    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }

    @Override
    protected double getManualAimSpeed() {
        return 0.23;
    }

    @Override
    protected CurvePoint getShootPreloadEndPoint() {
        return new CurvePoint(-33.5, 0, 0.8 * SCALE_FACTOR, 0.30 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getShootPreloadHeading() {
        return Math.toRadians(-90);
    }

    @Override
    protected int getShootPreloadFollowCurveTolerance() {
        return 2;
    }

    @Override
    protected CurvePoint getFirstThreeBallsEndPoint() {
        return new CurvePoint(-24, -28, 0.8 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getFirstThreeBallsHeading() {
        return Math.toRadians(160);
    }

    @Override
    protected int getFirstThreeBallsFollowCurveTolerance() {
        return 2;
    }

    @Override
    protected CurvePoint getIntakeFirstThreeBallsEndPoint() {
        return new CurvePoint(-8, -29, 0.14 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getIntakeFirstThreeBallsHeading() {
        return Math.toRadians(90);
    }

    @Override
    protected int getIntakeFirstThreeBallsFollowCurveTolerance() {
        return 2;
    }

    @Override
    protected CurvePoint getShootingPointEndPoint() {
        return new CurvePoint(-33, -28, 0.8 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getShootingPointHeading() {
        return Math.toRadians(-90);
    }

    @Override
    protected int getShootingPointFollowCurveTolerance() {
        return 2;
    }

    @Override
    protected CurvePoint getSecondThreeBallsEndPoint() {
        return new CurvePoint(-33, -53, 0.8 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getSecondThreeBallsHeading() {
        return Math.toRadians(180);
    }

    @Override
    protected int getSecondThreeBallsFollowCurveTolerance() {
        return 2;
    }

    @Override
    protected CurvePoint getIntakeSecondThreeBallsEndPoint() {
        return new CurvePoint(-8, -54, 0.15 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getIntakeSecondThreeBallsHeading() {
        return Math.toRadians(90);
    }

    @Override
    protected int getIntakeSecondThreeBallsTolerance() {
        return 1;
    }


    @Override
    protected CurvePoint getShootPointToEndEndPoint() {
        return new CurvePoint(-31, -25, 1 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getShootPointToEndHeading() {
        return Math.toRadians(-90);
    }

    @Override
    protected int getShootPointToEndFollowCurveTolerance() {
        return 3;
    }

    @Override
    protected CurvePoint getDriveOutsideShootZoneEndPoint() {
        return new CurvePoint(-31, -35, 1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getDriveOutsideShootZoneHeading() {
        return Math.toRadians(0);
    }

    @Override
    protected int getDriveOutsideShootZoneFollowCurveTolerance() {
        return 1;
    }
}