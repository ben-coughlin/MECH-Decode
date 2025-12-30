package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBlue extends AutoMaster {

    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagBlueGoal(tagId);
    }

    @Override
    protected double getManualAimSpeed() {
        return -0.37;
    }

    @Override
    protected CurvePoint getShootPreloadEndPoint() {
        return new CurvePoint(-31, 0, 0.8 * SCALE_FACTOR, 0.30 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getShootPreloadHeading() {
        return Math.toRadians(-90);
    }

    @Override
    protected int getShootPreloadFollowCurveTolerance() {
        return 1;
    }

    @Override
    protected CurvePoint getFirstThreeBallsEndPoint() {
        return new CurvePoint(-28, 29, 0.8 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getFirstThreeBallsHeading() {
        return Math.toRadians(0);
    }

    @Override
    protected int getFirstThreeBallsFollowCurveTolerance() {
        return 1;
    }

    @Override
    protected CurvePoint getIntakeFirstThreeBallsEndPoint() {
        return new CurvePoint(-6, 29, 0.115 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getIntakeFirstThreeBallsHeading() {
        return Math.toRadians(90);
    }

    @Override
    protected int getIntakeFirstThreeBallsFollowCurveTolerance() {
        return 1;
    }

    @Override
    protected CurvePoint getShootingPointEndPoint() {
        return new CurvePoint(-31, 30, 0.8 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getShootingPointHeading() {
        return Math.toRadians(-90);
    }

    @Override
    protected int getShootingPointFollowCurveTolerance() {
        return 3;
    }

    @Override
    protected CurvePoint getSecondThreeBallsEndPoint() {
        return new CurvePoint(-33, 55, 0.8 * SCALE_FACTOR, 0.1 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getSecondThreeBallsHeading() {
        return Math.toRadians(45);
    }

    @Override
    protected int getSecondThreeBallsFollowCurveTolerance() {
        return 1;
    }

    @Override
    protected void onDriveToSecondThreeBallsComplete() {
        shooterSubsystem.turnOff();
        nextStage(progStates.endBehavior.ordinal());
    }

    @Override
    protected CurvePoint getIntakeSecondThreeBallsEndPoint() {
        return new CurvePoint(-6, 55, 0.115 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.3);
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
    protected void onIntakeSecondThreeBallsComplete() {
        isLookingAtObelisk = false;
        spindexer.rotateToNextSlotInPattern();
        if ((startTime - SystemClock.uptimeMillis()) * 1000 > 26 && spindexer.isAtTargetPosition()) {
            nextStage(progStates.endBehavior.ordinal());
        } else {
            nextStage(progStates.driveToShootPointToEnd.ordinal());
        }
    }

    @Override
    protected CurvePoint getShootPointToEndEndPoint() {
        return new CurvePoint(-31, 25, 1 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getShootPointToEndHeading() {
        return Math.toRadians(-90);
    }

    @Override
    protected int getShootPointToEndFollowCurveTolerance() {
        return 1;
    }

    @Override
    protected CurvePoint getDriveOutsideShootZoneEndPoint() {
        return new CurvePoint(-31, 35, 1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.3);
    }

    @Override
    protected double getDriveOutsideShootZoneHeading() {
        return Math.toRadians(0);
    }

    @Override
    protected int getDriveOutsideShootZoneFollowCurveTolerance() {
        return 1;
    }

    @Override
    protected void handleDriveToSecondThreeBalls() {
        if (stageFinished) {
            past5In = false;
            isLookingAtObelisk = true;  // Blue-specific behavior
            initializeStateVariables();
        }
        super.handleDriveToSecondThreeBalls();
    }
}