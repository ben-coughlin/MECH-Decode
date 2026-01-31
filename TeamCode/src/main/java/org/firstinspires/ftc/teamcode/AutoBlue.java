package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.Collections;
import java.util.Set;

public class AutoBlue extends AutoMaster {


    //other general differences

    @Override
    protected Set<Integer> getSkippedStages() {
        //return Set.of(AutoStage.hitGate.ordinal()); //this would skip hitting the gate
        return Collections.emptySet();
    }
    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagBlueGoal(tagId);
    }

    @Override
    protected int getStateAfterHitGate() {
        return AutoStage.scoreFirstBalls.ordinal(); // Hit gate after first score
    }



    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(9, 116, Math.toRadians(180));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 115.000),

                                new Pose(50.000, 115.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 115.000),
                                new Pose(62.354, 78.608),
                                new Pose(16.421, 85.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getHitGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.421, 85.000),
                                new Pose(34.868, 73.737),
                                new Pose(15.000, 70.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();
    }

    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 70.000),

                                new Pose(40.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                .build();
    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(40.000, 105.000),
                                new Pose(64.518, 54.099),
                                new Pose(15.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(36.306, 78.223),
                                new Pose(40.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

    }

    @Override
    protected PathChain getGrabPickup3(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(40.000, 105.000),
                                new Pose(68.678, 27.502),
                                new Pose(15.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

    }

    @Override
    protected PathChain getScorePickup3(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 36.000),
                                new Pose(17.284, 48.690),
                                new Pose(31.430, 55.779),
                                new Pose(13.456, 77.070),
                                new Pose(44.305, 79.392),
                                new Pose(34.111, 93.135),
                                new Pose(40.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();
    }

    @Override
    protected PathChain getPark(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 105.000),

                                new Pose(25.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();
    }


}
