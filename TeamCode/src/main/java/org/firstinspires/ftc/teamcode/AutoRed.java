package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Collections;
import java.util.Set;

@Autonomous(name = "Red Auto")

public class AutoRed extends AutoMaster {


    //other general differences

    @Override
    protected Set<Integer> getSkippedStages() {
        //return Set.of(AutoStage.hitGate.ordinal()); //this would skip hitting the gate
        return Set.of(AutoStage.hitGate.ordinal(), AutoStage.grabThirdBalls.ordinal(), AutoStage.scoreThirdBalls.ordinal());
    }
    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }

    @Override
    protected int getStateAfterHitGate() {
        return AutoStage.scoreFirstBalls.ordinal(); // Hit gate after first score
    }



    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(120, 121, Math.toRadians(0));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.678, 119.908),

                                new Pose(85.000, 120.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 120.000),
                                new Pose(77.218, 82.927),
                                new Pose(119.222, 90.422)
                        )
                ).setTangentHeadingInterpolation()

                .build();

    }

    @Override
    protected PathChain getHitGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(119.222, 90.422),
                                new Pose(87.927, 113.270),
                                new Pose(118.990, 87.437)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

    }

    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.990, 87.437),
                                new Pose(93.537, 81.321),
                                new Pose(106.673, 108.282),
                                new Pose(93.158, 105.385)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))

                .build();
    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(93.158, 105.385),
                                new Pose(66.771, 57.761),
                                new Pose(120.349, 60.657)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                .build();
    }

    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(120.349, 60.657),
                                new Pose(104.837, 78.065),
                                new Pose(127.374, 119.506),
                                new Pose(97.256, 108.758)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();



    }

    @Override
    protected PathChain getGrabPickup3(Follower follower) {
        return null;
    }

    @Override
    protected PathChain getScorePickup3(Follower follower) {
        return null;
    }

    @Override
    protected PathChain getPark(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(97.256, 108.758),

                                new Pose(102.362, 79.464)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();

    }


}
