package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Set;

@Autonomous(name = "Blue Auto Gate")

public class AutoBlueGate extends AutoMaster {


    //other general differences

    @Override
    protected Set<Integer> getSkippedStages() {
        //return Set.of(AutoStage.hitGate.ordinal()); //this would skip hitting the gate
        return Set.of(AutoStage.grabThirdBalls.ordinal(), AutoStage.scoreThirdBalls.ordinal());
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
        return new Pose(9, 115, Math.toRadians(180));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 115.000),
                                new Pose(50.000, 115.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 115.000),
                                new Pose(62.354, 78.608),
                                new Pose(12.421, 85.000)
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
                                new Pose(12.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();
    }

    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.421, 85.000),
                                new Pose(46.597, 96.634)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(40.000, 105.000),
                                new Pose(64.518, 54.099),
                                new Pose(16.500, 60.000)
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
                                new Pose(42.092, 111.275)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

    }

    @Override
    protected PathChain getGrabPickup3(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(40.000, 105.000),
                                new Pose(68.678, 27.502),
                                new Pose(12.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

    }

    @Override
    protected PathChain getScorePickup3(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 36.000),

                                new Pose(37.537, 104.483)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
    }

    @Override
    protected PathChain getPark(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 105.000),

                                new Pose(18.000, 70.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
    }


}
