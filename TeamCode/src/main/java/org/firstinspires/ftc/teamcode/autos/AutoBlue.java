package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;

import java.util.HashSet;
import java.util.Set;
@Disabled
@Autonomous(name = "Blue Auto")

public class AutoBlue extends AutoMaster {

    public static boolean GATE_HIT = true;
    public static boolean DO_FIRST_CYCLE = true;
    public static boolean DO_SECOND_CYCLE = true;
    public static boolean DO_THIRD_CYCLE = true;
    public static boolean DO_FOURTH_CYCLE = true;
    public static boolean DO_ZONE_PARK = true;
    @Override
    protected boolean isAutoFar()
    {
        return false;
    }
    @Override
    protected Set<Integer> getSkippedStages() {
                Set<Integer> skipped = new HashSet<Integer>() {};

                // Build the skip set based on gate settings
                if (!GATE_HIT) {
                        skipped.add(AutoStage.hitGate.ordinal());
                    }

                if (!DO_FIRST_CYCLE) {
                        skipped.add(AutoStage.grabMiddleSpike.ordinal());
                        skipped.add(AutoStage.scoreMiddleSpike.ordinal());
                    }

                if (!DO_SECOND_CYCLE) {
                        skipped.add(AutoStage.grabGateCycle.ordinal());
                        skipped.add(AutoStage.scoreGateCycle.ordinal());
                    }

                if (!DO_THIRD_CYCLE) {
                        skipped.add(AutoStage.grabGateCycleTwo.ordinal());
                        skipped.add(AutoStage.scoreGateCycleTwo.ordinal());
                    }
                if(!DO_FOURTH_CYCLE) {
                    skipped.add(AutoStage.grabCloseSpike.ordinal());
                }

        AutoMaster.doZonePark = DO_ZONE_PARK;


                return skipped;
            }



    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagBlueGoal(tagId);
    }

    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(10, 115, Math.toRadians(180));
    }

    @Override
    
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10, 115.000),
                                new Pose(43.984, 89.438)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

    }

    @Override
    protected PathChain getGrabMiddleSpike(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(43.984, 89.438),
                                new Pose(33.992, 66.402),
                                new Pose(1.1, 70)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

    }
    @Override //unused
    protected PathChain getHitGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11, 65),
                                new Pose(4, 65)
                        )
                ).setTangentHeadingInterpolation()
                .build();

    }

    @Override
    protected PathChain getScoreMiddleSpike(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(1.1, 70),
                                new Pose(30.226, 67.863),
                                new Pose(44.110, 83.161)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    @Override
    protected PathChain getGrabGateCycle(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.110, 83.161),
                                new Pose(10.118, 43.003),
                                new Pose(-3.000, 55.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(142))
                .build();

    }

    @Override
    protected PathChain getScoreGateCycle(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(-3, 55.5),
                                new Pose(23.399, 32.253),
                                new Pose(43.794, 78.418)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(135))
                .build();

    }

    //we still wanna be able to disable or tweak the second gate cycle
    @Override
    protected PathChain getGrabGateCycleTwo(Follower follower) {
        return getGrabGateCycle(follower);
    }

    @Override
    protected PathChain getScoreGateCycleTwo(Follower follower) {
        return getScoreGateCycle(follower);

    }

    @Override
    protected PathChain getGrabCloseSpike(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.794, 78.418),
                                new Pose(7, 79.050)

                        )
                )
                .setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getScoreCloseSpike(Follower follower) {
        return follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(7, 79.050),
                                        new Pose(43.426, 78.271)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

    }

    @Override
    protected PathChain getParkGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose, //this gets the current bot position as the start point

                                new Pose(30, 70.155)
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180))

                .build();
    }

    @Override
    protected PathChain getParkZone(Follower follower) {

        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,
                                new Pose(48.688, 106.750)
                        )
        ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180))

                .build();
    }


}
