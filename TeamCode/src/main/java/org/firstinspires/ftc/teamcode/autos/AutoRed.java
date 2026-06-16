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
@Autonomous(name = "Red Auto")

public class AutoRed extends AutoMaster {


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
            skipped.add(AutoStage.grabCycleOne.ordinal());
            skipped.add(AutoStage.scoreCycleOne.ordinal());
        }

        if (!DO_SECOND_CYCLE) {
            skipped.add(AutoStage.grabCycleTwo.ordinal());
            skipped.add(AutoStage.scoreCycleTwo.ordinal());
        }

        if (!DO_THIRD_CYCLE) {
            skipped.add(AutoStage.grabCycleThree.ordinal());
            skipped.add(AutoStage.scoreCycleThree.ordinal());
        }
        if(!DO_FOURTH_CYCLE) {
            skipped.add(AutoStage.grabCycleFour.ordinal());
        }

        AutoMaster.doZonePark = DO_ZONE_PARK;

        return skipped;
    }
    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }

    @Override
    protected double getTargetTurretAngle() {
        return 0;
    }

    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(127.5, 115, Math.toRadians(0));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.500, 115.000),

                                new Pose(97.516, 89.438)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override //mid spike &gate
    protected PathChain getGrabCycleOne(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(97.516, 89.438),
                                new Pose(97.346, 64.390),
                                new Pose(134.392, 66.146)
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(0))
                .build();
    }


    @Override //unused
    protected PathChain getHitGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
//                                new Pose(134.708, 61.304),
//                                new Pose(99.136, 65.304),
//                                new Pose(97.390, 83.161)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))

                .build();
    }

    @Override
    protected PathChain getScoreCycleOne(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.392, 66.146),
                                new Pose(98.180, 65.770),
                                new Pose(97.390, 83.161)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();
    }

    @Override //gate 1
    protected PathChain getGrabCycleTwo(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(97.390, 83.161),
                                new Pose(133.595, 42.687),
                                new Pose(141.393, 58)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(27))
                .build();
    }

    @Override
    protected PathChain getScoreCycleTwo(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(141.393, 58.5),
                                new Pose(104.504, 51.225),
                                new Pose(96.283, 81.896)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(27), Math.toRadians(45))
                .build();
    }

    @Override //gate 2
    protected PathChain getGrabCycleThree(Follower follower) {
        return getGrabCycleTwo(follower);
    }

    @Override
    protected PathChain getScoreCycleThree(Follower follower) {
        return getScoreCycleTwo(follower);

    }

    @Override //close spike
    protected PathChain getGrabCycleFour(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.283, 81.896),
                                new Pose(129.283, 80.947)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    protected PathChain getScoreCycleFour(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.283, 80.947),
                                new Pose(97.442, 82.065)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

    }

    @Override
    protected PathChain getParkGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,

                                new Pose(111.500, 70.155)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

    }

    @Override
    protected PathChain getParkZone(Follower follower) {

        return follower.pathBuilder().addPath(
                        new BezierLine(
                               follower::getPose,
                                new Pose(92.812, 106.750)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();
    }


}
