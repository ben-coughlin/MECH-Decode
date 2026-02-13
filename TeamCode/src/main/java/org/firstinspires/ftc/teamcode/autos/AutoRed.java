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
    public static boolean DO_GATE_PARK = true;

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
            skipped.add(AutoStage.grabFirstBalls.ordinal());
            skipped.add(AutoStage.scoreFirstBalls.ordinal());
        }

        if (!DO_SECOND_CYCLE) {
            skipped.add(AutoStage.grabSecondBalls.ordinal());
            skipped.add(AutoStage.scoreSecondBalls.ordinal());
        }

        if (!DO_THIRD_CYCLE) {
            skipped.add(AutoStage.grabThirdBalls.ordinal());

        }
        if (DO_GATE_PARK) {
            skipped.add(AutoStage.parkZone.ordinal());
        } else {
            skipped.add(AutoStage.parkGate.ordinal());
        }

        return skipped;
    }
    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }


    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(120, 121, Math.toRadians(0));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.000, 115.000),

                                new Pose(94.000, 115.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(94.000, 115.000),
                                new Pose(81.646, 78.608),
                                new Pose(127.579, 85.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getHitGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.579, 85.000),
                                new Pose(109.132, 73.737),
                                new Pose(129.000, 70.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))

                .build();
    }

    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.000, 70.000),

                                new Pose(104.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(45))

                .build();
    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(104.000, 105.000),
                                new Pose(79.482, 54.099),
                                new Pose(129.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

    }

    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(129.000, 60.000),
                                new Pose(107.694, 78.223),
                                new Pose(104.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();
    }

    @Override
    protected PathChain getGrabPickup3(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(104.000, 105.000),
                                new Pose(75.322, 27.502),
                                new Pose(129.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

//    @Override
//    protected PathChain getScorePickup3(Follower follower) {
//        return follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(129.000, 36.000),
//                                new Pose(126.716, 48.690),
//                                new Pose(112.570, 55.779),
//                                new Pose(130.544, 77.070),
//                                new Pose(99.695, 79.392),
//                                new Pose(109.889, 93.135),
//                                new Pose(104.000, 105.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//
//                .build();
//
//    }

    @Override
    protected PathChain getParkGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,

                                new Pose(125.101, 69.667)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

    }

    @Override
    protected PathChain getParkZone(Follower follower) {

        return follower.pathBuilder().addPath(
                        new BezierLine(
                               follower::getPose,

                                new Pose(105.981, 120.372)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))

                .build();
    }


}
