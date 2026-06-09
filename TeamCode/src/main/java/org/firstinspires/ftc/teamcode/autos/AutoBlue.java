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
                                new Pose(44.301, 88.806)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(43.984, 89.438),
                                new Pose(50.953, 66.035),
                                new Pose(0.745, 72.897)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

    }
//no gate since its part of pickup 1
//    @Override
//    protected PathChain getHitGate(Follower follower) {
//        return follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(-0.203, 68.569),
//                                new Pose(28.803, 61.698),
//                                new Pose(33.873, 105.845)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
//
//                .build();
//
//    }

    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(0.745, 72.897),
                                new Pose(36.550, 62.014),
                                new Pose(44.110, 83.161)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(44.110, 83.161),
                                new Pose(25.351, 39.815),
                                new Pose(0.5, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

    }

    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(0.5, 60),
                                new Pose(5.375, 42.055),
                                new Pose(43.794, 78.418)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                .build();

    }

    @Override
    protected PathChain getGrabPickup3(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.794, 78.418),
                                new Pose(9.429, 79.050)

                        )
                )
                .setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getScorePickup3(Follower follower) {
        return follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(9.429, 79.050),
                                        new Pose(43.426, 78.271)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

    }

    @Override
    protected PathChain getParkGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose, //this gets the current bot position as the start point

                                new Pose(20.360, 70.155)
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
