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
        return VisionUtils.isTagBlueGoal(tagId);
    }

    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(8, 115, Math.toRadians(180));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.088, 115.000),

                                new Pose(35.210, 115.650)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 115.000),
                                new Pose(55.860, 78.932),
                                new Pose(10.901, 84.838)
                        )
                ).setTangentHeadingInterpolation()

                .build();


    }

    @Override
    protected PathChain getHitGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.901, 84.838),
                                new Pose(34.868, 73.737),
                                new Pose(12.240, 71.786)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();
    }

    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.240, 71.786),

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
                                new Pose(13.188, 59.675)
                        )
                ).setTangentHeadingInterpolation()

                .build();

    }

    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return  follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.188, 59.675),
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
                                new Pose(62.021, 26.041),
                                new Pose(10.941, 35.838)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

//    @Override
//    protected PathChain getScorePickup3(Follower follower) {
//        return follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(40.000, 105.000),
//                                new Pose(68.678, 27.502),
//                                new Pose(15.000, 36.000)
//                        )
//                ).setTangentHeadingInterpolation()
//
//                .build();
//    }

    @Override
    protected PathChain getParkGate(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose, //this gets the current bot position as the start point

                                new Pose(20.360, 70.155)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
    }

    @Override
    protected PathChain getParkZone(Follower follower) {

        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,
                                new Pose(38.692, 117.733)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();
    }


}
