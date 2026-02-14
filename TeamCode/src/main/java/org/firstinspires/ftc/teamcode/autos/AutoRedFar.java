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
@Autonomous(name = "Red Auto Far")
public class AutoRedFar extends AutoMaster {

    public static boolean DO_FIRST_CYCLE = true;
    public static boolean DO_SECOND_CYCLE = true;

    public static boolean DO_ZONE_PARK = true;

    @Override
    protected Set<Integer> getSkippedStages() {
        Set<Integer> skipped = new HashSet<>();

        //the master will auto-skip any null paths so we only add the things we might want to turn off
        if (!DO_FIRST_CYCLE) {
            skipped.add(AutoStage.grabFirstBalls.ordinal());
            skipped.add(AutoStage.scoreFirstBalls.ordinal());
        }
        if(!DO_SECOND_CYCLE){
            skipped.add(AutoStage.grabSecondBalls.ordinal());
            skipped.add(AutoStage.scoreSecondBalls.ordinal());
        }

        AutoMaster.doZonePark = DO_ZONE_PARK;

        return skipped;
    }
    @Override
    protected boolean isAutoFar()
    {
        return true;
    }

    @Override
    protected boolean isCorrectGoalTag(int tagId) {
        return VisionUtils.isTagRedGoal(tagId);
    }


    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(88, 8, Math.toRadians(90));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.000, 8.000),

                                new Pose(87.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                .build();
    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(87, 8),
                                new Pose(65.635, 34.021),
                                new Pose(125.621, 31.173)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

    }

    @Override
    protected PathChain getHitGate(Follower follower) {
        return null;
    }


    @Override
    protected PathChain getScorePickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.621, 31.173),
                                new Pose(94.236, 30.149),
                                new Pose(88.161, 13.309)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.161, 13.309),
                                new Pose(67.627, 61.103),
                                new Pose(123.208, 58.552)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }



    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.208, 58.552),
                                new Pose(93.477, 51.024),
                                new Pose(87.678, 13.470)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    protected PathChain getGrabPickup3(Follower follower) {
        return null;
    }

//    @Override
//    protected PathChain getScorePickup3(Follower follower) {
//        return null;
//    }

    @Override
    protected PathChain getParkGate(Follower follower) {
        return null;
    }

    @Override
    protected PathChain getParkZone(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                               follower::getPose,

                                new Pose(108.827, 12.747)
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(0))

                .build();
    }


}

