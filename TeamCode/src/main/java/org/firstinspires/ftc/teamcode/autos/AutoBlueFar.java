package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.RobotMaster;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;

import java.util.HashSet;
import java.util.Set;
@Disabled
@Autonomous(name = "Blue Auto Far")
public class AutoBlueFar extends AutoMaster {

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
        return VisionUtils.isTagBlueGoal(tagId);
    }


    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(60, 11, Math.toRadians(90));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 11.000),

                                new Pose(59.000, 11.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    @Override
    protected PathChain getGrabPickup1(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59, 11),
                                new Pose(48.278, 39.009),
                                new Pose(14.513, 32.591)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

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
                                new Pose(14.513, 32.591),
                                new Pose(49.925, 30.471),
                                new Pose(59.572, 15.630)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    protected PathChain getGrabPickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.572, 15.630),
                                new Pose(66.886, 70.945),
                                new Pose(15.974, 57.565)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }



    @Override
    protected PathChain getScorePickup2(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.974, 57.565),
                                new Pose(50.411, 48.419),
                                new Pose(59.896, 15.468)
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
    protected PathChain getParkZone(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,

                                new Pose(40.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180))

                .build();
    }
    @Override
    protected PathChain getParkGate(Follower follower)
    {
        return null;
    }



}

