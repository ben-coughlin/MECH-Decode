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
            skipped.add(AutoStage.grabCycleOne.ordinal());
            skipped.add(AutoStage.scoreCycleOne.ordinal());
        }
        if(!DO_SECOND_CYCLE){
            skipped.add(AutoStage.grabCycleTwo.ordinal());
            skipped.add(AutoStage.scoreCycleTwo.ordinal());
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

    @Override
    protected double getTargetTurretAngle() {
        return -64;
    }



    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(60, 11, Math.toRadians(180));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 11.000),

                                new Pose(60, 11)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override //close spike
    protected PathChain getGrabCycleOne(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 11.000),
                                new Pose(46.323, 30.513),
                                new Pose(18.149, 29.113)
                        )
                ).setTangentHeadingInterpolation()

                .build();

    }

    //@Override
    protected PathChain getHitGate(Follower follower) {
        return null;
    }


    @Override
    protected PathChain getScoreCycleOne(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.149, 29.113),
                                new Pose(43.636, 8.379),
                                new Pose(59.413, 10.887)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override //human player zone cycle 1
    protected PathChain getGrabCycleTwo(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.572, 11.836),
                                new Pose(37.163, 10.550),
                                new Pose(11.611, 4.634),
                                new Pose(11.389, 11.874)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }



    @Override
    protected PathChain getScoreCycleTwo(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.389, 11.874),
                                new Pose(1.581, 28.932),
                                new Pose(59.896, 13.254)

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                .build();

    }

    @Override
    protected PathChain getGrabCycleThree(Follower follower) {
        return getGrabCycleTwo(follower);
    }



    @Override
    protected PathChain getScoreCycleThree(Follower follower) {
        return getScoreCycleTwo(follower);

    }

    @Override
    protected PathChain getGrabCycleFour(Follower follower) {

        return getGrabCycleTwo(follower);
    }

    @Override
    protected PathChain getScoreCycleFour(Follower follower) {
        return getScoreCycleTwo(follower);
    }

    @Override
    protected PathChain getParkZone(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,

                                new Pose(37.426, 14.334)
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

