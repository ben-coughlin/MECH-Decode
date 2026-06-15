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
        return VisionUtils.isTagRedGoal(tagId);
    }

    @Override
    protected double getTargetTurretAngle() {
        return 63;
    }


    //pathing
    @Override
    protected Pose getStartPose() {
        return new Pose(81.5, 11, Math.toRadians(0));
    }

    @Override
    protected PathChain getScorePreload(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(81.5, 11),

                                new Pose(81.5, 11)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    @Override
    protected PathChain getGrabCycleOne(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(81.500, 11.000),
                                new Pose(95.177, 30.513),
                                new Pose(123.351, 29.113)
                        )
                )
                .setTangentHeadingInterpolation()

                .build();

    }

    @Override
    protected PathChain getHitGate(Follower follower) {
        return null;
    }


    @Override
    protected PathChain getScoreCycleOne(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.351, 29.113),
                                new Pose(97.864, 8.379),
                                new Pose(82.087, 10.887)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


    }

    @Override //
    protected PathChain getGrabCycleTwo(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(82.087, 10.887),
                                new Pose(103.230, 4.384),
                                new Pose(132.735, 4.792),
                                new Pose(129.795, 24.206)
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(40))

                .build();
    }



    @Override
    protected PathChain getScoreCycleTwo(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(129.795, 24.206),
                                new Pose(110.512, 3.478),
                                new Pose(81.604, 13.254)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
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
    protected PathChain getParkGate(Follower follower) {
        return null;
    }

    @Override
    protected PathChain getParkZone(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                               follower::getPose,

                                new Pose(104.074, 14.334)
                        )
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(0))

                .build();
    }


}

