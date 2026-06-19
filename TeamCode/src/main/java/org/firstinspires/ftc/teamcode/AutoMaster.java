package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.Pattern;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Collections;
import java.util.Set;


public abstract class AutoMaster extends RobotMaster {

    private Follower follower;
    private Timer pathTimer, stageStartTimer, opmodeTimer, centerResetTimer;
    protected boolean stageInit = true;
    public static String selectedProgram;


    protected static boolean doZonePark = true;

    protected enum AutoStage
    {
        scorePreload,
        grabCycleOne,
        hitGate,
        scoreCycleOne,
        grabCycleTwo,
        scoreCycleTwo,
        grabCycleThree,
        scoreCycleThree,
        grabCycleFour,
        scoreCycleFour,
        park,
        shootPrep,
        shoot,
        centerTurret,
        endBehavior,


    }

    public static int pathState = AutoStage.scorePreload.ordinal();
    private int pathAfterStateShotOrdinal = AutoStage.park.ordinal();
    protected abstract boolean isCorrectGoalTag(int tagId);
    protected abstract boolean isAutoFar();

    protected abstract double getTargetTurretAngle();

    protected Set<Integer> getSkippedStages() {
        return Collections.emptySet();  // Default: skip nothing
    }

    //points
    protected abstract Pose getStartPose();
    protected abstract PathChain getScorePreload(Follower follower);
    protected abstract PathChain getGrabCycleOne(Follower follower);
    protected abstract PathChain getHitGate(Follower follower);
    protected abstract PathChain getScoreCycleOne(Follower follower);
    protected abstract PathChain getGrabCycleTwo(Follower follower);
    protected abstract PathChain getScoreCycleTwo(Follower follower);
    protected abstract PathChain getGrabCycleThree(Follower follower);
    protected abstract PathChain getScoreCycleThree(Follower follower);
    protected abstract PathChain getGrabCycleFour(Follower follower);
    protected abstract PathChain getScoreCycleFour(Follower follower);
    protected abstract PathChain getParkGate(Follower follower);
    protected abstract PathChain getParkZone(Follower follower);


    private PathChain scorePreload;

    private PathChain grabCycleOne;
    private PathChain hitGate;
    private PathChain scoreCycleOne;
    private PathChain grabCycleTwo;
    private PathChain scoreCycleTwo;
    private PathChain grabCycleThree;
    private PathChain scoreCycleThree;
    private PathChain grabCycleFour;
    private PathChain scoreCycleFour;

    private double maxSpeed = 1;






    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();
        isAuto = true;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        stageStartTimer = new Timer();
        opmodeTimer.resetTimer();
        getSkippedStages(); //running this will set the park booleans, and then we call it later to get stages

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(getStartPose());
        follower.update();
        telemetry.addData("Selected", selectedProgram);
        maxSpeed = (isAutoFar()) ? .85 : 1;

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        telemetry.addData("Selected", selectedProgram);
        super.init_loop();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        super.start();
        opmodeTimer.resetTimer();
    }
    @Override
    public void loop() {
        super.mainLoop();
        follower.update();

        boolean hasValidVision = Limelight.currResult != null
                && Limelight.currResult.isValid()
                && isCorrectGoalTag(VisionUtils.getTagId(Limelight.currResult));

        double tx = 0; // use safe values for both of these if vision fails
        if(hasValidVision)
        {
             tx = isCorrectGoalTag(VisionUtils.getTagId(Limelight.currResult)) ? Limelight.currResult.getTx() : 0;
        }



        turret.aimTurret(
                hasValidVision,
                tx,
                0,
                true,
                getTargetTurretAngle(),
                follower.getVelocity().getMagnitude() < 2
        );

        updatePaths();



    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        super.stop();
    }

    public void buildPaths() {

        scorePreload = getScorePreload(follower);

        grabCycleOne = getGrabCycleOne(follower);
       // hitGate = getHitGate(follower);

        scoreCycleOne = getScoreCycleOne(follower);

        grabCycleTwo = getGrabCycleTwo(follower);

        scoreCycleTwo = getScoreCycleTwo(follower);

        grabCycleThree = getGrabCycleThree(follower);

        scoreCycleThree = getScoreCycleThree(follower);

        grabCycleFour = getGrabCycleFour(follower);

        scoreCycleFour = getScoreCycleFour(follower);

        //we don't build these at init because we use lazy generation to get the robot pose at the time we need to park
        //parkGate = getParkGate(follower);
        //parkZone = getParkZone(follower);
    }

    public void updatePaths() {


        if (pathState == AutoStage.scorePreload.ordinal()) {
            if (stageInit) {
                intake.turnIntakeOn();
                shooterSubsystem.spinUp();
                initState();
                if (scorePreload != null) {
                    follower.followPath(scorePreload);
                } else {
                    nextStage();
                }
            }
            shooterSubsystem.updateSpin();

            if (!follower.isBusy() || (isAutoFar())) {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabCycleOne.ordinal());
            }
        }

        if (pathState == AutoStage.grabCycleOne.ordinal()) {
            if (stageInit) {
                initState();
                intake.turnIntakeOn();
                transfer.turnTransferOn();
                if (grabCycleOne != null) {
                    follower.followPath(grabCycleOne, true);
                } else {
                    nextStage();
                }

            }

            if (!follower.isBusy()) {
                nextStage(AutoStage.scoreCycleOne.ordinal());
            }
        }

//        if(pathState == AutoStage.hitGate.ordinal())
//        {
//            if(stageInit)
//            {
//                initState();
//                if(hitGate != null) { follower.followPath(hitGate); }
//                else { nextStage(); }
//            }
//
//            if(!follower.isBusy())
//            {
//                nextStage(AutoStage.scoreMiddleSpike.ordinal());
//            }
//        }

        if (pathState == AutoStage.scoreCycleOne.ordinal()) {
            if (stageInit) {
                shooterSubsystem.spinUp();

                if (scoreCycleOne != null) {
                    follower.followPath(scoreCycleOne);
                } else {
                    nextStage();
                }
                initState();
            }

            shooterSubsystem.updateSpin();

            if (!follower.isBusy()) {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabCycleTwo.ordinal());
            }
        }

        if (pathState == AutoStage.grabCycleTwo.ordinal()) {
            if (stageInit) {

                intake.turnIntakeOn();
                transfer.turnTransferOn();
                if (grabCycleTwo != null) {
                    follower.followPath(grabCycleTwo, maxSpeed, true);
                } else {
                    nextStage();
                }
                initState();

            }

            if (!follower.isBusy() || stageStartTimer.getElapsedTimeSeconds() > 3) {
                nextStage(AutoStage.scoreCycleTwo.ordinal());
            }
        }
        if (pathState == AutoStage.scoreCycleTwo.ordinal()) {
            if (stageInit) {
                initState();
                shooterSubsystem.spinUp();
                if (scoreCycleTwo != null) {
                    follower.followPath(scoreCycleTwo, maxSpeed, true);
                } else {
                    nextStage();
                }

            }

            shooterSubsystem.updateSpin();


            if (!follower.isBusy()) {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabCycleThree.ordinal());
            }
        }

        if (pathState == AutoStage.grabCycleThree.ordinal()) {
            if (stageInit) {

                intake.turnIntakeOn();
                transfer.turnTransferOn();
                if (grabCycleThree != null) {
                    follower.followPath(grabCycleThree, maxSpeed, true);
                } else {
                    nextStage();
                }
                initState();

            }

            if (!follower.isBusy() || stageStartTimer.getElapsedTimeSeconds() > 3) {
                nextStage(AutoStage.scoreCycleThree.ordinal());
            }
        }
        if (pathState == AutoStage.scoreCycleThree.ordinal()) {
            if (stageInit) {
                initState();
                shooterSubsystem.spinUp();
                if (scoreCycleThree != null) {
                    follower.followPath(scoreCycleThree, maxSpeed, true);
                } else {
                    nextStage();
                }

            }

            shooterSubsystem.updateSpin();


            if (!follower.isBusy() && stageStartTimer.getElapsedTime() > 30) {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabCycleFour.ordinal());
            }
        }
        if (pathState == AutoStage.grabCycleFour.ordinal()) {
            if (stageInit) {
                if (grabCycleFour != null) {
                    intake.turnIntakeOn();
                    transfer.turnTransferOn();
                    follower.followPath(grabCycleFour, maxSpeed, true);
                    initState();
                } else {
                    nextStage();
                }
            }

            if (!follower.isBusy() || stageStartTimer.getElapsedTimeSeconds() > 3.5) //stupid ugly hack bc isBusy isn't working well on far but idgaf anymore
            {
                nextStage(AutoStage.scoreCycleFour.ordinal());
            }
        }

        if (pathState == AutoStage.scoreCycleFour.ordinal()) {
            if (stageInit) {
                shooterSubsystem.spinUp();
                intake.turnIntakeOn();
                if (scoreCycleFour != null) {
                    follower.followPath(scoreCycleFour, maxSpeed, true);
                } else {
                    nextStage();
                }
                initState();
            }
            shooterSubsystem.updateSpin();


            if (!follower.isBusy()) {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.park.ordinal());
            }

        }

        if (pathState == AutoStage.park.ordinal()) {
            if (stageInit) {
                PathChain parkGate = getParkGate(follower);
                PathChain parkZone = getParkZone(follower);

                if (parkGate != null && !doZonePark) {
                    follower.followPath(parkGate);
                } else if (parkZone != null && doZonePark) {
                    follower.followPath(parkZone);
                } else {
                    nextStage();
                }
                initState();
            }

            if (!follower.isBusy() && stageStartTimer.getElapsedTime() > 30) {
                nextStage(AutoStage.endBehavior.ordinal());
            }
        }


        if (pathState == AutoStage.shootPrep.ordinal()) {
            if (stageInit) {
                shooterSubsystem.spinUp();
                initState();
            }
            shooterSubsystem.updateSpin();

            if (!follower.isBusy() || stageStartTimer.getElapsedTimeSeconds() > 3) {
                nextStage(AutoStage.shoot.ordinal());
            }
        }
        if (pathState == AutoStage.shoot.ordinal()) {
            if (stageInit) {
                initState();
                ShooterSubsystem.isFlywheelSpun = true;
                shooterSubsystem.startShotSequence(isAuto);
            }


            if (stageStartTimer.getElapsedTime() > 600) {
                shooterSubsystem.stopAutoShot();
                intake.turnIntakeOff();
                nextStage(pathAfterStateShotOrdinal);

            }

        }
        if (pathState == AutoStage.centerTurret.ordinal()) {
            if (isAutoFar()) {
                if (Math.abs(turret.getTurretDeg()) > 2.0 && centerResetTimer.getElapsedTimeSeconds() < 2.0) {
                    turret.aimTurret(false, 0, 0, false, 0, true);
                } else {
                    turret.resetEncoder(); // just the encoder reset, no aimTurret call
                    nextStage(AutoStage.endBehavior.ordinal());
                }
            } else {
                nextStage(AutoStage.endBehavior.ordinal());
            }
        }

        if (pathState == AutoStage.endBehavior.ordinal()) {
            drive.hardStopMotors();
            turret.turnOffFlywheel();
            shooterSubsystem.runStopActions();
            intake.turnIntakeOff();
        }
    }

    protected void nextStage() {

        do {
            pathState++;
        } while (getSkippedStages().contains(pathState));

        pathTimer.resetTimer();
        stageInit = true;
    }

    protected void nextStage(int stage) {
        pathState = stage;

        while (getSkippedStages().contains(pathState)) {
            pathState++;
        }

        pathTimer.resetTimer();
        stageInit = true;
    }

    protected void nextStage(int stage, int afterShotOrdinal) {
        pathAfterStateShotOrdinal = afterShotOrdinal;
        pathState = stage;
        while (getSkippedStages().contains(pathAfterStateShotOrdinal)) {
            pathAfterStateShotOrdinal++;
        }


        pathTimer.resetTimer();
        stageInit = true;
    }

    protected void initState()
    {
        opmodeTimer.resetTimer();
        stageStartTimer.resetTimer();
        stageInit = false;
    }

    public static void addAutoTelemetry(Telemetry telemetry)
    {
     //   telemetry.addData("X: ", follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getHeading()));
    }



}

