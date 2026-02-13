package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Collections;
import java.util.Set;


public abstract class AutoMaster extends RobotMaster {

    private Follower follower;
    private Timer pathTimer, stageStartTimer, opmodeTimer;
    protected boolean stageInit = true;
    public static String selectedProgram;



    protected enum AutoStage
    {
        scorePreload,
        grabFirstBalls,
        hitGate,
        scoreFirstBalls,
        grabSecondBalls,
        scoreSecondBalls,
        grabThirdBalls,
       // scoreThirdBalls,
        parkGate,
        parkZone,
        endBehavior,
        shootPrep,
        shoot,

    }

    public static int pathState = AutoStage.scorePreload.ordinal();
    private int pathAfterStateShotOrdinal = AutoStage.grabFirstBalls.ordinal();
    protected abstract boolean isCorrectGoalTag(int tagId);
    protected abstract boolean isAutoFar();

    protected Set<Integer> getSkippedStages() {
        return Collections.emptySet();  // Default: skip nothing
    }

    //points
    protected abstract Pose getStartPose();
    protected abstract PathChain getScorePreload(Follower follower);
    protected abstract PathChain getGrabPickup1(Follower follower);
    protected abstract PathChain getHitGate(Follower follower);
    protected abstract PathChain getScorePickup1(Follower follower);
    protected abstract PathChain getGrabPickup2(Follower follower);
    protected abstract PathChain getScorePickup2(Follower follower);
    protected abstract PathChain getGrabPickup3(Follower follower);
   // protected abstract PathChain getScorePickup3(Follower follower);
    protected abstract PathChain getParkGate(Follower follower);
    protected abstract PathChain getParkZone(Follower follower);

    private PathChain scorePreload, grabPickup1,  hitGate, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, parkGate, parkZone;




    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();
        isAutoFar = isAutoFar();
        isAuto = true;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        stageStartTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(getStartPose());
        telemetry.addData("Selected", selectedProgram);
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

        LLResult currVision = Limelight.getCurrResult();
        boolean hasValidVision = currVision != null
                && currVision.isValid()
                && isCorrectGoalTag(VisionUtils.getTagId(currVision));

        double tx = 0; // use safe values for both of these if vision fails
        double distance = 999;
        if(hasValidVision)
        {
             tx = isCorrectGoalTag(VisionUtils.getTagId(currVision)) ? Limelight.getCurrResult().getTx() : 0;
             distance = isCorrectGoalTag(VisionUtils.getTagId(currVision)) ? Limelight.getDistance() : 0;
        }


        // ALWAYS call aimTurret - it will use odometry if vision is lost
        turret.aimTurret(
                hasValidVision,
                tx,
                gamepad2.right_stick_x,
                distance,
                follower.getHeading(),
                follower.getAngularVelocity(),
                follower.getVelocity().getMagnitude()
        );

        updatePaths();
        telemetry.addData("superstructure state", pathState);

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

        grabPickup1 = getGrabPickup1(follower);
        hitGate = getHitGate(follower);

        scorePickup1 = getScorePickup1(follower);

        grabPickup2 = getGrabPickup2(follower);

        scorePickup2 = getScorePickup2(follower);

        grabPickup3 = getGrabPickup3(follower);

        //scorepickup isn't used bc we run out of time -leaving it in case we get past state
        //scorePickup3 = getScorePickup3(follower);

        //we don't build these at init because we use lazy generation to get the robot pose at the time we need to park
        //parkGate = getParkGate(follower);
        //parkZone = getParkZone(follower);
    }

    public void updatePaths() {
        telemetry.addData("Current Stage", AutoStage.values()[pathState].name());
        telemetry.addData("stageInit", stageInit);

        if(pathState == AutoStage.scorePreload.ordinal()){
            if(stageInit)
            {
                intakeSubsystem.turnIntakeOn();
                shooterSubsystem.spinUp();
                clock.moveClockToPreShootPosition();
                initState();
                if(scorePreload != null) { follower.followPath(scorePreload); }
                else { nextStage(); }
            }
            shooterSubsystem.updateSpin();

            if(!follower.isBusy())
            {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabFirstBalls.ordinal());
            }
        }

        if(pathState == AutoStage.grabFirstBalls.ordinal())
        {
            if(stageInit)
            {
                initState();
                intakeSubsystem.turnIntakeOn();
                if(grabPickup1 != null) { follower.followPath(grabPickup1, 0.9, false); }
                else { nextStage(); }

            }

            if(!follower.isBusy())
            {
                nextStage(AutoStage.hitGate.ordinal());
            }
        }

        if(pathState == AutoStage.hitGate.ordinal())
        {
            if(stageInit)
            {
                initState();
                if(hitGate != null) { follower.followPath(hitGate); }
                else { nextStage(); }
            }

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 2000)
            {
                nextStage(AutoStage.scoreFirstBalls.ordinal());
            }
        }

        if(pathState == AutoStage.scoreFirstBalls.ordinal())
        {
            if(stageInit)
            {
                shooterSubsystem.spinUp();

                if(scorePickup1 != null) { follower.followPath(scorePickup1); }
                else { nextStage(); }
                initState();
            }
            intakeSubsystem.turnIntakeOn();
            shooterSubsystem.updateSpin();

            if(!follower.isBusy())
            {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabSecondBalls.ordinal());
            }
        }

        if(pathState == AutoStage.grabSecondBalls.ordinal())
        {
            if(stageInit)
            {

                intakeSubsystem.turnIntakeOn();
                if(grabPickup2 != null) { follower.followPath(grabPickup2); }
                else { nextStage(); }
                initState();

            }

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 200)
            {
                nextStage(AutoStage.scoreSecondBalls.ordinal());
            }
        }
        if(pathState == AutoStage.scoreSecondBalls.ordinal())
        {
            if(stageInit)
            {
                initState();
                shooterSubsystem.spinUp();
                if(scorePickup2 != null) { follower.followPath(scorePickup2); }
                else { nextStage(); }

            }
            intakeSubsystem.turnIntakeOn();
            shooterSubsystem.updateSpin();


            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 50)
            {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabThirdBalls.ordinal());
            }
        }
        if(pathState == AutoStage.grabThirdBalls.ordinal())
        {
            if(stageInit)
            {
                if(grabPickup3 != null) {
                    intakeSubsystem.turnIntakeOn();
                    follower.followPath(grabPickup3);
                    initState();
                }
                else {
                    nextStage();
                }
            }

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 50) //keeps skipping this state idk why so we wanna make sure we don't jump
            {
                nextStage();
            }
        }

        //not enough time to shoot the third set of balls so we just grab them and park
//        if(pathState == AutoStage.scoreThirdBalls.ordinal())
//        {
//            if(stageInit)
//            {
//                shooterSubsystem.spinUp();
//                intakeSubsystem.turnIntakeOn();
//                if(scorePickup3 != null) { follower.followPath(scorePickup3); }
//                else { nextStage(); }
//                initState();
//            }
//            shooterSubsystem.updateSpin();
//
//
//            if(!follower.isBusy())
//            {
//
//                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.park.ordinal());
//            }
//
//        }

        if(pathState == AutoStage.parkGate.ordinal())
        {
            if(stageInit)
            {
                parkGate = getParkGate(follower);
                if(parkGate != null) { follower.followPath(parkGate); }
                else { nextStage(); }
                initState();
            }

            if(!follower.isBusy())
            {
                nextStage(AutoStage.endBehavior.ordinal());
            }
        }
        if(pathState == AutoStage.parkZone.ordinal())
        {
            if(stageInit)
            {
                parkZone = getParkZone(follower);
                if(parkZone != null) { follower.followPath(parkZone); }
                else { nextStage(); }
                initState();
            }


            if(!follower.isBusy())
            {
                nextStage(AutoStage.endBehavior.ordinal());
            }
        }

        if(pathState == AutoStage.shootPrep.ordinal())
        {
            if(stageInit)
            {
                shooterSubsystem.spinUp();
                initState();
            }
            shooterSubsystem.updateSpin();

            if(shooterSubsystem.isFlywheelReady && !follower.isBusy())
            {
                nextStage(AutoStage.shoot.ordinal());
            }
        }
        if(pathState == AutoStage.shoot.ordinal())
        {
            if (stageInit) {
                initState();
                shooterSubsystem.startShotSequence(isAuto);
            }


            double shootWaitTime = isAutoFar ? 5500 : 3500;

            if(stageStartTimer.getElapsedTime() > 3500)
            {
                shooterSubsystem.stopAutoShot();
                intakeSubsystem.turnIntakeOff();
                if(stageStartTimer.getElapsedTime() > shootWaitTime)
                {
                    nextStage(pathAfterStateShotOrdinal);
                }
            }

        }
        if(pathState == AutoStage.endBehavior.ordinal())
        {
            follower.breakFollowing();
            drive.hardStopMotors();
            turret.turnOffFlywheel();
            clock.resetClock();
            intakeSubsystem.turnIntakeOff();
            IndicatorLight.setLightRed();

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

        turret.resetPID();
        opmodeTimer.resetTimer();
        stageStartTimer.resetTimer();
        stageInit = false;
    }


}

