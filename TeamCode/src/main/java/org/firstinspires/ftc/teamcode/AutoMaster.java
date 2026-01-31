package org.firstinspires.ftc.teamcode;



import android.os.SystemClock;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Collections;
import java.util.Set;


public abstract class AutoMaster extends RobotMasterPinpoint {

    private Follower follower;
    private Timer pathTimer, stageStartTimer, opmodeTimer;
    protected boolean stageInit;
    protected enum AutoStage
    {
        scorePreload,
        grabFirstBalls,
        hitGate,
        scoreFirstBalls,
        grabSecondBalls,
        scoreSecondBalls,
        grabThirdBalls,
        scoreThirdBalls,
        park,
        endBehavior,
        shootPrep,
        shoot,

    }

    public static int pathState = AutoStage.scorePreload.ordinal();
    private int pathAfterStateShotOrdinal = -1;
    protected abstract int getStateAfterHitGate();
    protected abstract boolean isCorrectGoalTag(int tagId);

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
    protected abstract PathChain getScorePickup3(Follower follower);
    protected abstract PathChain getPark(Follower follower);


    private PathChain scorePreload, grabPickup1, hitGate, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, park;


    @Override
    public void loop() {
        super.mainLoop();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        updatePaths();

        Pose2D pos = odo.pos; // try not to flood odo with reads
        LLResult currVision = Limelight.getCurrResult();
        boolean hasValidVision = currVision != null
                && currVision.isValid()
                && isCorrectGoalTag(VisionUtils.getTagId(currVision));


        // ALWAYS call aimTurret - it will use odometry if vision is lost
        turret.aimTurret(
                hasValidVision,
                hasValidVision ? Limelight.getCurrResult().getTx() : 0,
                gamepad2.right_stick_x,
                Limelight.getDistance(),
                pos.getHeading(AngleUnit.DEGREES),
                odo.getVelocityComponents()[2]
        );



    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(getStartPose());

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
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

        scorePickup3 = getScorePickup3(follower);

        park = getPark(follower);
    }

    public void updatePaths() {
        if(pathState == AutoStage.scorePreload.ordinal()){
            if(stageInit)
            {
                shooterSubsystem.spinUp();
                initState();
            }
            shooterSubsystem.updateSpin();
            if(scorePreload != null) { follower.followPath(scorePreload); }
            else { nextStage(); }

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
            }
            if(grabPickup1 != null) { follower.followPath(grabPickup1); }
            else { nextStage(); }

            if(!follower.isBusy())
            {
                intakeSubsystem.turnIntakeOff();
                nextStage(AutoStage.hitGate.ordinal());
            }
        }

        if(pathState == AutoStage.hitGate.ordinal())
        {
            if(stageInit)
            {
                initState();
            }
            if(grabPickup2 != null) { follower.followPath(hitGate); }
            else { nextStage(); }

            if(!follower.isBusy())
            {
                nextStage(AutoStage.scoreFirstBalls.ordinal());
            }
        }

         if(pathState == AutoStage.scoreFirstBalls.ordinal())
         {
             if(stageInit)
             {
                 shooterSubsystem.spinUp();
                 initState();
             }
             shooterSubsystem.updateSpin();
             if(scorePickup1 != null) { follower.followPath(scorePickup1); }
             else { nextStage(); }

             if(!follower.isBusy()) {
                 // Check if we should hit gate after first score
                 if(getStateAfterHitGate() == AutoStage.scoreFirstBalls.ordinal()) {
                     nextStage(AutoStage.hitGate.ordinal());
                 } else {
                     nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabSecondBalls.ordinal());
                 }
             }
         }

         if(pathState == AutoStage.grabSecondBalls.ordinal())
         {
             if(stageInit)
             {
                 intakeSubsystem.turnIntakeOn();
                 initState();
             }
             shooterSubsystem.updateSpin();
             if(grabPickup2 != null) { follower.followPath(grabPickup2); }
             else { nextStage(); }

             if(!follower.isBusy())
             {
                 intakeSubsystem.turnIntakeOff();
                 nextStage(AutoStage.scoreSecondBalls.ordinal());

             }

             if(pathState == AutoStage.scoreSecondBalls.ordinal())
             {
                 if(stageInit)
                 {
                     shooterSubsystem.spinUp();
                     initState();
                 }
                 shooterSubsystem.updateSpin();
                 if(scorePickup2 != null) { follower.followPath(scorePickup2); }
                 else { nextStage(); }

                 if(!follower.isBusy()) {
                     // Check if we should hit gate after second score
                     if(getStateAfterHitGate() == AutoStage.scoreSecondBalls.ordinal()) {
                         nextStage(AutoStage.hitGate.ordinal());
                     } else {
                         nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabThirdBalls.ordinal());
                     }
                 }
             }
             if(pathState == AutoStage.grabThirdBalls.ordinal())
             {
                 if(stageInit)
                 {
                     intakeSubsystem.turnIntakeOn();
                     initState();
                 }
                 if(grabPickup3 != null) { follower.followPath(grabPickup3); }
                 else { nextStage(); }

                 if(!follower.isBusy())
                 {
                     intakeSubsystem.turnIntakeOff();
                     nextStage(AutoStage.scoreThirdBalls.ordinal());

                 }
             }

             if(pathState == AutoStage.scoreThirdBalls.ordinal())
             {
                 if(stageInit)
                 {
                     shooterSubsystem.spinUp();
                     initState();
                 }
                 shooterSubsystem.updateSpin();
                 if(scorePickup3 != null) { follower.followPath(scorePickup3); }
                 else { nextStage(); }

                 if(!follower.isBusy())
                 {
                     nextStage(AutoStage.shootPrep.ordinal(), AutoStage.park.ordinal());
                 }

             }

             if(pathState == AutoStage.park.ordinal())
             {
                 if(stageInit)
                 {
                     initState();
                 }
                 if(park != null) { follower.followPath(park); }
                 else { nextStage(); }

                 if(!follower.isBusy())
                 {
                     nextStage(AutoStage.endBehavior.ordinal());
                 }
             }

         }

        if(pathState == AutoStage.shootPrep.ordinal())
        {
            if(stageInit)
            {
                shooterSubsystem.spinUp();
                clock.moveClockToPreShootPosition();
                initState();
            }
            shooterSubsystem.updateSpin();

            if(shooterSubsystem.isFlywheelReady)
            {
                nextStage(AutoStage.shoot.ordinal());
            }
        }
        if(pathState == AutoStage.shoot.ordinal())
        {
            if (stageInit) {
                intakeSubsystem.turnIntakeOff();
                initState();
            }
            shooterSubsystem.startShotSequence();

            if(!shooterSubsystem.isShotInProgress)
            {
                nextStage(pathAfterStateShotOrdinal);
            }
        }
        if(pathState == AutoStage.endBehavior.ordinal())
        {
            drive.stopAllMovementDirectionBased();
            turret.turnOffFlywheel();
            clock.resetClock();
            intakeSubsystem.turnIntakeOff();
            IndicatorLight.setLightWhite();

            if(!follower.isBusy()){nextStage(-1);}

        }

    }

    protected void nextStage() {
        pathState++;

        // Skip stages if they're in the skip set
        while (getSkippedStages().contains(pathState)) {
            pathState++;
        }

        pathTimer.resetTimer();
        stageInit = true;
    }

    protected void nextStage(int stage) {
        pathState = stage;

        // Skip stages if they're in the skip set
        while (getSkippedStages().contains(pathState)) {
            pathState++;
        }

        pathTimer.resetTimer();
        stageInit = true;
    }

    protected void nextStage(int stage, int afterShotOrdinal) {
        pathAfterStateShotOrdinal = afterShotOrdinal;
        pathState = stage;

        // Skip stages if they're in the skip set
        while (getSkippedStages().contains(pathState)) {
            pathState++;
        }

        pathTimer.resetTimer();
        stageInit = true;
    }

    protected void initState()
    {
        turret.resetPID();
        opmodeTimer.resetTimer();


        stageInit = false;
    }



}
