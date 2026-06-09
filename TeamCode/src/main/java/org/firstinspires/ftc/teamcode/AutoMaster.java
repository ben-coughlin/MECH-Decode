package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Collections;
import java.util.Set;


public abstract class AutoMaster extends RobotMaster {

    private static Follower follower;
    private Timer pathTimer, stageStartTimer, opmodeTimer;
    protected boolean stageInit = true;
    public static String selectedProgram;

    protected static boolean doZonePark = true;

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
        shootPrep,
        shoot,
        endBehavior,


    }

    public static int pathState = AutoStage.scorePreload.ordinal();
    private int pathAfterStateShotOrdinal = AutoStage.park.ordinal();
    protected abstract boolean isCorrectGoalTag(int tagId);
    protected abstract boolean isAutoFar();

    protected Set<Integer> getSkippedStages() {
        return Collections.emptySet();  // Default: skip nothing
    }

    //points
    protected abstract Pose getStartPose();
    protected abstract PathChain getScorePreload(Follower follower);
    protected abstract PathChain getGrabPickup1(Follower follower);
   // protected abstract PathChain getHitGate(Follower follower);
    protected abstract PathChain getScorePickup1(Follower follower);
    protected abstract PathChain getGrabPickup2(Follower follower);
    protected abstract PathChain getScorePickup2(Follower follower);
    protected abstract PathChain getGrabPickup3(Follower follower);
    protected abstract PathChain getScorePickup3(Follower follower);
    protected abstract PathChain getParkGate(Follower follower);
    protected abstract PathChain getParkZone(Follower follower);

    private PathChain scorePreload;
    private PathChain grabPickup1;
    private PathChain hitGate;
    private PathChain scorePickup1;
    private PathChain grabPickup2;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain parkZone;




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
        getSkippedStages(); //running this will set the park booleans and then we call it later to get stages

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(getStartPose());
        follower.update();
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
                false
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

        grabPickup1 = getGrabPickup1(follower);
        //hitGate = getHitGate(follower);

        scorePickup1 = getScorePickup1(follower);

        grabPickup2 = getGrabPickup2(follower);

        scorePickup2 = getScorePickup2(follower);

        grabPickup3 = getGrabPickup3(follower);

        scorePickup3 = getScorePickup3(follower);

        //we don't build these at init because we use lazy generation to get the robot pose at the time we need to park
        //parkGate = getParkGate(follower);
        //parkZone = getParkZone(follower);
    }

    public void updatePaths() {


        if(pathState == AutoStage.scorePreload.ordinal()){
            if(stageInit)
            {
                intake.turnIntakeOn();
                shooterSubsystem.spinUp();
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
                intake.turnIntakeOn();
                transfer.turnTransferOn();
                if(grabPickup1 != null) { follower.followPath(grabPickup1, true); }
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

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 1000)
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
            intake.turnIntakeOn();
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

                intake.turnIntakeOn();
                transfer.turnTransferOn();
                if(grabPickup2 != null) { follower.followPath(grabPickup2,   true); }
                else { nextStage(); }
                initState();

            }

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 300)
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
            intake.turnIntakeOn();
            shooterSubsystem.updateSpin();


            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 30)
            {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.grabThirdBalls.ordinal());
            }
        }
        if(pathState == AutoStage.grabThirdBalls.ordinal())
        {
            if(stageInit)
            {
                if(grabPickup3 != null) {
                    intake.turnIntakeOn();
                    transfer.turnTransferOn();
                    follower.followPath(grabPickup3,  true);
                    initState();
                }
                else {
                    nextStage();
                }
            }

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 30) //keeps skipping this state idk why so we wanna make sure we don't jump
            {
                nextStage(AutoStage.scoreThirdBalls.ordinal());
            }
        }

        if(pathState == AutoStage.scoreThirdBalls.ordinal())
        {
            if(stageInit)
            {
                shooterSubsystem.spinUp();
                intake.turnIntakeOn();
                if(scorePickup3 != null) { follower.followPath(scorePickup3); }
                else { nextStage(); }
                initState();
            }
            shooterSubsystem.updateSpin();


            if(!follower.isBusy())
            {

                nextStage(AutoStage.shootPrep.ordinal(), AutoStage.park.ordinal());
            }

        }

        if(pathState == AutoStage.park.ordinal())
        {
            if(stageInit)
            {
                PathChain parkGate = getParkGate(follower);
                PathChain parkZone = getParkZone(follower);

                if(parkGate != null && !doZonePark) { follower.followPath(parkGate); }
                else if(parkZone != null && doZonePark ) {follower.followPath(parkZone); }
                else { nextStage(); }
                initState();
            }

            if(!follower.isBusy() && stageStartTimer.getElapsedTime() > 30)
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

            if(ShooterSubsystem.isFlywheelReady && !follower.isBusy())
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



            if(stageStartTimer.getElapsedTime() > 1000)
            {
                shooterSubsystem.stopAutoShot();
                intake.turnIntakeOff();
                nextStage(pathAfterStateShotOrdinal);

            }

        }
        if(pathState == AutoStage.endBehavior.ordinal())
        {
           // follower.breakFollowing();
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

