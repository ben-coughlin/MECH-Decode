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


public abstract class AutoMaster extends RobotMasterPinpoint {

    private Follower follower;
    private Timer pathTimer, stageStartTimer, opmodeTimer;
    protected int pathStateEndOverride = -2; //use this if at any point we want to stop progressing - likely to work w someone
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
        shootPrep,
        shoot,
        endBehavior
    }
    public static int pathState = AutoStage.scorePreload.ordinal();

    protected abstract boolean isCorrectGoalTag(int tagId);
    //points
    protected abstract Pose getStartPose();
    protected abstract Pose getScorePose();
    protected abstract Pose getGrabFirstBallsPose();
    protected abstract Pose getFirstBallsControlPose();
    protected abstract Pose getHitGatePose();
    protected abstract Pose getHitGateControlPose();
    protected abstract Pose getGrabSecondBallsPose();
    protected abstract Pose getSecondBallsControlPose();
    protected abstract Pose getGrabThirdBallsPose();
    protected abstract Pose getThirdBallsControlPose();
    protected abstract Pose getParkPose();

    private Path scorePreload;
    private PathChain grabPickup1, hitGate, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, park;

    @Override
    public void loop() {
        super.mainLoop();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

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


//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);

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
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(getStartPose(), getScorePose()));
        scorePreload.setLinearHeadingInterpolation(getStartPose().getHeading(), getScorePose().getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(getScorePose(), getFirstBallsControlPose(), getGrabFirstBallsPose()))
                .setLinearHeadingInterpolation(getScorePose().getHeading(), getGrabFirstBallsPose().getHeading())
                .build();

        hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(getGrabFirstBallsPose(), getHitGateControlPose(), getHitGatePose()))
                .setLinearHeadingInterpolation(getGrabFirstBallsPose().getHeading(), getHitGatePose().getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a BezierCurve */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(getHitGatePose(), getScorePose()))
                .setLinearHeadingInterpolation(getHitGatePose().getHeading(), getScorePose().getHeading())
                .build();

        /* This is our grabPickup2 PathChain. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(getScorePose(), getSecondBallsControlPose(), getGrabSecondBallsPose()))
                .setLinearHeadingInterpolation(getScorePose().getHeading(), getGrabSecondBallsPose().getHeading())
                .build();

        /* This is our scorePickup2 PathChain.  */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(getGrabSecondBallsPose(), getScorePose()))
                .setLinearHeadingInterpolation(getGrabSecondBallsPose().getHeading(), getScorePose().getHeading())
                .build();

        /* This is our grabPickup3 PathChain. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(getScorePose(), getThirdBallsControlPose(), getGrabThirdBallsPose()))
                .setLinearHeadingInterpolation(getScorePose().getHeading(), getGrabThirdBallsPose().getHeading())
                .build();

        /* This is our scorePickup3 PathChain.  */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(getGrabThirdBallsPose(), getScorePose()))
                .setLinearHeadingInterpolation(getGrabThirdBallsPose().getHeading(), getScorePose().getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(getScorePose(), getParkPose()))
                .setLinearHeadingInterpolation(getScorePose().getHeading(), getParkPose().getHeading())
                .build();
    }

    public void updatePaths() {
        if(pathState == AutoStage.scorePreload.ordinal()){
            if(stageInit)
            {
                shooterSubsystem.spinUp();
                initState();
            }
            follower.followPath(scorePreload);
            shooterSubsystem.updateSpin();

            if(!follower.isBusy())
            {
                nextStage(AutoStage.shootPrep.ordinal());
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
            if(!shooterSubsystem.isShotInProgress || SystemClock.uptimeMillis() - stateStartTime > 3700)
            {
                clock.resetClock();
            }
            if(SystemClock.uptimeMillis() - stateStartTime > 4300)
            {
                ishouldremovetheselater(stageAfterShotOrdinal);
            }
        }
    }

    protected void nextStage() {

        //once we get to the state we define as the "end early" (if we do) then we end
        if(pathState == pathStateEndOverride)
        {
            pathState = -1;
        }
        else
        {
            pathState++;
        }
        pathTimer.resetTimer();
        stageInit = true;
    }
    protected void nextStage(int stage)
    {
        //once we get to the state we define as the "end early" (if we do) then we end
        if(pathState == pathStateEndOverride)
        {
            pathState = -1;
        }
        else
        {
            pathState = stage;
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
