package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class AutoMaster extends RobotMasterPinpoint {

    protected final double SCALE_FACTOR = 0.9;
    protected long startTime = 0;

    protected boolean hasFinishedManualAim = false;
    protected boolean isLookingAtObelisk = true;
    protected boolean past5In = false;


    protected static final String SIMULATOR_HOST = "192.168.43.22";
    protected static final int SIMULATOR_PORT = 7777;
    private boolean hasClockStartedResetting = false;



    protected enum progStates {
        driveBackwardsFromStartToShootPreload,
        driveToFirstThreeBalls,
        intakeFirstThreeBalls,
        driveToShootingPoint,
        driveToSecondThreeBalls,
        intakeSecondThreeBalls,
        driveToShootPointToEnd,
        driveOutsideOfShootZoneToEnd,
        driveToThirdThreeBalls,
        intakeThirdThreeBalls,
        SHOOT_PREP,
        SHOOT,
        endBehavior
    }
    private int stageAfterShootPointOrdinal = progStates.driveToFirstThreeBalls.ordinal();

    protected abstract boolean isCorrectGoalTag(int tagId);


    // Override these for alliance-specific coordinates and headings
    protected abstract CurvePoint getShootPreloadEndPoint();
    protected abstract double getShootPreloadHeading();
    protected abstract int getShootPreloadFollowCurveTolerance();

    protected abstract CurvePoint getFirstThreeBallsEndPoint();
    protected abstract double getFirstThreeBallsHeading();
    protected abstract int getFirstThreeBallsFollowCurveTolerance();

    protected abstract CurvePoint getIntakeFirstThreeBallsEndPoint();
    protected abstract double getIntakeFirstThreeBallsHeading();
    protected abstract int getIntakeFirstThreeBallsFollowCurveTolerance();

    protected abstract CurvePoint getShootingPointEndPoint();
    protected abstract double getShootingPointHeading();
    protected abstract int getShootingPointFollowCurveTolerance();

    protected abstract CurvePoint getSecondThreeBallsEndPoint();
    protected abstract double getSecondThreeBallsHeading();
    protected abstract int getSecondThreeBallsFollowCurveTolerance();

    protected abstract CurvePoint getIntakeSecondThreeBallsEndPoint();
    protected abstract double getIntakeSecondThreeBallsHeading();
    protected abstract int getIntakeSecondThreeBallsTolerance();

    protected abstract CurvePoint getShootPointToEndEndPoint();
    protected abstract double getShootPointToEndHeading();
    protected abstract int getShootPointToEndFollowCurveTolerance();
    protected abstract CurvePoint getDriveOutsideShootZoneEndPoint();
    protected abstract double getDriveOutsideShootZoneHeading();
    protected abstract int getDriveOutsideShootZoneFollowCurveTolerance();
    protected abstract CurvePoint getIntakeThirdThreeBallsPoint();
    protected abstract double getIntakeThirdThreeBallsHeading();
    protected abstract CurvePoint getDriveToThirdThreeBallsPoint();
    protected abstract double getDriveToThirdThreeBallsHeading();



    @Override
    public void init() {
        super.init();
        //spindexer.setInventory(new Pattern(Pattern.Ball.GREEN, Pattern.Ball.PURPLE, Pattern.Ball.PURPLE)); //todo: we might  need a motif method, check later
        isAuto = true;
        clock.initClock();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        startTime = SystemClock.uptimeMillis();
    }

    @Override
    public void mainLoop() {
        super.mainLoop();
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

        Log.i("DEBUG", "Current stage: " + programStage + " = " + progStates.values()[programStage].name());
        Log.i("DEBUG", "=== LOOP START === Stage: " + programStage + " = " + progStates.values()[programStage].name());
        Log.i("DEBUG", "isShotInProgress: " + shooterSubsystem.isShotInProgress + ", shotsRemaining: " + shooterSubsystem.shotsRemaining);

        // State machine
        if (programStage == progStates.driveBackwardsFromStartToShootPreload.ordinal()) {
            handleDriveBackwardsFromStartToShootPreload();
        }

        if (programStage == progStates.driveToFirstThreeBalls.ordinal()) {
            handleDriveToFirstThreeBalls();
        }

        if (programStage == progStates.intakeFirstThreeBalls.ordinal()) {
            handleIntakeFirstThreeBalls();
        }

        if (programStage == progStates.driveToShootingPoint.ordinal()) {
            handleDriveToShootingPoint();
        }

        if (programStage == progStates.driveToSecondThreeBalls.ordinal()) {
            handleDriveToSecondThreeBalls();
        }

        if (programStage == progStates.intakeSecondThreeBalls.ordinal()) {
            handleIntakeSecondThreeBalls();
        }


        if(programStage == progStates.driveToThirdThreeBalls.ordinal())
        {
            handleDriveToThirdThreeBalls();
        }

        if (programStage == progStates.intakeThirdThreeBalls.ordinal()) {
            handleIntakeThirdThreeBalls();
        }


        if (programStage == progStates.SHOOT_PREP.ordinal()) {
            handleShootPrep();
        }

        if (programStage == progStates.SHOOT.ordinal()) {
            handleShoot();
        }

        if (programStage == progStates.endBehavior.ordinal()) {
            handleEndBehavior();
        }
        if(programStage == progStates.driveOutsideOfShootZoneToEnd.ordinal())
        {
            handleDriveOutsideOfShootZoneToEnd();
        }
    }

    protected void handleDriveBackwardsFromStartToShootPreload() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
            shooterSubsystem.spinUp();
        }
        shooterSubsystem.updateSpin();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getShootPreloadEndPoint());

        if (Movement.followCurve(points, getShootPreloadHeading(), getShootPreloadFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();
            nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveToFirstThreeBalls.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleDriveToFirstThreeBalls() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getFirstThreeBallsEndPoint());

        if (Movement.followCurve(points, getFirstThreeBallsHeading(), getFirstThreeBallsFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();
            nextStage(progStates.intakeFirstThreeBalls.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleIntakeFirstThreeBalls() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
            intakeSubsystem.turnIntakeOn();
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getIntakeFirstThreeBallsEndPoint());

        if (Movement.followCurve(points, getIntakeFirstThreeBallsHeading(), getIntakeFirstThreeBallsFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();
            shooterSubsystem.isFlywheelSpun = true;
            stageAfterShootPointOrdinal = getStageAfterShootPointOrdinal(progStates.intakeFirstThreeBalls.ordinal());
            nextStage(progStates.driveToShootingPoint.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleDriveToShootingPoint() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
            shooterSubsystem.spinUp();

        }
        shooterSubsystem.updateSpin();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getShootingPointEndPoint());

        if (Movement.followCurve(points, getShootingPointHeading(), getShootingPointFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();
            nextStage(progStates.SHOOT_PREP.ordinal(), stageAfterShootPointOrdinal);
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleDriveToSecondThreeBalls() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getSecondThreeBallsEndPoint());

        if (Movement.followCurve(points, getSecondThreeBallsHeading(), getSecondThreeBallsFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();
            nextStage(progStates.intakeSecondThreeBalls.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleIntakeSecondThreeBalls() {
        if (stageFinished) {
            past5In = false;
            intakeSubsystem.turnIntakeOn();
            shooterSubsystem.spinUp();
            Log.i("DEBUG", "intakeSecondThreeBalls");
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getIntakeSecondThreeBallsEndPoint());


        if (Movement.followCurve(points, getIntakeSecondThreeBallsHeading(), getIntakeSecondThreeBallsTolerance())) {
            drive.stopAllMovementDirectionBased();
            stageAfterShootPointOrdinal = getStageAfterShootPointOrdinal(progStates.intakeSecondThreeBalls.ordinal());
            nextStage(progStates.driveToShootingPoint.ordinal());

        }
        drive.applyMovementDirectionBased();
    }

    protected void handleDriveToThirdThreeBalls()
    {
        if(stageFinished)
        {
            past5In = false;
            initializeStateVariables();
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getDriveToThirdThreeBallsPoint());

        if (Movement.followCurve(points, getDriveToThirdThreeBallsHeading(), 2)) {
            drive.stopAllMovementDirectionBased();
            nextStage(progStates.intakeThirdThreeBalls.ordinal());

        }
        drive.applyMovementDirectionBased();

    }

    protected void handleIntakeThirdThreeBalls() {
        if (stageFinished) {
            past5In = false;
            intakeSubsystem.turnIntakeOn();
            Log.i("DEBUG", "intakeThirdThreeBalls");
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getIntakeThirdThreeBallsPoint());

        if (Movement.followCurve(points, getIntakeThirdThreeBallsHeading(), 3)) {
            drive.stopAllMovementDirectionBased();
            stageAfterShootPointOrdinal = getStageAfterShootPointOrdinal(progStates.intakeThirdThreeBalls.ordinal());
            nextStage(progStates.driveToShootingPoint.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleDriveOutsideOfShootZoneToEnd() {
        if (stageFinished) {
            past5In = false;
        }

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getDriveOutsideShootZoneEndPoint());

        if (Movement.followCurve(points, getDriveOutsideShootZoneHeading(), getDriveOutsideShootZoneFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();

        }
        drive.applyMovementDirectionBased();
    }

//    protected void handleDriveToShootPointToEnd() {
//        if (stageFinished) {
//            past5In = false;
//            initializeStateVariables();
//            shooterSubsystem.isFlywheelReady = false;
//            shooterSubsystem.spinUp();
//            intakeSubsystem.turnIntakeOff();
//        }
//        shooterSubsystem.updateSpin();
//
//        ArrayList<CurvePoint> points = new ArrayList<>();
//        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
//        points.add(getShootPointToEndEndPoint());
//
//        if (Movement.followCurve(points, getShootPointToEndHeading(), getShootPointToEndFollowCurveTolerance())) {
//            intakeSubsystem.turnIntakeOff();
//            drive.stopAllMovementDirectionBased();
//            shooterSubsystem.isFlywheelSpun = true;
//            nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveOutsideOfShootZoneToEnd.ordinal());
//        }
//        drive.applyMovementDirectionBased();
//    }

    protected void handleShootPrep() {
        if (stageFinished) {
            initializeStateVariables();
            shooterSubsystem.spinUp();
        }
        shooterSubsystem.updateSpin();

        if (shooterSubsystem.isFlywheelReady) {
            clock.moveClockToPreShootPosition();
            nextStage(progStates.SHOOT.ordinal());
        }
    }

    protected void handleShoot() {
        if (stageFinished) {
            stageFinished = false;
            initializeStateVariables();
            intakeSubsystem.turnIntakeOff();
        }
        shooterSubsystem.startShotSequence();
        if(!shooterSubsystem.isShotInProgress || SystemClock.uptimeMillis() - stateStartTime > 4000)
        {
            clock.resetClock();
        }
        if(SystemClock.uptimeMillis() - stateStartTime > 4800)
        {
            nextStage(stageAfterShotOrdinal);
        }



    }

    protected void handleEndBehavior() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
        }
        drive.stopAllMovementDirectionBased();
    }

    /**
     * determines what state we go to after the shoot point since we reuse that state
     * @return int ordinal of the state to go to after we're done shooting
     */
    private int getStageAfterShootPointOrdinal(int ordinal)
    {
       if(ordinal == progStates.intakeFirstThreeBalls.ordinal())
       {
           return progStates.driveToSecondThreeBalls.ordinal();
       }
       else if(ordinal == progStates.intakeSecondThreeBalls.ordinal())
       {
           return progStates.driveOutsideOfShootZoneToEnd.ordinal();
       }
       else if(ordinal == progStates.intakeThirdThreeBalls.ordinal())
       {
           return progStates.driveOutsideOfShootZoneToEnd.ordinal();
       }
       else
       {
           //this shouldn't happen
           return progStates.endBehavior.ordinal();
       }
    }

}