package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class AutoMaster extends RobotMasterPinpoint {

    protected final double SCALE_FACTOR = 0.9;
    protected long startTime = 0;
    protected int cycle = 0;
    protected int driveToGetSampleCycle = 0;
    protected boolean hasFinishedManualAim = false;
    protected boolean isLookingAtObelisk = true;
    protected boolean past5In = false;
    protected double cutOffTime = 22.5;
    protected String currentState;
    public int overallCycleToChamber = 0;

    protected static final String SIMULATOR_HOST = "192.168.43.22";
    protected static final int SIMULATOR_PORT = 7777;
    public static boolean pickupOffWall = false;

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

    protected abstract boolean isCorrectGoalTag(int tagId);
    protected abstract double getManualAimSpeed();

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

    // Legacy from Center Stage season
    protected HashMap<Integer, PointDouble> purpleDrop = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(105, 26));
        put(1, new PointDouble(103.24, 35));
        put(2, new PointDouble(109.62, 36.8));
    }};

    @Override
    public void init() {
        super.init();
        spindexer.setInventory(new Pattern(Pattern.Ball.GREEN, Pattern.Ball.PURPLE, Pattern.Ball.PURPLE));
        isAuto = true;
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

        // Limelight targeting logic
        if (!isLookingAtObelisk) {
            if (Limelight.getCurrResult() != null && Limelight.getCurrResult().isValid()
                    && isCorrectGoalTag(VisionUtils.getTagId(Limelight.getCurrResult()))) {
                double llError = Limelight.getCurrResult().getTx();
                turret.aimTurret(true, llError, gamepad2.right_stick_x, limelight.getDistanceToTag(Limelight.getCurrResult()));
                hasFinishedManualAim = true;
            }
        } else {
            if (!hasFinishedManualAim) {
                turret.aimTurret(false, 0, getManualAimSpeed(), 999);
            }

            if (Limelight.getCurrResult() != null && Limelight.getCurrResult().isValid()
                    && isCorrectGoalTag(VisionUtils.getTagId(Limelight.getCurrResult()))) {
                turret.aimTurret(false, 0, 0, 0);
                turret.resetEncoder();
                isLookingAtObelisk = false;
                return;
            }
        }

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

        if (programStage == progStates.driveToShootPointToEnd.ordinal()) {
            handleDriveToShootPointToEnd();
        }

        if (programStage == progStates.driveOutsideOfShootZoneToEnd.ordinal()) {
            handleDriveOutsideOfShootZoneToEnd();
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
    }

    protected void handleDriveBackwardsFromStartToShootPreload() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
            shooterSubsystem.spinUp();
        }
        shooterSubsystem.updateSpin();
        shooterSubsystem.updateActiveShot();

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
            spindexer.startIntakeCycle();
        }
        spindexer.intakeNewBall();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getIntakeFirstThreeBallsEndPoint());

        if (Movement.followCurve(points, getIntakeFirstThreeBallsHeading(), getIntakeFirstThreeBallsFollowCurveTolerance())) {
            drive.stopAllMovementDirectionBased();
            shooterSubsystem.isFlywheelSpun = true;
            nextStage(progStates.driveToShootingPoint.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleDriveToShootingPoint() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
            shooterSubsystem.isFlywheelReady = false;
            shooterSubsystem.spinUp();
            intakeSubsystem.turnIntakeOff();
        }
        shooterSubsystem.updateSpin();
        shooterSubsystem.updateActiveShot();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getShootingPointEndPoint());

        if (Movement.followCurve(points, getShootingPointHeading(), getShootingPointFollowCurveTolerance())) {
            intakeSubsystem.turnIntakeOff();
            drive.stopAllMovementDirectionBased();
            nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveToSecondThreeBalls.ordinal());
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
            onDriveToSecondThreeBallsComplete();
        }
        drive.applyMovementDirectionBased();
    }

    // Hook method for alliance-specific behavior after second three balls
    protected void onDriveToSecondThreeBallsComplete() {
        spindexer.rotateToNextSlotInPattern();
        nextStage(progStates.endBehavior.ordinal());
    }

    protected void handleIntakeSecondThreeBalls() {
        if (stageFinished) {
            past5In = false;
            intakeSubsystem.turnIntakeOn();
            spindexer.startIntakeCycle();
            shooterSubsystem.spinUp();
            Log.i("DEBUG", "intakeSecondThreeBalls");
        }

        spindexer.intakeNewBall();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getIntakeSecondThreeBallsEndPoint());

        if (Movement.followCurve(points, getIntakeSecondThreeBallsHeading(), getIntakeSecondThreeBallsTolerance())) {
            drive.stopAllMovementDirectionBased();
            onIntakeSecondThreeBallsComplete();
        }
        drive.applyMovementDirectionBased();
    }

    // Hook method for alliance-specific behavior after intake second three balls
    protected void onIntakeSecondThreeBallsComplete() {
        if ((startTime - SystemClock.uptimeMillis()) * 1000 > 26) {
            nextStage(progStates.endBehavior.ordinal());
        } else {
            nextStage(progStates.driveToShootPointToEnd.ordinal());
        }
    }

    protected void handleDriveToShootPointToEnd() {
        if (stageFinished) {
            past5In = false;
            spindexer.intakeCycleActive = false;
            initializeStateVariables();
            shooterSubsystem.isFlywheelReady = false;
            shooterSubsystem.spinUp();
            intakeSubsystem.turnIntakeOff();
        }
        shooterSubsystem.updateSpin();
        shooterSubsystem.updateActiveShot();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(getShootPointToEndEndPoint());

        if (Movement.followCurve(points, getShootPointToEndHeading(), getShootPointToEndFollowCurveTolerance())) {
            intakeSubsystem.turnIntakeOff();
            drive.stopAllMovementDirectionBased();
            shooterSubsystem.isFlywheelSpun = true;
            nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveOutsideOfShootZoneToEnd.ordinal());
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
            nextStage(progStates.endBehavior.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleIntakeThirdThreeBalls() {
        if (stageFinished) {
            past5In = false;
            intakeSubsystem.turnIntakeOn();
            spindexer.startIntakeCycle();
            Log.i("DEBUG", "intakeThirdThreeBalls");
        }

        spindexer.intakeNewBall();

        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY, 0, 0, 0, 0, 0, 0));
        points.add(new CurvePoint(-10, 71, 0.14 * SCALE_FACTOR, 0 * SCALE_FACTOR, 15, 15, Math.toRadians(60), 0.5));

        if (Movement.followCurve(points, Math.toRadians(90), 1)) {
            drive.stopAllMovementDirectionBased();
            spindexer.intakeCycleActive = false;
            shooterSubsystem.reset();
            nextStage(progStates.endBehavior.ordinal());
        }
        drive.applyMovementDirectionBased();
    }

    protected void handleShootPrep() {
        if (stageFinished) {
            initializeStateVariables();
            shooterSubsystem.spinUp();
            spindexer.rotateToNextSlotInPattern();
        }
        shooterSubsystem.updateSpin();
        shooterSubsystem.updateActiveShot();

        if (shooterSubsystem.isReadyToShoot()) {
            nextStage(progStates.SHOOT.ordinal());
        }
    }

    protected void handleShoot() {
        shooterSubsystem.updateSpin();
        shooterSubsystem.updateActiveShot();

        if (stageFinished) {
            stageFinished = false;
            initializeStateVariables();
            shooterSubsystem.startMultiShot(3);
            Log.i("ShooterSubsystem", "SHOOTING 3");
        }

        if (shooterSubsystem.isReadyToShoot()) {
            Log.i("DEBUG 1", "stageAfterShotOrdinal = " + stageAfterShotOrdinal);
            Log.i("DEBUG 1", "progressing to " + progStates.values()[stageAfterShotOrdinal]);
            nextStage(stageAfterShotOrdinal);
        }
    }

    protected void handleEndBehavior() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
        }
        shooterSubsystem.turnOff();
        drive.stopAllMovementDirectionBased();
    }
}