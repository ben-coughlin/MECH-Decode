package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;

public abstract class AutoMasterFar extends RobotMasterPinpoint {
    protected long startTime = 0;
    protected boolean past5In = false;

    protected enum progStates {
        SHOOT_PREP,
        SHOOT,
        driveOutsideOfShootZoneToEnd,
        endBehavior
    }
    protected abstract boolean isCorrectGoalTag(int tagId);
    protected abstract CurvePoint getDriveOutsideShootZoneEndPoint();
    protected abstract double getDriveOutsideShootZoneHeading();
    protected abstract int getDriveOutsideShootZoneFollowCurveTolerance();


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



        if (programStage == progStates.SHOOT_PREP.ordinal()) {
            handleShootPrep();
        }

        if (programStage == progStates.SHOOT.ordinal()) {
            handleShoot();
        }
        if(programStage == progStates.driveOutsideOfShootZoneToEnd.ordinal())
        {
            handleDriveOutsideOfShootZoneToEnd();
        }

        if (programStage == progStates.endBehavior.ordinal()) {
            handleEndBehavior();
        }
    }

    protected void handleShootPrep() {
        if (stageFinished) {
            initializeStateVariables();
            shooterSubsystem.spinUp();
        }
        shooterSubsystem.updateSpin();

        if (shooterSubsystem.isFlywheelReady) {
            clock.moveClockToPreShootPosition();
            ishouldremovetheselater(progStates.SHOOT.ordinal());
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
            ishouldremovetheselater(stageAfterShotOrdinal);
        }


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

    protected void handleEndBehavior() {
        if (stageFinished) {
            past5In = false;
            initializeStateVariables();
        }
        drive.stopAllMovementDirectionBased();
    }

}