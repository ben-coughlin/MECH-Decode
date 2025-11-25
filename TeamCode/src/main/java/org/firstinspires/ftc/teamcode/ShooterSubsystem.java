package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {

    public enum ShootState {
        IDLE,
        SPINUP,
        READY_TO_FIRE,
        FIRE_BALL,
        WAIT_BETWEEN_SHOTS
    }

    private ShootState currentState = ShootState.IDLE;
    private long stateStartTime = 0;

    private int shotsRequested = 1;
    private int shotsCompleted = 0;
    private boolean isAutoShooting = false;
    private boolean hasManualApproval = false;

    private final Turret turret;
    private final IntakeSubsystem intake;
    private final Spindexer spindexer;

    public ShooterSubsystem(Turret turret, IntakeSubsystem intake, Spindexer spindexer) {
        this.turret = turret;
        this.intake = intake;
        this.spindexer = spindexer;
    }

    // Spin up flywheel manually
    public void spinUp() {
        if (currentState == ShootState.IDLE) {
            currentState = ShootState.SPINUP;
            stateStartTime = SystemClock.uptimeMillis();
            turret.turnOnFlywheel();
        }
    }

    // Fire a single shot manually
    public void fireManual() {
       hasManualApproval = true;
    }

    // Start an automatic multi-shot sequence
    public void startAuto(int count) {
        currentState = ShootState.SPINUP;
        stateStartTime = SystemClock.uptimeMillis();
        turret.turnOnFlywheel();
        shotsRequested = count;
        shotsCompleted = 0;
        isAutoShooting = true;
    }

    // Must be called every loop
    public boolean update(Gamepad gamepad1, Gamepad gamepad2) {
        long elapsed = SystemClock.uptimeMillis() - stateStartTime;

        switch (currentState) {

            case SPINUP:
                if (elapsed >= 4000) { // wait for flywheel spinup
                    if (gamepad1 != null) gamepad1.rumble(1, 1, 200);
                    if (gamepad2 != null) gamepad2.rumble(1, 1, 200);
                    currentState = ShootState.READY_TO_FIRE;
                    stateStartTime = SystemClock.uptimeMillis();
                    hasManualApproval = false;
                }
                break;

            case READY_TO_FIRE:
                if(hasManualApproval || isAutoShooting)
                {
                    currentState = ShootState.FIRE_BALL;
                    stateStartTime = SystemClock.uptimeMillis();
                }
                break;

            case FIRE_BALL:
                if (elapsed >= 200) intake.moveKickerVertical();
                if (elapsed >= 500) intake.moveKickerHorizontal();
                if (elapsed >= 1000) {
                    spindexer.recordShotBall();
                    shotsCompleted++;

                    if (shotsCompleted < shotsRequested && isAutoShooting) {
                        currentState = ShootState.WAIT_BETWEEN_SHOTS;
                        stateStartTime = SystemClock.uptimeMillis();
                    } else {
                        currentState = ShootState.IDLE;
                        turret.turnOffFlywheel();
                        return true; // finished all shots
                    }
                }
                break;

            case WAIT_BETWEEN_SHOTS:
                if (elapsed >= 400) {
                    intake.moveKickerHorizontal(); // reset kicker
                    currentState = ShootState.FIRE_BALL;
                    stateStartTime = SystemClock.uptimeMillis();
                }
                break;

            case IDLE:
            default:
                break;
        }

        return false;
    }

    public ShootState getState() {
        return currentState;
    }

    public void showShooterTelemetry(Telemetry telemetry)
    {
        telemetry.addLine(" --- Shooter ---");
        telemetry.addData("State", getState());
    }
}
