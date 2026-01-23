package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

public class ShooterSubsystem {

    private final Clock clock;
    private final Turret turret;
    private final IntakeSubsystem intake;

    public boolean isFlywheelReady = false;
    public boolean isShotInProgress = false; //todo: change this back to private after debugging
    public boolean isFlywheelSpun = false;

    private long flywheelStartTime = 0;
    private long shotStartTime = 0;
    public int shotsRemaining = 0; //todo: change this back to private after debugging



    public ShooterSubsystem(Clock clock, Turret turret, IntakeSubsystem intake) {
        this.clock = clock;
        this.turret = turret;
        this.intake = intake;
    }

    public void stopShot() {
        runStopActions();
    }


    /**
     * Call ONCE to begin flywheel spin up
     */
    public void spinUp() {
        if (!isFlywheelReady) {
            turret.turnOnFlywheel();
            flywheelStartTime = SystemClock.uptimeMillis();
            Log.i("ShooterSubsystem", "SpinUp called at " + flywheelStartTime);
            isShotInProgress = true;
            intake.turnIntakeOn();
        }
    }

    /**
     * Call every loop until ready
     */
    public void updateSpin() {
        if ((!isFlywheelReady && SystemClock.uptimeMillis() - flywheelStartTime > 200) || isFlywheelSpun) {
            Log.i("ShooterSubsystem", "Spin update elapsed = " +
                    (SystemClock.uptimeMillis() - flywheelStartTime));
            isFlywheelReady = true;
            clock.setRampToShootPower();
            clock.moveClockToPreShootPosition();
            intake.turnIntakeOff();
        }
    }

    /**
     * Begin shooting sequence
     */
    public void startShotSequence() {
        if (!isFlywheelReady) return;
        shotStartTime = SystemClock.uptimeMillis();
        clock.moveClockToShootPosition();
        if ((SystemClock.uptimeMillis() - shotStartTime >= 2800)) {
            runStopActions();
            return;
        }

        Log.i("ShooterSubsystem", "shot seq started at " + shotStartTime);
    }

    public void runStopActions() {
        turret.turnOffFlywheel();
        clock.resetClock();
        clock.stopRamp();
        isFlywheelReady = false;
        isShotInProgress = false;
    }
}






