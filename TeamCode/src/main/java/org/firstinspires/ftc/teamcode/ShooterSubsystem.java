package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

public class    ShooterSubsystem {

    private final IntakeSubsystem intakeSubsystem;
    private final Spindexer spindexer;
    private final Turret turret;

    private boolean isFlywheelReady = false;
    private boolean isShotInProgress = false;

    private long flywheelStartTime = 0;
    private long shotStartTime = 0;
    private int shotsRemaining = 0;
    private long shotInterval = 300; // ms between balls
    private long lastShotTime = 0;


    public ShooterSubsystem(IntakeSubsystem intakeSubsystem, Spindexer spindexer, Turret turret) {
        this.intakeSubsystem = intakeSubsystem;
        this.spindexer = spindexer;
        this.turret = turret;
    }

    /** Call ONCE to begin flywheel spin up */
    public void spinUp() {
        if (!isFlywheelReady) {
            turret.turnOnFlywheel();
            flywheelStartTime = SystemClock.uptimeMillis();
            Log.i("ShooterSubsystem", "SpinUp called at " + flywheelStartTime);

        }
    }

    /** Call every loop until ready */
    public void updateSpin() {
        if (!isFlywheelReady && SystemClock.uptimeMillis() - flywheelStartTime > 3000) {
            Log.i("ShooterSubsystem", "Spin update elapsed = " +
                    (SystemClock.uptimeMillis() - flywheelStartTime));

            isFlywheelReady = true;
        }
    }

    /** Begin shooting sequence */
    public void startShotSequence() {
        if (!isFlywheelReady || isShotInProgress) return;

        isShotInProgress = true;
        shotStartTime = SystemClock.uptimeMillis();
        Log.i("ShooterSubsystem", "shot seq started at " + shotStartTime);
    }
    public void startMultiShot(int numShots) {
        if (!isFlywheelReady || isShotInProgress) return;
        shotsRemaining = numShots;
        isShotInProgress = true;
        shotStartTime = SystemClock.uptimeMillis();
        lastShotTime = shotStartTime;
        Log.i("ShooterSubsystem", "Multi-shot sequence started: " + numShots + " shots");
    }


    /** Must be called EVERY LOOP */
    public void updateActiveShot() {
        if (!isShotInProgress) return;

        long now = SystemClock.uptimeMillis();
        long elapsed = now - lastShotTime;

        if (elapsed > 200) intakeSubsystem.moveKickerVertical();
        if (elapsed > 400) intakeSubsystem.moveKickerHorizontal();

        if (elapsed > 400 + shotInterval) {
            spindexer.recordShotBall(); // counts a ball
            shotsRemaining--;
            lastShotTime = now;

            if (shotsRemaining <= 0) {
                isShotInProgress = false; // sequence done
                Log.i("ShooterSubsystem", "Multi-shot sequence complete");
                turnOff();
            }
        }
    }


    public boolean isReadyToShoot() {
        return isFlywheelReady && !isShotInProgress;
    }
    public void turnOff()
    {
        turret.turnOffFlywheel();
    }

}
