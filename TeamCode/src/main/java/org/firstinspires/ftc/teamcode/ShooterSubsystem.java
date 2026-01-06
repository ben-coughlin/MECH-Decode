package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.os.SystemClock;
import android.util.Log;

public class ShooterSubsystem {

    private final IntakeSubsystem intakeSubsystem;
    private final Spindexer spindexer;
    private final Turret turret;
    private final ColorSensor colorSensor;

    public boolean isFlywheelReady = false;
    public boolean isShotInProgress = false; //todo: change this back to private after debugging
    public boolean isFlywheelSpun = false;

    private long flywheelStartTime = 0;
    private long shotStartTime = 0;
    public int shotsRemaining = 0; //todo: change this back to private after debugging
    private long shotInterval = 300; // ms between balls
    private long lastShotTime = 0;
    private boolean hasKickerGoneVertical = false;
    private boolean hasKickerGoneHorizontal = false;
    private boolean hasRecordedShotBall = false;
    public static boolean activeShotDisable = false;


    public ShooterSubsystem(IntakeSubsystem intakeSubsystem, Spindexer spindexer, Turret turret, ColorSensor colorSensor) {
        this.intakeSubsystem = intakeSubsystem;
        this.spindexer = spindexer;
        this.turret = turret;
        this.colorSensor = colorSensor;
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
        if ((!isFlywheelReady && SystemClock.uptimeMillis() - flywheelStartTime > 600) || isFlywheelSpun) {
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
        if(activeShotDisable) return;

        long now = SystemClock.uptimeMillis();
        long elapsed = now - lastShotTime;


        if ((spindexer.isAtTargetPosition() || elapsed >= 620)  && !hasKickerGoneVertical) {
           // intakeSubsystem.moveKickerVertical();
            hasKickerGoneVertical = true;
            Log.i("ShooterSubsystem", "move kicker vertical@"+elapsed);
        }

        if (elapsed >= 750 && !hasKickerGoneHorizontal) {
            //intakeSubsystem.moveKickerHorizontal();
            hasKickerGoneHorizontal = true;
            Log.i("ShooterSubsystem", "move kicker horizontal@"+elapsed);
        }



// Phase 1: detect shot
        if (!hasRecordedShotBall && elapsed >= 850) {
            if(!spindexer.recordShotBall(true))
            {
                lastShotTime = now;
                hasKickerGoneVertical = false;
                hasKickerGoneHorizontal = false;
                Log.i("Spindexer", "shot failed, resetting timers to try again" + elapsed);
            }
            else
            {
                hasRecordedShotBall = true;
            }
            Log.i("ShooterSubsystem","Shot detected@" + elapsed);
        }

// Phase 2: wait for spindexer to reach next position OR timeout
        if (hasRecordedShotBall && (spindexer.isAtTargetPosition() || elapsed >= 1650)) {
            shotsRemaining--;
            lastShotTime = now;
            hasKickerGoneVertical = false;
            hasKickerGoneHorizontal = false;
            hasRecordedShotBall = false;
            Log.i("ShooterSubsystem","Ready for next shot@" + elapsed);
        }


        if (shotsRemaining <= 0) {
            isShotInProgress = false;
            Log.i("ShooterSubsystem", "Multi-shot sequence complete");
        }
    }



    public boolean isReadyToShoot() {
        return isFlywheelReady && !isShotInProgress;
    }
    public void turnOff()
    {
        turret.turnOffFlywheel();
    }
    public void reset() {
        isShotInProgress = false;
        shotsRemaining = 0;
        hasKickerGoneVertical = false;
        hasKickerGoneHorizontal = false;
        hasRecordedShotBall = false;
        Log.i("ShooterSubsystem", "Shot sequence reset");
    }

}
