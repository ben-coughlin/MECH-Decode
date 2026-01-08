package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.os.SystemClock;
import android.util.Log;

public class ShooterSubsystem {

    private final IntakeSubsystem intakeSubsystem;
    private final Clock clock;
    private final Turret turret;
    private final ColorSensor colorSensor;

    public boolean isFlywheelReady = false;
    public boolean isShotInProgress = false; //todo: change this back to private after debugging
    public boolean isFlywheelSpun = false;

    private long flywheelStartTime = 0;
    private long shotStartTime = 0;
    public int shotsRemaining = 0; //todo: change this back to private after debugging



    public ShooterSubsystem(IntakeSubsystem intakeSubsystem, Clock clock, Turret turret, ColorSensor colorSensor) {
        this.intakeSubsystem = intakeSubsystem;
        this.clock = clock;
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
        if (!isFlywheelReady) return;
        clock.rampShoot();
        shotStartTime = SystemClock.uptimeMillis();

        if (SystemClock.uptimeMillis() - shotStartTime >= 600)
        {
            clock.clockShoot();
        }
        if (SystemClock.uptimeMillis() - shotStartTime >= 1200)
        {
            turret.turnOffFlywheel();
            clock.clockInit();
            clock.rampInit();
            isFlywheelReady = false;
        }

        Log.i("ShooterSubsystem", "shot seq started at " + shotStartTime);
    }





}
