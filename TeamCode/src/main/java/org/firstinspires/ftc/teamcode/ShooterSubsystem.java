package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

public class ShooterSubsystem {

    private final Clock clock;
    private final Turret turret;
    private final IntakeSubsystem intake;

    public boolean isFlywheelReady = false;
    public boolean isShotInProgress = false;
    public boolean isFlywheelSpun = false;

    private long flywheelStartTime = 0;
    private long shotStartTime = 0;




    public ShooterSubsystem(Clock clock, Turret turret, IntakeSubsystem intake) {
        this.clock = clock;
        this.turret = turret;
        this.intake = intake;
    }



    /**
     * Call ONCE to begin flywheel spin up
     */
    public void spinUp() {
        if (!isFlywheelReady) {
            turret.turnOnFlywheel();
            flywheelStartTime = SystemClock.uptimeMillis();
            Log.i("ShooterSubsystem", "SpinUp called at " + flywheelStartTime);

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
    public void startShotSequence(boolean isAuto) {
        if (!isFlywheelReady) return;

        // Only initialize on first call
        if (!isShotInProgress) {
            isShotInProgress = true;
            shotStartTime = SystemClock.uptimeMillis();
            clock.moveClockToShootPosition();
            Log.i("ShooterSubsystem", "shot seq started at " + shotStartTime);
        }



    }

    // In TeleOp, call this when driver presses button:
    public void stopShot() {
        clock.resetClock();
        TeleOpMaster.hasClockReset = true;
        runStopActions();
    }


    public void runStopActions() {
        turret.turnOffFlywheel();
        clock.stopRamp();
        isFlywheelReady = false;
        isShotInProgress = false;
        clock.setNumBallsInClock(0);
    }
    public void stopAutoShot()
    {
        clock.resetClock();
        clock.stopRamp();
        isFlywheelReady = false;
        isShotInProgress = false;
        clock.setNumBallsInClock(0);
    }

}






