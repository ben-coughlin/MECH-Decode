package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;
import android.util.Log;

import org.firstinspires.ftc.teamcode.TeleOpMaster;

public class ShooterSubsystem {


    private final Turret turret;
    private final Intake intake;
    private final Transfer transfer;

    public boolean isFlywheelReady = false;
    public static boolean isShotInProgress = false;
    public boolean isFlywheelSpun = false;

    private long flywheelStartTime = 0;
    private long shotStartTime = 0;




    public ShooterSubsystem(Turret turret, Intake intake, Transfer transfer) {
        this.turret = turret;
        this.intake = intake;
        this.transfer = transfer;
    }



    /**
     * Call ONCE to begin flywheel spin up
     */
    public void spinUp() {
        if (!isFlywheelReady) {
            turret.turnOnFlywheel();
            flywheelStartTime = SystemClock.uptimeMillis();
            Log.i("ShooterSubsystem", "SpinUp called at " + flywheelStartTime);
            transfer.turnTransferOff();
            intake.turnIntakeOff();
        }
    }

    /**
     * Call every loop until ready
     */
    public void updateSpin() {
        if ((!isFlywheelReady && SystemClock.uptimeMillis() - flywheelStartTime > 800) || isFlywheelSpun) {
            Log.i("ShooterSubsystem", "Spin update elapsed = " +
                    (SystemClock.uptimeMillis() - flywheelStartTime));
            isFlywheelReady = true;

        }
    }

    /**
     * Begin shooting sequence
     */
    public void startShotSequence(boolean isAuto) {
        if (!isFlywheelReady) return;

        // Only initialize on first call
        if (!isShotInProgress) {
            isShotInProgress = true; //this flag starts the transfer
            shotStartTime = SystemClock.uptimeMillis();
            Log.i("ShooterSubsystem", "shot seq started at " + shotStartTime);
        }

    }

    // In TeleOp, call this when driver presses button:
    public void stopShot() {

        runStopActions();
    }


    public void runStopActions() {
        turret.turnOffFlywheel();
        transfer.setTriggerToBlock();
        transfer.turnTransferOff();

        isFlywheelReady = false;
        isShotInProgress = false;

    }
    public void stopAutoShot()
    {

        isFlywheelReady = false;
        isShotInProgress = false;

    }

}






