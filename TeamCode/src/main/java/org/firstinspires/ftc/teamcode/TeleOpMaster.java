package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;

import android.os.SystemClock;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class TeleOpMaster extends RobotMasterPinpoint {

    //state machines!!
    enum progStates {
        IDLE,
        DEBUG,
        STOP,
        SHOOT_PREP,
        READY_TO_SHOOT,
        FIRE_BALL,
        ROTATE_AFTER_SHOT,
        OUTTAKE,
        INTAKE,
    }

    private final ElapsedTime stateTimer = new ElapsedTime();
    PIDFController headingHold = new PIDFController(0.03, 0, 0.0001);
    Toggle circleToggle = new Toggle(false);
    Toggle squareToggle = new Toggle(false);
    Toggle autoAimToggle = new Toggle(true);
    Toggle intakeToggle = new Toggle(false);

    boolean isAutoHeading = false;
    boolean isAutoAiming = false;
    double targetHeading = 0.0;
    public boolean wasBallIntakeSuccessful;

    // Abstract method for alliance-specific goal checking
    protected abstract boolean isCorrectGoalTag(int tagId);

    // Override these if you need alliance-specific tuning
    protected double getShootPrepTime() {
        return 2700;
    }

    protected double getKickerVerticalTime() {
        return 200;
    }

    protected double getClockShootTime() {
        return 500;
    }

    protected double getShotCompleteTime() {
        return 1000;
    }

    protected double getOuttakeTime() {
        return 2;
    }

    @Override
    public void init() {
        isAuto = false;
        resetEncoders = false;
        super.init();
    }

    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void mainLoop() {
        super.mainLoop();
        currentState = progStates.values()[programStage].name();
        debugKeyValues.put("Superstructure State", currentState);

        if (wasBallIntakeSuccessful) {
            gamepad1.rumble(50);
            wasBallIntakeSuccessful = false;
        }

        // toggles
        circleToggle.updateToggle(gamepad1.circle);
        squareToggle.updateToggle(gamepad1.square);
        autoAimToggle.updateToggle(gamepad2.guide);
        intakeToggle.updateToggle(gamepad1.right_trigger > 0.1);

        // driving stuff
        movement_y = gamepad1.left_stick_y;
        movement_x = gamepad1.left_stick_x;

        double currentHeadingRad = RobotPosition.worldAngle_rad;
        double turnDeadzone = 0.05;

        if (Math.abs(gamepad1.right_stick_x) > turnDeadzone) {
            // if the driver is turning manually then that takes priority
            movement_turn = gamepad1.right_stick_x;
            isAutoHeading = false;
            targetHeading = currentHeadingRad;
            headingHold.reset();
        } else {
            // if there's no turning at all, use PID to hold us at the same heading
            headingHold.setReference(targetHeading);
            double error = headingHold.calculatePIDF(currentHeadingRad);
            movement_turn = (error < Math.toRadians(2)) ? Math.toRadians(0) : error;
            isAutoHeading = true;
        }

        drive.applyMovementDirectionBasedFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, movement_turn, isAutoHeading);

        // limelight
        isAutoAiming = autoAimToggle.getState();
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


        // honestly no idea why there are two separate statements but it works :P
        if (programStage == progStates.IDLE.ordinal()) {
            if (gamepad1.left_bumper || gamepad2.dpad_up) {
                nextStage(progStates.SHOOT_PREP.ordinal());
            } else if (gamepad1.left_trigger > 0.1) {
                intakeSubsystem.turnIntakeOff();
                nextStage(progStates.OUTTAKE.ordinal());
            }
        }
        // safe home state
        if (programStage == progStates.IDLE.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                turret.turnOffFlywheel();
                intakeSubsystem.turnIntakeOff();
                clock.resetClock();
            }
        }


        if (programStage == progStates.READY_TO_SHOOT.ordinal()) {

            if (gamepad1.right_bumper || gamepad2.dpad_down) {
                nextStage(progStates.FIRE_BALL.ordinal());
            }
            // cancels the shot
            else if (gamepad1.circle || gamepad2.circle) {
                shooterSubsystem.stopShot();

            }
        }

        if (intakeToggle.getState()) {
            nextStage(progStates.INTAKE.ordinal());
        } else { // WHEN the trigger is RELEASED
            if (programStage == progStates.INTAKE.ordinal()) {
                nextStage(progStates.IDLE.ordinal());
            }
        }



        if (programStage == progStates.SHOOT_PREP.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                shooterSubsystem.spinUp();
            }
            shooterSubsystem.updateSpin();
            if (shooterSubsystem.isFlywheelReady) {
                nextStage(progStates.READY_TO_SHOOT.ordinal());
            }
        }

        if (programStage == progStates.READY_TO_SHOOT.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                gamepad1.rumble(1, 1, 200);
                gamepad2.rumble(1, 1, 200);
            }
        }

        if (programStage == progStates.FIRE_BALL.ordinal()) {
            shooterSubsystem.updateSpin();
            if (stageFinished) {
                stageFinished = false;
                initializeStateVariables();
            }
            shooterSubsystem.startShotSequence();
            //use the shooterSubsys booleans instead of our own timers because shooterSubsys is a strong independent woman that can handle itself - and so we don't double-time stuff
            if (!shooterSubsystem.isShotInProgress) {
                //we reset the clock in idle so no need to do it here
                nextStage(progStates.IDLE.ordinal());
            }
        }

        if (programStage == progStates.OUTTAKE.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }
            if (stateTimer.seconds() >= getOuttakeTime()) {
                intakeSubsystem.outtake();
                nextStage(progStates.IDLE.ordinal());
            }
        }

        if (programStage == progStates.INTAKE.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                intakeSubsystem.turnIntakeOn();
            }

        }

        // kills everything
        if (programStage == progStates.STOP.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }
            drive.stopAllMovementDirectionBased();
            intakeSubsystem.turnIntakeOff();
            turret.turnOffFlywheel();
        }
    }
}