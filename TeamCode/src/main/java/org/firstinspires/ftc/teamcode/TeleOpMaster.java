package org.firstinspires.ftc.teamcode;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.Toggle;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public abstract class TeleOpMaster extends RobotMaster {

    private Follower follower;
    public static Pose startingPose;
    public static String selectedProgram;
    private TelemetryManager telemetryM;

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
    private final ElapsedTime clockResetTimer = new ElapsedTime();
    Toggle circleToggle = new Toggle(false);
    Toggle squareToggle = new Toggle(false);
    Toggle autoAimToggle = new Toggle(true);
    Toggle intakeToggle = new Toggle(false);

    public static boolean hasClockReset = false;
    boolean isAutoAiming = false;


    protected abstract boolean isCorrectGoalTag(int tagId);


    protected double getOuttakeTime() {
        return 2;
    }

    @Override
    public void init() {
        super.init();
        isAuto = false;
        resetEncoders = false;
        Constants.createFollower(hardwareMap).pathConstraints.setBrakingStrength(2);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    public void init_loop() {
        telemetry.addData("Selected", selectedProgram);
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        clock.initClock();
        follower.startTeleopDrive(false);

    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void mainLoop() {
        super.mainLoop();
        follower.update();

        currentState = progStates.values()[programStage].name();
        debugKeyValues.put("Superstructure State", currentState);

        // toggles
        circleToggle.updateToggle(gamepad1.circle);
        squareToggle.updateToggle(gamepad1.square);
        autoAimToggle.updateToggle(gamepad2.guide);
        intakeToggle.updateToggle(gamepad1.right_trigger > 0.1);

        // limelight
        isAutoAiming = autoAimToggle.getState();
        LLResult currVision = Limelight.getCurrResult();
        boolean hasValidVision = currVision != null
                && currVision.isValid()
                && isCorrectGoalTag(VisionUtils.getTagId(currVision));

       // rumble when we're not using vision
        if(turret.getTrackingModeForTelemetry().contains("VELOCITY"))
        {
            gamepad2.rumble(5);
        }
        //blip the gamepad so i know it's about to auto-center
        else if(turret.getTrackingModeForTelemetry().contains("HEADING"))
        {
            gamepad2.rumbleBlips(1);
        }

        if(turret.getTrackingModeForTelemetry().contains("VISION") && programStage == progStates.READY_TO_SHOOT.ordinal())
        {
            IndicatorLight.setLightAzure();
        }
        else if(!turret.getTrackingModeForTelemetry().contains("VISION") && programStage == progStates.READY_TO_SHOOT.ordinal())
        {
            IndicatorLight.setLightWhite();
        }



        // ALWAYS call aimTurret - it will use odometry if vision is lost
        turret.aimTurret(
                hasValidVision,
                hasValidVision ? Limelight.getCurrResult().getTx() : 0,
                gamepad2.right_stick_x,
                Limelight.getDistance(),
                follower.getHeading(),
                follower.getAngularVelocity(),
                follower.getVelocity().getMagnitude()
        );

        runStateMachines();

        //todo: tune directions and power amounts
        if(gamepad1.dpad_left)
        {
            follower.setTeleOpDrive(
                    0,
                    0,
                    0.2,
                    false // field Centric
            );
        }
        else if(gamepad1.dpad_right)
        {
            follower.setTeleOpDrive(
                    0,
                    -0,
                    -0.2,
                    false // field Centric
            );
        }
        else
        {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // field Centric
            );
        }





        // Telemetry at the end to show final state
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.update();
    }

    private void runStateMachines()
    {
        switch (progStates.values()[programStage]) {
            case IDLE:
                if (gamepad1.left_bumper || gamepad2.dpad_up) {
                    shooterSubsystem.isFlywheelReady = false;
                    incrementStage(progStates.SHOOT_PREP.ordinal());
                }
                else if (gamepad1.left_trigger > 0.1) {
                    intakeSubsystem.turnIntakeOff();
                    incrementStage(progStates.OUTTAKE.ordinal());
                } else if (stageFinished) {
                    initializeStateVariables();
                    turret.turnOffFlywheel();
                    intakeSubsystem.turnIntakeOff();
                    IndicatorLight.turnLightOff();
                    clock.resetClock();
                    clock.stopRamp();
                }
                break;

            case SHOOT_PREP:
                if (stageFinished) {
                    initializeStateVariables();
                    shooterSubsystem.spinUp();
                    clock.moveClockToPreShootPosition();
                }
                shooterSubsystem.updateSpin();
                if (shooterSubsystem.isFlywheelReady && !stageFinished) {
                    incrementStage(progStates.READY_TO_SHOOT.ordinal());
                }
                break;

            case READY_TO_SHOOT:
                if (stageFinished) {
                    initializeStateVariables();
                    gamepad1.rumble(1, 1, 200);
                    gamepad2.rumble(1, 1, 200);
                    clock.moveClockToPreShootPosition();
                }

                if ((gamepad1.right_bumper || gamepad2.dpad_down)) {
                    incrementStage(progStates.FIRE_BALL.ordinal());
                }
                // cancels the shot
                else if (gamepad1.circle || gamepad2.circle) {
                    shooterSubsystem.stopShot();
                    incrementStage(progStates.IDLE.ordinal());
                }
                break;

            case FIRE_BALL:
                shooterSubsystem.updateSpin();
                if (stageFinished) {
                    IndicatorLight.setLightOrange();
                    initializeStateVariables();
                    clock.setRampToShootPower();
                }
                shooterSubsystem.startShotSequence(isAuto);
                if (gamepad1.circle || gamepad2.circle) {
                    shooterSubsystem.stopShot();
                    hasClockReset = false;
                    clockResetTimer.reset();
                    incrementStage(progStates.IDLE.ordinal());
                }
                break;

            case OUTTAKE:
                if (stageFinished) {
                    initializeStateVariables();
                }
                if (stateTimer.seconds() >= getOuttakeTime()) {
                    intakeSubsystem.outtake();
                    incrementStage(progStates.IDLE.ordinal());
                }
                break;

            case INTAKE:
                if (stageFinished) {
                    initializeStateVariables();


                }
                break;

            case STOP:
                if (stageFinished) {
                    initializeStateVariables();
                }
                follower.setTeleOpDrive(0,0,0);
                intakeSubsystem.turnIntakeOff();
                //turret.turnOffFlywheel();
                break;

            default:
                break;
        }

        if (intakeToggle.getState()) {
                IndicatorLight.setLightGreen();
                intakeSubsystem.turnIntakeOn();

        } else {
            intakeSubsystem.turnIntakeOff();
        }



    }
}