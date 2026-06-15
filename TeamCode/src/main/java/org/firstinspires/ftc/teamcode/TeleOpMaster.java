package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.Toggle;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public abstract class TeleOpMaster extends RobotMaster {



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
    Toggle outttakeToggle = new Toggle(false);

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
        follower = Constants.createFollower(hardwareMap);
       // follower.setStartingPose( new Pose(10, 115, Math.toRadians(180)));
        follower.update();
        telemetry.addData("Selected", selectedProgram);


    }

    public void init_loop() {
        telemetry.addData("Selected", selectedProgram);
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
        follower.update();


        drive.teleopDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.getHeading());



        currentState = progStates.values()[programStage].name();
        debugKeyValues.put("Superstructure State", currentState);

        // toggles
        circleToggle.updateToggle(gamepad1.circle);
        squareToggle.updateToggle(gamepad1.square);
        autoAimToggle.updateToggle(gamepad2.guide);
        intakeToggle.updateToggle(gamepad1.right_trigger > 0.1);
        outttakeToggle.updateToggle(gamepad1.left_trigger > 0.1);

        // limelight
        isAutoAiming = autoAimToggle.getState();
        LLResult currVision = Limelight.getCurrResult();
        boolean hasValidVision = currVision != null
                && currVision.isValid()
                && isCorrectGoalTag(VisionUtils.getTagId(currVision));

       // rumble when we're not using vision

            if (intakeToggle.getState()) {
                //purple when intake on

                intake.turnIntakeOn();
                transfer.turnTransferOn();

            } else if (!intakeToggle.getState() && gamepad1.left_trigger > 0.1)
            {
                //orange when outtake on
                intake.turnOuttakeOn();

            }
            else
            {
                transfer.turnTransferOff();
                intake.turnIntakeOff();

            }


        // ALWAYS call aimTurret - it will use odometry if vision is lost
        turret.aimTurret(
                hasValidVision,
                hasValidVision ? Limelight.getCurrResult().getTx() : 0,
                gamepad2.right_stick_x,
                true,
                0.0 //doesn't matter since we're using autoaim
        );

        if(gamepad1.touchpad)
        {
            kickstand.lowerKickstand();
        }

        runStateMachines();
        //telemetry.addData("X: ", follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getHeading()));


    }




    private void runStateMachines()
    {
        switch (progStates.values()[programStage]) {
            case IDLE:
                if (gamepad1.left_bumper || gamepad2.dpad_up) {
                    ShooterSubsystem.isFlywheelReady = false;
                    incrementStage(progStates.SHOOT_PREP.ordinal());
                }
                else if (stageFinished) {
                    initializeStateVariables();
                    turret.turnOffFlywheel();
                    intake.turnIntakeOff();


                }
                break;

            case SHOOT_PREP:
                if (stageFinished) {
                    initializeStateVariables();
                    shooterSubsystem.spinUp();
                }
                shooterSubsystem.updateSpin();
                if (ShooterSubsystem.isFlywheelReady && !stageFinished) {
                    incrementStage(progStates.READY_TO_SHOOT.ordinal());
                }
                break;

            case READY_TO_SHOOT:
                if (stageFinished) {
                    initializeStateVariables();
                    gamepad1.rumble(1, 1, 200);
                    gamepad2.rumble(1, 1, 200);
                }

                if ((gamepad1.right_bumper || gamepad2.dpad_down)) {
                    incrementStage(progStates.FIRE_BALL.ordinal());
                }
                // cancels the shot
                else if (gamepad1.circle || gamepad2.circle) {
                    shooterSubsystem.runStopActions();
                    incrementStage(progStates.IDLE.ordinal());
                }
                break;

            case FIRE_BALL:
                shooterSubsystem.updateSpin();
                if (stageFinished) {
                    initializeStateVariables();
                }
                shooterSubsystem.startShotSequence(isAuto);
                if (gamepad1.circle || gamepad2.circle) {
                    shooterSubsystem.runStopActions();

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
                intake.turnIntakeOff();

                break;

            default:
                break;
        }




    }
}