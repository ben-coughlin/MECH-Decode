package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.omega;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
public class AutoBlue extends RobotMasterPinpoint {

    private final double SCALE_FACTOR = 1.0;

    private long startTime = 0;

    private int cycle = 0;
    private int driveToGetSampleCycle = 0;


    private static final String SIMULATOR_HOST = "192.168.43.22";
    private static final int SIMULATOR_PORT = 7777;


    private enum progStates {
        driveBackwardsFromStartToShootPreload,
        driveToFirstThreeBalls,
        intakeFirstThreeBalls,
        driveToShootingPoint,
        driveToSecondThreeBalls,
        intakeSecondThreeBalls,
        driveToThirdThreeBalls,
        intakeThirdThreeBalls,
        SHOOT_PREP,
        SHOOT,
        endBehavior
    }

    @Override
    public void init() {
        //RobotMaster.resetEncoders = true;
        super.init();
        spindexer.setInventory(new Pattern(Pattern.Ball.GREEN, Pattern.Ball.PURPLE, Pattern.Ball.PURPLE));
        isAuto = true;


    }

    private int timeDelay = 0;

    @Override
    public void init_loop() {
        super.init_loop();

    }

    @Override
    public void start() {
        super.start();
        startTime = SystemClock.uptimeMillis();


    }

    private int pixelDropLocation = 0;

    private HashMap<Integer, PointDouble> purpleDrop = new HashMap<Integer, PointDouble>() {{ // this is from center stage season
        put(0, new PointDouble(105, 26));
        put(1, new PointDouble(103.24, 35));
        put(2, new PointDouble(109.62, 36.8));
    }};


    private boolean hasGrabbedPixels = false;
    private double lockedHeading;

    private double cutOffTime = 22.5;
    private String currentState = String.valueOf(AutoBlue.progStates.values()[AutoBlue.programStage]);

    private boolean past5In = false;

    public static boolean pickupOffWall = false;

    public int overallCycleToChamber = 0;
    private boolean isLookingAtObelisk = true;



    @Override
    public void mainLoop() {
        super.mainLoop();


        if(!isLookingAtObelisk)
        {
            if (Limelight.getCurrResult() != null && Limelight.getCurrResult().isValid() && VisionUtils.isTagBlueGoal(VisionUtils.getTagId(Limelight.getCurrResult())))
            {
                double llError = Limelight.getCurrResult().getTx();

                turret.aimTurret(true, llError, gamepad2.right_stick_x, limelight.getDistanceToTag(Limelight.getCurrResult()));
            }
        }
        else {

            turret.aimTurret(false, 0, -0.27, 999);
            if (Limelight.getCurrResult() != null && Limelight.getCurrResult().isValid() && VisionUtils.isTagBlueGoal(VisionUtils.getTagId(Limelight.getCurrResult())))
            {
                turret.aimTurret(false, 0, 0, 0);
                turret.resetEncoder();
                isLookingAtObelisk = false;
                return;
            }
        }


        boolean jamDetected = false;//pixelJamAndCounting();
        Log.i("DEBUG", "Current stage: " + programStage + " = " + progStates.values()[programStage].name());
        Log.i("DEBUG", "=== LOOP START === Stage: " + programStage + " = " + progStates.values()[programStage].name());
        Log.i("DEBUG", "isShotInProgress: " + shooterSubsystem.isShotInProgress + ", shotsRemaining: " + shooterSubsystem.shotsRemaining);

        if (programStage == progStates.driveBackwardsFromStartToShootPreload.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
                shooterSubsystem.spinUp();  // Start flywheel

            }
            shooterSubsystem.updateSpin();
            shooterSubsystem.updateActiveShot();
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-31, 0,
                    0.8 * SCALE_FACTOR, 0.30 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.3));


            if (Movement.followCurve(points, Math.toRadians(-90),2)) { //the second term is is if drive strait or the strafe angle 90 deg is strait ahead
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveToFirstThreeBalls.ordinal());
            }
            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveToFirstThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-31,28,
                    0.8 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.3));


//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.8));

            if (Movement.followCurve(points, Math.toRadians(0),2)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.intakeFirstThreeBalls.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.intakeFirstThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
                intakeSubsystem.turnIntakeOn();
                spindexer.startIntakeCycle();
            }
            spindexer.intakeNewBall();



            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-10, 28,
                    0.13 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.3));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.3));


            if (Movement.followCurve(points, Math.toRadians(90),2)) {
                drive.stopAllMovementDirectionBased();
                shooterSubsystem.isFlywheelSpun = true;
                nextStage(progStates.driveToShootingPoint.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveToShootingPoint.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
                shooterSubsystem.isFlywheelReady = false;
                shooterSubsystem.spinUp();  // Start flywheel
                intakeSubsystem.turnIntakeOff();
            }
            shooterSubsystem.updateSpin();
            shooterSubsystem.updateActiveShot();
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-31, 30,
                    0.8* SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.3));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.3));

            if (Movement.followCurve(points, Math.toRadians(-90),1)) {
                intakeSubsystem.turnIntakeOff();
                drive.stopAllMovementDirectionBased();
                shooterSubsystem.isFlywheelSpun = false;
                nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveToSecondThreeBalls.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveToSecondThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;

            }

                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));




                points.add(new CurvePoint(-31, 52,
                        0.8 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.3));



                if (Movement.followCurve(points, Math.toRadians(0),1)) {
                    drive.stopAllMovementDirectionBased();
                    nextStage(progStates.intakeSecondThreeBalls.ordinal());
                }

                drive.applyMovementDirectionBased(); // always put at end of state

        }

        if (programStage == progStates.intakeSecondThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;
                intakeSubsystem.turnIntakeOn();
                spindexer.startIntakeCycle();
                shooterSubsystem.spinUp();
            }

            spindexer.intakeNewBall();



            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-10, 54,
                    0.13 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.3));



            if (Movement.followCurve(points, Math.toRadians(90),1)) {
                drive.stopAllMovementDirectionBased();
                spindexer.intakeCycleActive = false;
                nextStage(progStates.SHOOT_PREP.ordinal(), progStates.driveToThirdThreeBalls.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state

        }
        if (programStage == progStates.driveToThirdThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;

            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-31, 71,
                    0.8 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.3));



            if (Movement.followCurve(points, Math.toRadians(0),1)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.intakeThirdThreeBalls.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state

        }

        if (programStage == progStates.intakeThirdThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;
                intakeSubsystem.turnIntakeOn();
                spindexer.startIntakeCycle();

            }

            spindexer.intakeNewBall();


            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-10, 71,
                    0.14 * SCALE_FACTOR, 0 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.5));


            movement_turn = 0; //very stupid hack
            if (Movement.followCurve(points, Math.toRadians(90), 1)) {
                drive.stopAllMovementDirectionBased();
                spindexer.intakeCycleActive = false;
                shooterSubsystem.reset();
                nextStage(progStates.endBehavior.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state

        }


        if (programStage == progStates.SHOOT_PREP.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                shooterSubsystem.spinUp();
                spindexer.rotateToNextSlotInPattern();
            }
            shooterSubsystem.updateSpin();
            shooterSubsystem.updateActiveShot();

            // wait until ready
            if (shooterSubsystem.isReadyToShoot()) {
                nextStage(progStates.SHOOT.ordinal());
            }
        }
        if (programStage == progStates.SHOOT.ordinal()) {
            shooterSubsystem.updateSpin();
            shooterSubsystem.updateActiveShot();

            if (stageFinished) {
                stageFinished = false;
                initializeStateVariables();
                shooterSubsystem.startMultiShot(3);
                Log.i("ShooterSubsystem", "SHOOTING 3");
            }


            if (shooterSubsystem.isReadyToShoot()) {
                Log.i("ShooterSubsystem", "progressing to " + progStates.values()[stageAfterShotOrdinal]);
                nextStage(stageAfterShotOrdinal);
            }
        }


        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            drive.stopAllMovementDirectionBased();

        }

        //tp4.markStart();

        //superstructure.update(telemetry, gamepad1, gamepad2);

        //tp4.markEnd();

        //System.out.println("Time Profiler 4 Average Time: "  tp4.getAverageTimePerUpdateMillis());

    }
}

