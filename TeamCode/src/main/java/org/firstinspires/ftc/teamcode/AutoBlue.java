package org.firstinspires.ftc.teamcode;

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
        hangSpecimen,
        driveUpToSamples,
        strafeToSamples,
        pushSamplesToPlayerStation,

        pushSampleToPickUpSpecimen,
        SHOOT_PREP,
        SHOOT,
        endBehavior
    }

    @Override
    public void init() {
        //RobotMaster.resetEncoders = true;
        super.init();
        spindexer.setInventory(new Pattern(Pattern.Ball.GREEN, Pattern.Ball.PURPLE, Pattern.Ball.PURPLE));
        //isAuto = true;


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
        System.out.println("Superstructure State: " + currentState);
        System.out.println("Path State: " + programStage);
        Log.i("heading", String.valueOf(Math.toDegrees(worldAngle_rad)));
        Log.i("state",  String.valueOf(AutoBlue.progStates.values()[AutoBlue.programStage]));

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
                    0.8 * SCALE_FACTOR, 0.40 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));


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
                    Math.toRadians(60), 0.6));


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

            points.add(new CurvePoint(-11, 28,
                    0.13 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.6));


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
                intakeSubsystem.outtake();
            }
            shooterSubsystem.updateSpin();
            shooterSubsystem.updateActiveShot();
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-31, 30,
                    0.8* SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.6));

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
                        Math.toRadians(60), 0.6));



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
            }

            spindexer.intakeNewBall();

            if(spindexer.isAtTargetPosition())
            {
                spindexer.startIntakeCycle();
            }
            spindexer.intakeNewBall();


            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-10, 54,
                    0.105 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));



            if (Movement.followCurve(points, Math.toRadians(90),1)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.endBehavior.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state

        }

        if (programStage == progStates.intakeThirdThreeBalls.ordinal()) {
            if (stageFinished) {
                past5In = false;

                //superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_TRANSPORT.ordinal());
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 400) {
                pickupOffWall = true;
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 500) {

                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));


                double destinationX = Math.abs(Math.hypot(worldXPosition - 29, worldYPosition - (-7 + (overallCycleToChamber * -7)))) < 10 ? 28 : 20;

                points.add(new CurvePoint(destinationX, -7 + (overallCycleToChamber * -7),
                        1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 10,
                        Math.toRadians(60), 0.6));

                double relativePointAngle = AngleWrap(Math.toRadians(180) - worldAngle_rad);

                if (Movement.followCurve(points, Math.toRadians(-90)) && Math.abs(relativePointAngle) < Math.toRadians(4)) {
                    drive.stopAllMovementDirectionBased();
                    overallCycleToChamber++;

                    System.out.println("WorldPosXHang: " + worldXPosition);
                    System.out.println("WorldPosYHang: " + worldYPosition);

                    nextStage(progStates.hangSpecimen.ordinal());

                    //nextStage(progStates.endBehavior.ordinal());
                }

                if (Math.abs(Math.hypot(worldXPosition - 29, worldYPosition - (-7 + (overallCycleToChamber * -7)))) < 28) {
                    if (!past5In) {

                        past5In = true;
                    }

                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(180),
                            1,
                            Math.toRadians(30));
                }
                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveUpToSamples.ordinal()) {
            if (stageFinished) {
                past5In = false;

                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(45, stateStartingY,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 20,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(-90), 3)) {
                drive.stopAllMovementDirectionBased();

                //nextStage(progStates.hangSpecimen.ordinal());
                nextStage(progStates.strafeToSamples.ordinal());
            }
            drive.applyMovementDirectionBased();

        }
        if (programStage == progStates.strafeToSamples.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }


            movement_x = -0.7;

            if (worldYPosition < -49 + (driveToGetSampleCycle * -9)) { // 51 was 52 for northern
                drive.stopAllMovementDirectionBased();

                //nextStage(progStates.hangSpecimen.ordinal());
                nextStage(progStates.pushSamplesToPlayerStation.ordinal());
            }

            Movement.movementResult r = Movement.pointAngle(
                    Math.toRadians(180),
                    0.9,
                    Math.toRadians(30));

            drive.applyMovementDirectionBased();


        }
        if (programStage == progStates.pushSamplesToPlayerStation.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, -50 + (driveToGetSampleCycle * -9), //adjust here to change push specimen starting location
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(15, -51 + (driveToGetSampleCycle * -9),
                    0.9, 0.9, 20, 20,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(90),3)) {
                drive.stopAllMovementDirectionBased();

                //nextStage(progStates.hangSpecimen.ordinal());
                driveToGetSampleCycle++;
                if (driveToGetSampleCycle == 2) {
                    cycle = 0;
                    nextStage(progStates.pushSampleToPickUpSpecimen.ordinal());
                } else {
                    nextStage(progStates.driveUpToSamples.ordinal());
                }
            }

            Movement.movementResult r = Movement.pointAngle(
                    Math.toRadians(180),
                    0.9,
                    Math.toRadians(30));

            drive.applyMovementDirectionBased();

        }
        if (programStage == progStates.pushSampleToPickUpSpecimen.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(15, -45,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 15, 10,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(0))) {
                drive.stopAllMovementDirectionBased();
                //superstructure.nextState(Superstructure3Motor.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());

                nextStage(progStates.driveToThirdThreeBalls.ordinal());
            }

            Movement.movementResult r = Movement.pointAngle(
                    Math.toRadians(180),
                    0.9,
                    Math.toRadians(30));

            drive.applyMovementDirectionBased();

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

