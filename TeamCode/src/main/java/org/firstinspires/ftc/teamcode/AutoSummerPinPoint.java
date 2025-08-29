package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
public class AutoSummerPinPoint extends RobotMasterPinpoint {

    private final double SCALE_FACTOR = 1;

    private long startTime = 0;

    private int cycle = 0;
    private int driveToGetSampleCycle = 0;

    public enum progStates {

        driveForward,
        strafeLeft,
        strafeRight,
        driveBackward,
        driveToPlayerStation,
        grabSpecimen,
        driveToChamber,
        hangSpecimen,

        driveUpToSamples,

        strafeToSamples,
        pushSamplesToPlayerStation,

        pushSampleToPickUpSpecimen,

        endBehavior
    }




    @Override
    public void init() {
        //RobotMaster.resetEncoders = true;
        super.init();

        //isAuto = true;

        odo.recalibrateIMU();
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
        odo.resetPosAndIMU();

    }

    private int pixelDropLocation = 0;

    private HashMap<Integer, PointDouble> purpleDrop = new HashMap<Integer, PointDouble>() {{ // this is from center stage season
        put(0, new PointDouble(105, 26));
        put(1, new PointDouble(103.24, 35));
        put(2, new PointDouble(109.62, 36.8));
    }};


    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;
    private int currentState = AutoSummerPinPoint.programStage;

    private boolean past5In = false;

    public static boolean pickupOffWall = false;

    public int overallCycleToChamber = 0;

    @Override
    public void mainLoop() {

        boolean jamDetected = false;//pixelJamAndCounting();
        telemetry.addData("Superstructure State", currentState);
        telemetry.addData("Path State", programStage);
        System.out.println("Superstructure State: " + currentState);
        System.out.println("Path State: " + programStage);

        if (programStage == progStates.driveForward.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(20, 0,
                    0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 20, 10, // changed move speed from .35 to .45
                    Math.toRadians(0), 0.6));







//            points.add(new CurvePoint(15, 0,
//                    0.4 * SCALE_FACTOR, 0.40 * SCALE_FACTOR, 10, 10,
//                    Math.toRadians(60), 0.6));

//            points.add(new CurvePoint(20, 48,
//                    0.4 * SCALE_FACTOR, 0.40 * SCALE_FACTOR, 5, 10,
//                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(90),2)) { //the second term is is if drive strait or the strafe angle 90 deg is strait ahead
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.strafeLeft.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.strafeLeft.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(20,10,
                    0.35 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.6));

            if (Movement.followCurve(points, Math.toRadians(0),2)) {
                drive.stopAllMovementDirectionBased();
                //nextStage(progStates.endBehavior.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.strafeRight.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(15, 0,
                    0.35 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.6));


            if (Movement.followCurve(points, Math.toRadians(180),2)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.driveBackward.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveBackward.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(0, 0,
                    0.35 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

//            points.add(new CurvePoint(24, 35,
//                    0.25 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 12, 10,
//                    Math.toRadians(-90), 0.6));

            if (Movement.followCurve(points, Math.toRadians(-90),1)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.endBehavior.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveToPlayerStation.ordinal()) {
            if (stageFinished) {
                past5In = false;
                if (cycle == 1) {
                    //superstructure.nextState(Superstructure3Motor.SuperstructureStates.GOTO_RESTING_WORLDS.ordinal());
                } else {
                   // superstructure.nextState(Superstructure3Motor.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());
                }

                System.out.println("Starting X" + stateStartingX);
                System.out.println("Starting Y" + stateStartingY);
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 100) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));


                double wantedX = cycle == 1 ? 26 : 15;

                points.add(new CurvePoint(wantedX, -45,
                        1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                boolean completed = Movement.followCurve(points, Math.toRadians(90), 2);

                double relativePointAngle = AngleWrap(Math.toRadians(180) - worldAngle_rad);

                if (worldYPosition < -15 && cycle == 1) {
                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(180),
                            1,
                            Math.toRadians(30));
                } else  if (worldYPosition < -25) {
                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(180),
                            1,
                            Math.toRadians(30));
                }

                if (completed && Math.abs(relativePointAngle) < Math.toRadians(4)) {
                    drive.stopAllMovementDirectionBased();
                    cycle++;
                    //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());

                    if(cycle<2) {
                        nextStage(progStates.grabSpecimen.ordinal());
                    }else {
                        nextStage(progStates.driveUpToSamples.ordinal());
                    }
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.grabSpecimen.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 250) {

                if (!past5In) {
                    //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());
                    past5In = true;
                }

                movement_y = 0.20;

                if (Math.abs(Math.hypot(worldXPosition - 10, worldYPosition - (-45))) < 1) {
                    if (!past5In) {
                        //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal());
                        past5In = true;
                    }
                }

                if (SystemClock.uptimeMillis()-stateStartTime>1250) {
                    pickupOffWall = false;

                    drive.stopAllMovementDirectionBased();
                    nextStage(progStates.driveToChamber.ordinal());
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveToChamber.ordinal()) {
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

                nextStage(progStates.grabSpecimen.ordinal());
            }

            Movement.movementResult r = Movement.pointAngle(
                    Math.toRadians(180),
                    0.9,
                    Math.toRadians(30));

            drive.applyMovementDirectionBased();

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

