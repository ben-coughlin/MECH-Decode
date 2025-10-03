package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;

@Autonomous
@Disabled
public class AutoTemplate extends RobotMasterPinpoint {

    private final double SCALE_FACTOR = 1.0;

    //this enum is the master list of all states that should be executed
    public enum progStates {

        driveForward,
        strafeLeft,
        driveBackward,
        strafeRight,


        endBehavior
    }




    @Override
    public void init() {
        super.init();

    }

    @Override
    public void init_loop() {
        super.init_loop();

    }

    @Override
    public void start() {
        super.start();



    }

    private String currentState = String.valueOf(progStates.values()[AutoTemplate.programStage]);

    @Override
    public void mainLoop() {


        telemetry.addData("Superstructure State", currentState);
        telemetry.addData("Path State", programStage);
        System.out.println("Superstructure State: " + currentState);
        System.out.println("Path State: " + programStage);

        if (programStage == progStates.driveForward.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(20, 0,
                    0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 20, 10, // changed move speed from .35 to .45
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(90),2)) { //the second term is the **DIRECTION THE BOT SHOULD DRIVE IN, NOT THE DIRECTION IT IS CURRENTLY FACING**
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.strafeLeft.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.strafeLeft.ordinal()) {
            if (stageFinished) {

                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(20,10,
                    0.35 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(0),2)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.driveBackward.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveBackward.ordinal()) {
            if (stageFinished) {

                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(0, 10,
                    0.35 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(-90),2)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.strafeRight.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.strafeRight.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(0, 0,
                    0.35 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));


            if (Movement.followCurve(points, Math.toRadians(180),2)) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.endBehavior.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }


        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {

                initializeStateVariables();
            }
            drive.stopAllMovementDirectionBased();

        }



    }
}

