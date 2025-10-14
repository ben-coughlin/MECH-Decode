package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.Locale;

public abstract class RobotMasterPinpoint extends OpMode {
    MecanumDrivePinPoint drive = null;

    Odo odo = null;
    ColorSensor colorSensor = null;
    Limelight limelight = null;
    Pattern obelisk = null;
    IntakeSubsystem intakeSubsystem = null;


    Pattern currentPattern = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);

    public boolean isAuto = false;
    public static boolean resetEncoders = false;



//clocks

    ElapsedTime runtime = new ElapsedTime();


    //////// STATE MACHINE STUFF BELOW DO NOT TOUCH ////////
    public boolean stageFinished = true;
    public long stateStartTime = 0;

    public long programStartTime = 0;//time the program starts
    public static int programStage = 0;

    /**
     * STATE STARTING VARIABLES
     */
    public double stateStartingX = 0;
    public double stateStartingY = 0;
    public double stateStartingAngle_rad = 0;

    private final boolean DEBUGGING = false;

    private boolean inDebugState = false;

    //holds the stage we are going to next
    int nextStage = 0;

    public void nextStage(int ordinal) {
        nextStage = ordinal;
        //waits for a if on debug mode
        if (!DEBUGGING) {
            incrementStage();
            inDebugState = false;
        }

        //go into debug mode
        if (DEBUGGING) {
            inDebugState = true;
        }
    }

    /**
     * Increments the programStage
     */
    public void nextStage() {
        nextStage(programStage + 1);

    }

    private void incrementStage() {
        programStage = nextStage;
        stageFinished = true;
    }
    ///////////////////////////////

    public ArrayList<CurvePoint> mirrorPoints(ArrayList<CurvePoint> points) {
        ArrayList<CurvePoint> newPoints = new ArrayList<>();
        for (CurvePoint point : points) {
            newPoints.add(new CurvePoint(-point.x, point.y, point.moveSpeed, point.turnSpeed, point.followDistance, point.pointLength, point.slowDownTurnRadians, point.slowDownTurnRadians));
        }
        return newPoints;
    }


    @Override
    public void init() {


        drive = new MecanumDrivePinPoint(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);


    }

    @Override
    public void init_loop() {


        double startLoopTime = SystemClock.uptimeMillis();

        odo.updateOdo();
        odo.showOdoTelemetry(telemetry);

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));

    }

    @Override
    public void start() {
        programStage = 0;

    }
    @Override
    public void stop()
    {
        drive.stopAllMovementDirectionBased();
        limelight.stop();
    }


    @Override
    public void loop() {
        double startLoopTime = SystemClock.uptimeMillis();
        //read everything once and only once per loop
        colorSensor.updateDetection();
        limelight.updateLimelight();
        intakeSubsystem.updateIntakeSubsystem();
        odo.updateOdo();
        odo.showOdoTelemetry(telemetry);
        mainAutoLoop();

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }

    public void initializeStateVariables() {
        stateStartingX = worldXPosition;
        stateStartingY = worldYPosition;
        stateStartingAngle_rad = worldAngle_rad;
        stateStartTime = SystemClock.uptimeMillis();
        Movement.initCurve();
        stageFinished = false;
    }

    private void mainAutoLoop() {
        if (inDebugState) {
            drive.stopAllMovementDirectionBased();
            // ControlMovement(); CHANGE THIS

            telemetry.addLine("in debug state");
            if (gamepad1.a) {
                incrementStage();
                inDebugState = false;
            }
        } else {
            mainLoop();
        }
    }

    /**
     * I honestly don't know why this is here - it does nothing.
     */
    public void mainLoop() {
    }


}
