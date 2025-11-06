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

import java.util.HashMap;

public abstract class RobotMasterPinpoint extends OpMode {

    //hardware - - - - - -
    MecanumDrivePinPoint drive = null;
    Odo odo = null;
    ColorSensor colorSensor = null;
    Limelight limelight = null;
    IntakeSubsystem intakeSubsystem = null;
    Turret turret = null;
    Spindexer spindexer = null;
    PoseFusion pose = new PoseFusion();

    // ftcsim stuff - - - - - - - -
    private UdpClientFieldSim client;
    private UdpClientPlot clientPlot;

    private boolean DEBUGGING = false;
    //global keyvalue store
    public HashMap<String, String> debugKeyValues = new HashMap<>();

    //misc
    public boolean isAuto = false;
    public static boolean resetEncoders = false;



    //clocks

    ElapsedTime runtime = new ElapsedTime();


    //////// STATE MACHINE STUFF BELOW DO NOT TOUCH ////////
    public boolean stageFinished = true;
    public long stateStartTime = 0;
    public static int programStage = 0;
    public String currentState;

    /**
     * STATE STARTING VARIABLES
     */
    public double stateStartingX = 0;
    public double stateStartingY = 0;
    public double stateStartingAngle_rad = 0;


    //holds the stage we are going to next
    int nextStage = 0;

    public void nextStage(int ordinal) {
        nextStage = ordinal;
        incrementStage();
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



    @Override
    public void init() {

        drive = new MecanumDrivePinPoint(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        odo = new Odo(hardwareMap);
        turret = new Turret(hardwareMap);
        spindexer = new Spindexer(hardwareMap, colorSensor);
        pose.reset(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        if(gamepad1.options)
        {
            DEBUGGING = true;
            initDebugTools();
        }


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
    }


    @Override
    public void loop() {
       mainLoop();
    }

    public void initializeStateVariables() {
        stateStartingX = worldXPosition;
        stateStartingY = worldYPosition;
        stateStartingAngle_rad = worldAngle_rad;
        stateStartTime = SystemClock.uptimeMillis();
        Movement.initCurve();
        stageFinished = false;
    }




    public void mainLoop() {
        long startLoopTime = SystemClock.uptimeMillis();
        //sensor fusion stuff - atm we're just testing with this
        pose.update(odo.pos);
        Pose2D fusedPose = pose.getPoseEstimate();
        telemetry.addData("Fused X", fusedPose.getX(DistanceUnit.INCH));
        telemetry.addData("Fused Y", fusedPose.getY(DistanceUnit.INCH));
        telemetry.addData("Fused Heading", "%.2f rad  |  %.2f deg",
                fusedPose.getHeading(AngleUnit.RADIANS),
                fusedPose.getHeading(AngleUnit.DEGREES));

        //read everything once and only once per loop
        colorSensor.updateDetection();
        limelight.updateLimelight();
        odo.updateOdo();
        turret.updateTurret();
        spindexer.update();

        odo.showOdoTelemetry(telemetry);
        turret.showAimTelemetry(telemetry);
        colorSensor.showColorSensorTelemetry(telemetry);
        spindexer.showTelemetry(telemetry);
        spindexer.intakeNewBall();

        if(DEBUGGING)
        {
            addDebugTelemetry();
        }
        telemetry.addData("Superstructure State", currentState);
        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }

    /**
     * adds debug telemetry when in debug mode - we don't want this normally bc of loop times
     */
    private void addDebugTelemetry()
    {

        for(String k : debugKeyValues.keySet())
        {
            client.sendKeyValue(k, debugKeyValues.get(k));
        }


    }
    private void initDebugTools()
    {
        client = new UdpClientFieldSim("192.168.43.83", 7777);
        clientPlot = new UdpClientPlot("192.168.43.83", 7778);

        clientPlot.sendYLimits(SystemClock.uptimeMillis(), 0.5, 0);
        clientPlot.sendYUnits(SystemClock.uptimeMillis() + 1, "PID");
    }









}
