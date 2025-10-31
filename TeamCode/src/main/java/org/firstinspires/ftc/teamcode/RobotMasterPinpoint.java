package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class RobotMasterPinpoint extends OpMode {

    //hardware - - - - - -
    MecanumDrivePinPoint drive = null;
    Odo odo = null;
    ColorSensor colorSensor = null;
    Limelight limelight = null;
    Pattern obelisk = null;
    IntakeSubsystem intakeSubsystem = null;
    Turret turret = null;
    Spindexer spindexer = null;

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
        clientPlot.close();
        client.close();
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
        //read everything once and only once per loop
        colorSensor.updateDetection();
        limelight.updateLimelight();
        intakeSubsystem.updateIntakeSubsystem();
        odo.updateOdo();
        turret.updateTurret();
        spindexer.update();

        odo.showOdoTelemetry(telemetry);
        turret.showAimTelemetry(telemetry);
        intakeSubsystem.showSpindexerTelemetry(telemetry);
        colorSensor.showColorSensorTelemetry(telemetry);

        if(DEBUGGING)
        {
            addDebugTelemetry(startLoopTime);
        }
        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }

    /**
     * adds debug telemetry when in debug mode - we don't want this normally bc of loop times
     */
    private void addDebugTelemetry(long startLoopTime)
    {

        for(String k : debugKeyValues.keySet())
        {
            client.sendKeyValue(k, debugKeyValues.get(k));
        }

        clientPlot.sendLineY(startLoopTime, turret.autoAim.getProportional(), 1);
        clientPlot.sendLineY(startLoopTime, turret.autoAim.getDerivative(), 2);
        clientPlot.sendLineY(startLoopTime, turret.autoAim.getIntegral(), 3);
    }
    private void initDebugTools()
    {
        client = new UdpClientFieldSim("192.168.43.83", 7777);
        clientPlot = new UdpClientPlot("192.168.43.83", 7778);

        clientPlot.sendYLimits(SystemClock.uptimeMillis(), 0.5, 0);
        clientPlot.sendYUnits(SystemClock.uptimeMillis() + 1, "PID");
    }









}
