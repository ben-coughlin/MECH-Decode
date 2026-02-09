package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


import java.util.HashMap;

public abstract class RobotMasterPinpoint extends OpMode {

    //hardware - - - - - -
    MecanumDrivePinPoint drive = null;
  //  Odo odo = null;
    Limelight limelight = null;
    IntakeSubsystem intakeSubsystem = null;
    Turret turret = null;
    Clock clock = null;
    PoseFusion pose = new PoseFusion();
    ShooterSubsystem shooterSubsystem = null;
    Breakbeam breakbeam;
    IndicatorLight light;

    static Pattern obelisk = null;


    // ftcsim stuff - - - - - - - -
    private UdpClientFieldSim client;
    private UdpClientPlot clientPlot;

    //global keyvalue store
    public HashMap<String, String> debugKeyValues = new HashMap<>();

    //misc
    public static boolean isAuto = false;
    public static boolean resetEncoders = false;
    double lastHeading = 0;
    public boolean isMovementDone = false;
    int stageAfterShotOrdinal = 0;
    



    //clocks

    ElapsedTime runtime = new ElapsedTime();
    long lastLoopTime = System.nanoTime();


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
    static boolean[] breakbeamStates = {false, false};



    /**
     * Increments the programStage
     */
    public void incrementStage(int ordinal) {
        nextStage = ordinal;
        programStage = nextStage;
        stageFinished = true;
    }




    @Override
    public void init() {

        drive = new MecanumDrivePinPoint(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        //odo = new Odo(hardwareMap);
        turret = new Turret(hardwareMap);
        clock = new Clock(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(clock, turret, intakeSubsystem);
        breakbeam = new Breakbeam(hardwareMap);
        light = new IndicatorLight(hardwareMap);


        IndicatorLight.setLightWhite();

        initDebugTools();

    }

    @Override
    public void init_loop() {
        long startLoopTime = SystemClock.uptimeMillis();

        long now = System.nanoTime();
        double dt = (now - lastLoopTime) * 1e-9; // seconds
        lastLoopTime = now;

        limelight.updateLimelight();
//
        breakbeamStates[0] = breakbeam.getIntakeBreakbeamStatus();
        breakbeamStates[1] = breakbeam.getTurretBreakbeamStatus();


        obelisk = limelight.updateObelisk(true);
        if(obelisk != null) {
            telemetry.addData("Obelisk", "[%s] [%s] [%s]",
                    obelisk.spindexSlotOne,
                    obelisk.spindexSlotTwo,
                    obelisk.spindexSlotThree);
        }
        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));

    }

    @Override
    public void start() {
        clock.resetClock();
        programStage = 0;
        if(obelisk == null) {obelisk = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);} //uh oh someone set the bot up wrong

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
        isMovementDone = false;
        turret.resetPID();
    }




    public void mainLoop() {
        long startLoopTime = SystemClock.uptimeMillis();

        long now = System.nanoTime();
        double dt = (now - lastLoopTime) * 1e-9; // seconds
        lastLoopTime = now;

        //read everything once and only once per loop

        limelight.updateLimelight();
        turret.updateTurret();
        clock.clockUpdate();
        breakbeamStates[0] = breakbeam.getIntakeBreakbeamStatus();
        breakbeamStates[1] = breakbeam.getTurretBreakbeamStatus();


//
        turret.showAimTelemetry(telemetry);
        breakbeam.displayBreakbeamTelemetry(telemetry);


        telemetry.addData("Superstructure State", currentState);
        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        //addDebugData();

        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }

    /**
     * adds debug telemetry when in debug mode - we don't want this normally bc of loop times
     */
    private void addDebugData()
    {

        for(String k : debugKeyValues.keySet())
        {
            client.sendKeyValue(k, debugKeyValues.get(k));
        }

        client.sendPosition(worldXPosition, worldYPosition, Math.toDegrees(worldAngle_rad));


    }
    private void initDebugTools()
    {
        client = new UdpClientFieldSim("192.168.43.240", 7777);
        clientPlot = new UdpClientPlot("192.168.43.240  ", 7778);

        clientPlot.sendYLimits(SystemClock.uptimeMillis(), 0.5, 0);
        clientPlot.sendYUnits(SystemClock.uptimeMillis() + 1, "PID");
    }










}
