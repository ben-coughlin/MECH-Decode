package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoMaster.pathState;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.HashMap;

public abstract class RobotMasterPinpoint extends OpMode {

    //hardware - - - - - -
    MecanumDrivePinPoint drive = null;
    Odo odo = null;
    Limelight limelight = null;
    IntakeSubsystem intakeSubsystem = null;
    Turret turret = null;
    Clock clock = null;
    PoseFusion pose = new PoseFusion();
    ShooterSubsystem shooterSubsystem = null;
    Breakbeam breakbeam;
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

    public void doodooballs(int shootOrdinal, int ordinal) {
        nextStage = shootOrdinal;
        incrementStage();
        stageAfterShotOrdinal = ordinal;
    }

    /**
     * Increments the programStage
     */
    public void ishouldremovetheselater(int ordinal) {
        nextStage = ordinal;
        incrementStage();
    }

    private void incrementStage() {
        programStage = nextStage;
        stageFinished = true;
    }



    @Override
    public void init() {

        drive = new MecanumDrivePinPoint(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        odo = new Odo(hardwareMap);
        turret = new Turret(hardwareMap);
        clock = new Clock(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(clock, turret, intakeSubsystem);
        breakbeam = new Breakbeam(hardwareMap);

        initDebugTools();

    }

    @Override
    public void init_loop() {
        long startLoopTime = SystemClock.uptimeMillis();

        long now = System.nanoTime();
        double dt = (now - lastLoopTime) * 1e-9; // seconds
        lastLoopTime = now;

        limelight.updateLimelight();
        odo.updateOdo();
        pose.predict(odo.getVelocityComponents()[0], odo.getVelocityComponents()[1], (odo.getHeading() - lastHeading) / dt, dt);
        pose.updateFromLimelight(limelight.getPose(), Math.toRadians(turret.getTurretDeg()), limelight.getCurrLatency());
        pose.updateMotionComponents();
        lastHeading = odo.getHeading();


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
        programStage = 0;
        obelisk = limelight.updateObelisk(false);
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
        Log.i("DEBUG 2", "stateStartingX:" + stateStartingX + " stateStartingY:" + stateStartingY + " stateStartingAngle_deg:" + Math.toDegrees(stateStartingAngle_rad));
    }




    public void mainLoop() {
        long startLoopTime = SystemClock.uptimeMillis();

        long now = System.nanoTime();
        double dt = (now - lastLoopTime) * 1e-9; // seconds
        lastLoopTime = now;

        //read everything once and only once per loop
        limelight.updateLimelight();
        odo.updateOdo();
        turret.updateTurret();
        clock.clockUpdate();


        pose.predict(odo.getVelocityComponents()[0], odo.getVelocityComponents()[1], (odo.getHeading() - lastHeading) / dt, dt);
        pose.updateFromLimelight(limelight.getPose(), Math.toRadians(turret.getTurretDeg()), limelight.getCurrLatency());
        pose.updateMotionComponents();
        lastHeading = odo.getHeading();
        //pose.displayPoseTelemetry(telemetry, pose, odo.getVelocityComponents()[0], odo.getVelocityComponents()[1], (odo.getHeading() - lastHeading) / dt);
        odo.showOdoTelemetry(telemetry);
        turret.showAimTelemetry(telemetry);
        breakbeam.displayBreakbeamTelemetry(telemetry);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

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
