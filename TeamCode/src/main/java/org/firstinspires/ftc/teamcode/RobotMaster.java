package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Breakbeam;
import org.firstinspires.ftc.teamcode.subsystems.Clock;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Pattern;

import java.util.HashMap;

public abstract class RobotMaster extends OpMode {

    //misc
    public static boolean isAuto = false;
    public static boolean resetEncoders = false;
    public static int programStage = 0;
    static Pattern obelisk = null;
    static boolean[] breakbeamStates = {false, false};
    //global keyvalue store
    public HashMap<String, String> debugKeyValues = new HashMap<>();
    public boolean isMovementDone = false;
    /// ///// STATE MACHINE STUFF BELOW DO NOT TOUCH ////////
    public boolean stageFinished = true;
    public long stateStartTime = 0;
    public String currentState;
    /**
     * STATE STARTING VARIABLES
     */
    public double stateStartingX = 0;
    public double stateStartingY = 0;
    public double stateStartingAngle_rad = 0;
    //hardware - - - - - -
    protected MecanumDrive drive = null;
    //  Odo odo = null;
    Limelight limelight = null;
    IntakeSubsystem intakeSubsystem = null;
    Turret turret = null;
    Clock clock = null;


    //clocks
    ShooterSubsystem shooterSubsystem = null;
    Breakbeam breakbeam;


    protected ElapsedTime runtime = new ElapsedTime();
    long lastLoopTime = System.nanoTime();
    //holds the stage we are going to next
    int nextStage = 0;


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

        drive = new MecanumDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        //odo = new Odo(hardwareMap);
        turret = new Turret(hardwareMap);
        clock = new Clock(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(clock, turret, intakeSubsystem);
        breakbeam = new Breakbeam(hardwareMap);
        IndicatorLight.initLight(hardwareMap);


        IndicatorLight.setLightWhite();


    }

    @Override
    public void init_loop() {
        long startLoopTime = SystemClock.uptimeMillis();

        lastLoopTime = System.nanoTime();

        limelight.updateLimelight();
//
        breakbeamStates[0] = breakbeam.getIntakeBreakbeamStatus();
        breakbeamStates[1] = breakbeam.getTurretBreakbeamStatus();


        obelisk = limelight.updateObelisk(true);
        if (obelisk != null) {
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
        if (obelisk == null) {
            obelisk = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);
        } //uh oh someone set the bot up wrong

    }

    @Override
    public void stop() {
        drive.hardStopMotors();
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
        stageFinished = false;
        isMovementDone = false;
        turret.resetPID();
    }


    public void mainLoop() {
        long startLoopTime = SystemClock.uptimeMillis();

        lastLoopTime = System.nanoTime();

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


        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }




}
