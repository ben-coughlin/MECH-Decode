package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.AutoMaster.pathState;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Pattern;

import java.util.HashMap;

public abstract class RobotMaster extends OpMode {

    //misc
    public static boolean isAuto = false;
    public static boolean isAutoFar = false;
    public static boolean resetEncoders = false;
    public static int programStage = 0;
    static Pattern obelisk = null;
    public static final Pattern inventory = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);
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
    Limelight limelight = null;
    Intake intake = null;
    Turret turret = null;

    Transfer transfer = null;
    ShooterSubsystem shooterSubsystem = null;
    ColorSensor colorSensors = null;
    IndicatorLight indicatorLight = null;
    Kickstand kickstand = null;

    public Follower follower;
    public static Pose startingPose;
    public static String selectedProgram;
    public TelemetryManager telemetryM;


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

        Constants.createFollower(hardwareMap).pathConstraints.setBrakingStrength(2);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, -Math.PI / 2));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        inventory.updatePattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY); //just to make sure
        inventory.setNumBalls(0);

        drive = new MecanumDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(turret, intake, transfer);
        colorSensors = new ColorSensor(hardwareMap);
        indicatorLight = new IndicatorLight(hardwareMap);
        kickstand = new Kickstand(hardwareMap);
        transfer.initTransfer();

    }

    @Override
    public void init_loop() {
        long startLoopTime = SystemClock.uptimeMillis();

        lastLoopTime = System.nanoTime();

        limelight.updateLimelight();
//

        colorSensors.updateDetection();
        obelisk = limelight.updateObelisk(true);
        if (obelisk != null) {
            telemetry.addData("Obelisk", "[%s] [%s] [%s]",
                    obelisk.getLower(),
                    obelisk.getMiddle(),
                    obelisk.getUpper());
        }
        colorSensors.showColorSensorTelemetry(telemetry);
        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));

    }

    @Override
    public void start() {
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

    }


    public void mainLoop() {
        long startLoopTime = SystemClock.uptimeMillis();

        lastLoopTime = System.nanoTime();

        //read everything once and only once per loop

        limelight.updateLimelight();
        turret.updateTurret();
        colorSensors.updateDetection();
        transfer.updateTransfer();

        indicatorLight.setBallCount(inventory.getNumBalls());
        indicatorLight.setFlywheelReady(ShooterSubsystem.isFlywheelReady);
        indicatorLight.setOuttakeRunning(Intake.isOuttakeRunning);
        indicatorLight.setIntakeRunning(Intake.isIntakeRunning);
        indicatorLight.update();


        telemetry.addData("Inventory", "[%s] [%s] [%s]",
                inventory.getLower(),
                inventory.getMiddle(),
                inventory.getUpper());
        turret.showAimTelemetry(telemetry);
        colorSensors.showColorSensorTelemetry(telemetry);
        telemetry.addData("Current Stage", AutoMaster.AutoStage.values()[pathState].name());
        telemetry.addData("superstructure state", pathState);

        telemetry.addData("Superstructure State", currentState);
        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);


        telemetry.update();
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }




}
