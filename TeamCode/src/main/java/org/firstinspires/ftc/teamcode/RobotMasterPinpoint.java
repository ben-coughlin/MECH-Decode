package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Locale;

public abstract class RobotMasterPinpoint extends OpMode {
    MecanumDrivePinPoint drive = null;


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public boolean isAuto = false;
    public static boolean resetEncoders = false;


//clocks

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();

    //Lift P controllers

    // Vision for Tensor

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231227_193805.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "TPB",
            "TPR",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //public TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


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

    DigitalChannel frontPixelReceiver;
    DigitalChannel backPixelReceiver;

    public ArrayList<CurvePoint> mirrorPoints(ArrayList<CurvePoint> points) {
        ArrayList<CurvePoint> newPoints = new ArrayList<>();
        for (CurvePoint point : points) {
            newPoints.add(new CurvePoint(-point.x, point.y, point.moveSpeed, point.turnSpeed, point.followDistance, point.pointLength, point.slowDownTurnRadians, point.slowDownTurnRadians));
        }
        return newPoints;
    }

    //Vision vision;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(171.45, 38, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        drive = new MecanumDrivePinPoint(hardwareMap);


        odo.resetPosAndIMU();
    }

    private int pixelData = 0;
    private ArrayList<Boolean> count = new ArrayList<>();
    private int position = 1;

    @Override
    public void init_loop() {
        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        double startLoopTime = SystemClock.uptimeMillis();

        //Pose est for RR
//        PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
//        Pose2d pose = drive.localizer.getPose();

//        worldXPosition = pose.position.x;
//        worldYPosition = pose.position.y;
//        worldAngle_rad = pose.heading.toDouble();

        // pose update for pinpoint
        odo.update();

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        worldXPosition = pos.getX(DistanceUnit.INCH);
        worldYPosition = pos.getY(DistanceUnit.INCH);
        worldAngle_rad = pos.getHeading(AngleUnit.RADIANS);


        // DO NOT CHANGE THIS LINE RR
        //SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);
        // DO NOT CHANGE THIS LINE Pinpoint
        SpeedOmeter.update(odo.getVelY(DistanceUnit.INCH), odo.getVelX(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();


        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("World X", worldXPosition);
        telemetry.addData("World Y", worldYPosition);
        telemetry.addData("World Angle Rad", (worldAngle_rad));
        telemetry.addData("World Angle Deg", Math.toDegrees(worldAngle_rad));

        //       RR Vel Telemetery
//        telemetry.addData("vel x", currentPoseVel.linearVel.x);
//        telemetry.addData("vel y", currentPoseVel.linearVel.y);
//        telemetry.addData("vel heading", currentPoseVel.angVel);

        //  pinpoint tlelemetry
        telemetry.addData("vel x", odo.getVelX(DistanceUnit.INCH));
        telemetry.addData("vel y", odo.getVelY(DistanceUnit.INCH));
        telemetry.addData("vel heading", odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

//        telemetry.addData("x", pose.position.x);
//        telemetry.addData("y", pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));

    }

    @Override
    public void start() {
        programStage = 0;

    }

    @Override
    public void loop() {
        double startLoopTime = SystemClock.uptimeMillis();
        // RR pose update
//        PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
//        Pose2d pose = drive.localizer.getPose();//

        //pinpoint pose update
        odo.update();

        Pose2D pos = odo.getPosition();
        mainAutoLoop();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", odo.getDeviceStatus());
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        worldXPosition = pos.getX(DistanceUnit.INCH);
        worldYPosition = pos.getY(DistanceUnit.INCH);
        worldAngle_rad = pos.getHeading(AngleUnit.RADIANS);


        // DO NOT CHANGE THIS LINE RR
        //SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);
        // DO NOT CHANGE THIS LINE Pinpoint
        SpeedOmeter.update(odo.getVelY(DistanceUnit.INCH), odo.getVelX(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);



        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("World X", worldXPosition);
        telemetry.addData("World Y", worldYPosition);
        telemetry.addData("World Angle Rad", (worldAngle_rad));
        telemetry.addData("World Angle Deg", Math.toDegrees(worldAngle_rad));

        //       RR Vel Telemetery
//        telemetry.addData("vel x", currentPoseVel.linearVel.x);
//        telemetry.addData("vel y", currentPoseVel.linearVel.y);
//        telemetry.addData("vel heading", currentPoseVel.angVel);

        //  pinpoint tlelemetry
        telemetry.addData("vel x", odo.getVelX(DistanceUnit.INCH));
        telemetry.addData("vel y", odo.getVelY(DistanceUnit.INCH));
        telemetry.addData("vel heading", odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

//        telemetry.addData("x", pose.position.x);
//        telemetry.addData("y", pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

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

    public void mainLoop() {
    }


}
