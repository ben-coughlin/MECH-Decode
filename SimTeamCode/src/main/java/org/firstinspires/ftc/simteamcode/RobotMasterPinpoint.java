package org.firstinspires.ftc.simteamcode;

import static org.firstinspires.ftc.simteamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.simteamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.simteamcode.RobotPosition.worldYPosition;

import static org.firstinspires.ftc.simteamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.simteamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.simteamcode.MovementVars.movement_turn;

import java.util.Locale;

import static org.firstinspires.ftc.simteamcode.SimApp.clientSim;

public abstract class RobotMasterPinpoint {
    private static final boolean DEBUG_MOVEMENT = true;

    static class MockGoBildaPinpointDriver {
        private int deviceStatus   = 0;
        private int loopTime       = 0;
        private int xEncoderValue  = 0;
        private int yEncoderValue  = 0;
        private float xPosition    = 0;
        private float yPosition    = 0;
        private float hOrientation = 0;
        private float xVelocity    = 0;
        private float yVelocity    = 0;
        private float hVelocity    = 0;

        private static double SCALE = 100;

        public MockGoBildaPinpointDriver() {
        }

        void update() {
//            xPosition    = xPosition + 5;
//            yPosition    = 0;
//            hOrientation = 0;
//            xVelocity    = 10;
//            yVelocity    = 10;
//            hVelocity    = 10;

//            yPosition    += movement_y * SCALE;
//            xPosition    += movement_x * SCALE;
////            xPosition    += 0;
////            yPosition    += 0;
//            hOrientation -= (float) (movement_turn / 3);
//            hOrientation = (float) (movement_turn / 3);
            //        double fl_power_raw = movement_y - movement_turn + movement_x * 1.5;
            //        double bl_power_raw = movement_y - movement_turn - movement_x * 1.5;
            //        double br_power_raw = movement_y + movement_turn + movement_x * 1.5;
            //        double fr_power_raw = movement_y + movement_turn - movement_x * 1.5;

//            hOrientation = 0;
//
//            xVelocity    = 0;
//            yVelocity    = 0;
//            hVelocity    = 0;

            // ALEJANDRO mecanum
            double x_motor_multiplier = 1;
            double fl_power_raw = movement_y - movement_turn + movement_x * x_motor_multiplier;
            double bl_power_raw = movement_y - movement_turn - movement_x * x_motor_multiplier;
            double br_power_raw = movement_y + movement_turn + movement_x * x_motor_multiplier;
            double fr_power_raw = movement_y + movement_turn - movement_x * x_motor_multiplier;

            //find the maximum of the powers
            double maxRawPower = Math.abs(fl_power_raw);
            if (Math.abs(bl_power_raw) > maxRawPower) {
                maxRawPower = Math.abs(bl_power_raw);
            }
            if (Math.abs(br_power_raw) > maxRawPower) {
                maxRawPower = Math.abs(br_power_raw);
            }
            if (Math.abs(fr_power_raw) > maxRawPower) {
                maxRawPower = Math.abs(fr_power_raw);
            }

            //if the maximum is greater than 1, scale all the powers down to preserve the shape
            double scaleDownAmount = 1.0;
            if (maxRawPower > 1.0) {
                //when max power is multiplied by this ratio, it will be 1.0, and others less
                scaleDownAmount = 1.0 / maxRawPower;
            }
            fl_power_raw *= scaleDownAmount;
            bl_power_raw *= scaleDownAmount;
            br_power_raw *= scaleDownAmount;
            fr_power_raw *= scaleDownAmount;

            // ALEJANDRO: I've switched Vx/Vy to test!!!
            double Vx = (fl_power_raw + fr_power_raw + bl_power_raw + br_power_raw) / 4.0;
            double Vy = (-fl_power_raw + fr_power_raw + bl_power_raw - br_power_raw) / 4.0;
            double omega = (-fl_power_raw + fr_power_raw - bl_power_raw + br_power_raw) / 4.0;

            // original from ChatGPT
//            double Vy = (fl_power_raw + fr_power_raw + bl_power_raw + br_power_raw) / 4.0;
//            double Vx = (-fl_power_raw + fr_power_raw + bl_power_raw - br_power_raw) / 4.0;

            xVelocity    = (float) Vx;
            yVelocity    = (float) Vy;
            hVelocity    = (float) omega;

            // dt = time step in seconds
            float dt = (float) 0.07;
            xPosition += Vx * dt * 1000;
            yPosition += Vy * dt * 1000;
//            hOrientation += omega * dt * 1;
//            hOrientation += omega * dt ;

            hOrientation = (float) Math.toRadians(0);
//            hOrientation = 0;

            // --- BEGIN PRINT STATEMENTS ---
            System.out.println("--- Mecanum Block Update ---");
            System.out.printf(Locale.US, "    Inputs: movement_x: %.3f, movement_y: %.3f, movement_turn: %.3f%n",
                    movement_x, movement_y, Math.toDegrees(movement_turn));
            System.out.printf(Locale.US, "    Raw Powers: FL: %.3f, BL: %.3f, BR: %.3f, FR: %.3f%n",
                    fl_power_raw, bl_power_raw, br_power_raw, fr_power_raw);
            System.out.printf(Locale.US, "    MaxRawPower: %.3f, ScaleDownAmount: %.3f%n",
                    maxRawPower, scaleDownAmount);
            System.out.printf(Locale.US, "    Calculated Velocities (unit/s): Vx: %.3f, Vy: %.3f, Omega: %.3f rad/s%n",
                    Vx, Vy, omega);
            System.out.printf(Locale.US, "    Time Step (dt): %.3f s%n", dt);
            System.out.printf(Locale.US, "    Updated State: xPos (mm): %.3f, yPos (mm): %.3f, hOrient (rad): %.3f, hOrient (deg): %.2f%n",
                    xPosition, yPosition, hOrientation, Math.toDegrees(hOrientation));
            System.out.println("--- End Mecanum Block ---");
            // --- END PRINT STATEMENTS ---
            // ALEJANDRO mecanum END




//
////            // Print the values after they have been updated
//            System.out.println("MockGoBildaPinpointDriver.update():");
//            System.out.println("  xPosition (mm): " + xPosition);
//            System.out.println("  yPosition (mm): " + yPosition);
//            System.out.println("  hOrientation (rad): " + hOrientation);
//            System.out.println("  xVelocity (mm/s): " + xVelocity);
//            System.out.println("  yVelocity (mm/s): " + yVelocity);
//            System.out.println("  hVelocity (rad/s): " + hVelocity);
//            System.out.println("  movement_x : " + movement_x);
//            System.out.println("  movement_y : " + movement_y);
//            System.out.println("  movement_turn : " + movement_turn);

            // Prepare pose data for FtcFieldSimulator
            // FtcFieldSimulator expects X, Y in INCHES and heading in DEGREES
            double xInches = xPosition / 25.4;
            double yInches = yPosition / 25.4;
            double headingDegrees = Math.toDegrees(hOrientation);

            // Ensure headingDegrees is in the range [-180, 180] or [0, 360) as preferred by simulator
            // FtcFieldSimulator seems to handle various ranges, but normalizing is good practice.
            // This maps it to (-180, 180]
//            headingDegrees = ((headingDegrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
//            headingDegrees = (headingDegrees - 90) % 360.0;

            System.out.println("MockGoBildaPinpointDriver.update():");
            System.out.println("  xPosition (mm): " + xPosition);
            System.out.println("  yPosition (mm): " + yPosition);
            System.out.println("  hOrientation (deg): " + Math.toDegrees(hOrientation));
            System.out.println("  xVelocity (mm/s): " + xVelocity);
            System.out.println("  yVelocity (mm/s): " + yVelocity);
            System.out.println("  hVelocity (rad/s): " + hVelocity);
            System.out.println("  movement_x : " + movement_x);
            System.out.println("  movement_y : " + movement_y);
            System.out.println("  movement_turn : " + movement_turn);

            if (DEBUG_MOVEMENT) {
                clientSim.sendPosition(xInches, yInches, headingDegrees);
//                clientSim.sendText(String.format(Locale.US, "x/y(%.1f,%.1f) hOrientation(%.0f)", xInches, yInches, headingDegrees));
//                System.exit(1); // ALEJANDRO

            }
        }

        void recalibrateIMU() {
            System.out.println("MockGoBildaPinpointDriver: recalibrateIMU");
        }
        void resetPosAndIMU() {
            System.out.println("MockGoBildaPinpointDriver: resetPosAndIMU");
        }
        public Pose2D getPosition(){
            return new Pose2D(DistanceUnit.MM,
                    xPosition,
                    yPosition,
                    AngleUnit.RADIANS,
                    hOrientation);

                    //this wraps the hOrientation variable from -180° to +180°
//                    ((hOrientation + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
        }

        public double getVelX(){
            return xVelocity;
        }
        public double getVelX(DistanceUnit distanceUnit){
            return distanceUnit.fromMm(xVelocity);
        }
        public double getVelY(){
            return yVelocity;
        }
        public double getVelY(DistanceUnit distanceUnit){
            return distanceUnit.fromMm(yVelocity);
        }
        public double getHeadingVelocity() {
            return hVelocity;
        }
        public double getHeadingVelocity(UnnormalizedAngleUnit unnormalizedAngleUnit){
            return unnormalizedAngleUnit.fromRadians(hVelocity);
        }
        public float getXOffset(DistanceUnit distanceUnit){
            return (float) distanceUnit.fromMm(0);
        }
        public float getYOffset(DistanceUnit distanceUnit){
            return (float) distanceUnit.fromMm(0);
        }
        public float getYawScalar(){return 0; }

        public Pose2D setPosition(Pose2D pos){
            yPosition    = (float) pos.getY(DistanceUnit.MM);
            xPosition    = (float) pos.getX(DistanceUnit.MM);
            hOrientation = (float) pos.getHeading(AngleUnit.RADIANS);
            return pos;
        }
    }
    MockGoBildaPinpointDriver odo = new MockGoBildaPinpointDriver();

    static class MockRobotCore {
        public void addData(String caption, Object value) {
            System.out.println(caption + ": " + value.toString());
        }
    }
    MockRobotCore telemetry = new MockRobotCore();

    static class MockMecanumDrivePinPoint {
        public void stopAllMovementDirectionBased() {
            System.out.println("MockMecanumDrivePinPoint: stopAllMovementDirectionBased");
        }
        public void applyMovementDirectionBased() {
            System.out.println("MockMecanumDrivePinPoint: applyMovementDirectionBased");
            System.out.println("  movement_x : " + movement_x);
            System.out.println("  movement_y : " + movement_y);
            System.out.println("  movement_turn : " + movement_turn);
        }

    }
    MockMecanumDrivePinPoint drive = new MockMecanumDrivePinPoint();

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

    public void initializeStateVariables() {
        stateStartingX = worldXPosition;
        stateStartingY = worldYPosition;
        stateStartingAngle_rad = worldAngle_rad;
        stateStartTime = SystemClock.uptimeMillis();
        Movement.initCurve();
        stageFinished = false;
    }
    ///////////////////////////////
    public void init() {
        System.out.println("RobotMasterPinpoint: init()");
    }
//    public void init_loop() {
//        System.out.println("RobotMasterPinpoint: init_loop()");
//    }

    public void init_loop() {
        System.out.println("RobotMasterPinpoint: init_loop()");

        double startLoopTime = SystemClock.uptimeMillis();

//        // pose update for pinpoint
//        odo.update();

        // Set initial robot position
        odo.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES, 90));
//        clientSim.sendPosition(60, 0, 90);

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
//        System.exit(1); // ALEJANDRO
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
//        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
//        telemetry.update();


        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("World X", worldXPosition);
        telemetry.addData("World Y", worldYPosition);
        telemetry.addData("World Angle Rad", (worldAngle_rad));
        telemetry.addData("World Angle Deg", Math.toDegrees(worldAngle_rad));


        //  pinpoint tlelemetry
        telemetry.addData("vel x", odo.getVelX(DistanceUnit.INCH));
        telemetry.addData("vel y", odo.getVelY(DistanceUnit.INCH));
        telemetry.addData("vel heading", odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));


        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));

    }



    public void start() {
        System.out.println("RobotMasterPinpoint: start()");
        programStage = 0;
    }
    public void mainLoop() {
        System.out.println("RobotMasterPinpoint: mainLoop()");
    }

    public void loop() {
        double startLoopTime = SystemClock.uptimeMillis();

        //pinpoint pose update
        odo.update();

        Pose2D pos = odo.getPosition();

//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Status", odo.getDeviceStatus());
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

        SpeedOmeter.update(odo.getVelY(DistanceUnit.INCH), odo.getVelX(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("World X", worldXPosition);
        telemetry.addData("World Y", worldYPosition);
        telemetry.addData("World Angle Rad", (worldAngle_rad));
        telemetry.addData("World Angle Deg", Math.toDegrees(worldAngle_rad));

        //  pinpoint tlelemetry
        telemetry.addData("vel x", odo.getVelX(DistanceUnit.INCH));
        telemetry.addData("vel y", odo.getVelY(DistanceUnit.INCH));
        telemetry.addData("vel heading", odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
    }

}

//package org.firstinspires.ftc.simteamcode;
//
//import static org.firstinspires.ftc.simteamcode.RobotPosition.worldAngle_rad;
//import static org.firstinspires.ftc.simteamcode.RobotPosition.worldXPosition;
//import static org.firstinspires.ftc.simteamcode.RobotPosition.worldYPosition;
//
//import android.os.SystemClock;
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.ArrayList;
//import java.util.Locale;
//
//public abstract class RobotMasterPinpoint extends OpMode {
//    MecanumDrivePinPoint drive = null;
//
//
//    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
//
//    public boolean isAuto = false;
//    public static boolean resetEncoders = false;
//
//
////clocks
//
//    ElapsedTime runtime = new ElapsedTime();
//    ElapsedTime waitTimer1 = new ElapsedTime();
//    ElapsedTime waitTimer2 = new ElapsedTime();
//
//    //Lift P controllers
//
//    // Vision for Tensor
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
//    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "model_20231227_193805.tflite";
//    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
//    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//    // Define the labels recognized in the model for TFOD (must be in training order!)
//    private static final String[] LABELS = {
//            "TPB",
//            "TPR",
//    };
//
//    /**
//     * The variable to store our instance of the TensorFlow Object Detection processor.
//     */
//    //public TfodProcessor tfod;
//
//    /**
//     * The variable to store our instance of the vision portal.
//     */
//    private VisionPortal visionPortal;
//
//
//    //////// STATE MACHINE STUFF BELOW DO NOT TOUCH ////////
//    public boolean stageFinished = true;
//    public long stateStartTime = 0;
//
//    public long programStartTime = 0;//time the program starts
//    public static int programStage = 0;
//
//    /**
//     * STATE STARTING VARIABLES
//     */
//    public double stateStartingX = 0;
//    public double stateStartingY = 0;
//    public double stateStartingAngle_rad = 0;
//
//    private final boolean DEBUGGING = false;
//
//    private boolean inDebugState = false;
//
//    //holds the stage we are going to next
//    int nextStage = 0;
//
//    public void nextStage(int ordinal) {
//        nextStage = ordinal;
//        //waits for a if on debug mode
//        if (!DEBUGGING) {
//            incrementStage();
//            inDebugState = false;
//        }
//
//        //go into debug mode
//        if (DEBUGGING) {
//            inDebugState = true;
//        }
//    }
//
//    /**
//     * Increments the programStage
//     */
//    public void nextStage() {
//        nextStage(programStage + 1);
//
//    }
//
//    private void incrementStage() {
//        programStage = nextStage;
//        stageFinished = true;
//    }
//    ///////////////////////////////
//
//    DigitalChannel frontPixelReceiver;
//    DigitalChannel backPixelReceiver;
//
//    public ArrayList<CurvePoint> mirrorPoints(ArrayList<CurvePoint> points) {
//        ArrayList<CurvePoint> newPoints = new ArrayList<>();
//        for (CurvePoint point : points) {
//            newPoints.add(new CurvePoint(-point.x, point.y, point.moveSpeed, point.turnSpeed, point.followDistance, point.pointLength, point.slowDownTurnRadians, point.slowDownTurnRadians));
//        }
//        return newPoints;
//    }
//
//    //Vision vision;
//
//    @Override
//    public void init() {
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(61.0, 165.0, DistanceUnit.MM);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();
//
//        drive = new MecanumDrivePinPoint(hardwareMap);
//
//
//        //initTfod();
///*
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
//                "id",
//                hardwareMap.appContext.getPackageName());
//        vision = new Vision(
//                OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
//                                "Webcam 1"),
//                        cameraMonitorViewId));
//        vision.openCameraDevice();
//        vision.setPipeline(new Vision.AutoVisionPipeline());
//        vision.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//*/
//
//
//
//        // distance sensors
//
//    }
//
//    private int pixelData = 0;
//    private ArrayList<Boolean> count = new ArrayList<>();
//    private int position = 1;
//
//    @Override
//    public void init_loop() {
//        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
//                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
//                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
//                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
//                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
//                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);
//
//        double startLoopTime = SystemClock.uptimeMillis();
//
//        //Pose est for RR
////        PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
////        Pose2d pose = drive.localizer.getPose();
//
////        worldXPosition = pose.position.x;
////        worldYPosition = pose.position.y;
////        worldAngle_rad = pose.heading.toDouble();
//
//        // pose update for pinpoint
//        odo.update();
//
//        Pose2D pos = odo.getPosition();
//        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//
//            /*
//            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
//             */
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
//
//        worldXPosition = pos.getX(DistanceUnit.INCH);
//        worldYPosition = pos.getY(DistanceUnit.INCH);
//        worldAngle_rad = pos.getHeading(AngleUnit.RADIANS);
//
//
//        // DO NOT CHANGE THIS LINE RR
//        //SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);
//        // DO NOT CHANGE THIS LINE Pinpoint
//        SpeedOmeter.update(odo.getVelY(DistanceUnit.INCH), odo.getVelX(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
//        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
//        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
//        telemetry.addData("Heading Scalar", odo.getYawScalar());
//        telemetry.update();
//
//
//        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//
//        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//        telemetry.addData("World X", worldXPosition);
//        telemetry.addData("World Y", worldYPosition);
//        telemetry.addData("World Angle Rad", (worldAngle_rad));
//        telemetry.addData("World Angle Deg", Math.toDegrees(worldAngle_rad));
//
//        //       RR Vel Telemetery
////        telemetry.addData("vel x", currentPoseVel.linearVel.x);
////        telemetry.addData("vel y", currentPoseVel.linearVel.y);
////        telemetry.addData("vel heading", currentPoseVel.angVel);
//
//        //  pinpoint tlelemetry
//        telemetry.addData("vel x", odo.getVelX(DistanceUnit.INCH));
//        telemetry.addData("vel y", odo.getVelY(DistanceUnit.INCH));
//        telemetry.addData("vel heading", odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//
////        telemetry.addData("x", pose.position.x);
////        telemetry.addData("y", pose.position.y);
////        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//
//        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
//
//    }
//
//    @Override
//    public void start() {
//        programStage = 0;
//
//    }
//
//    @Override
//    public void loop() {
//        double startLoopTime = SystemClock.uptimeMillis();
//        // RR pose update
////        PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();
////        Pose2d pose = drive.localizer.getPose();//
//
//        //pinpoint pose update
//        odo.update();
//
//        Pose2D pos = odo.getPosition();
//        mainAutoLoop();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Status", odo.getDeviceStatus());
//        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//
//            /*
//            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
//             */
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
//        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//
//        worldXPosition = pos.getX(DistanceUnit.INCH);
//        worldYPosition = pos.getY(DistanceUnit.INCH);
//        worldAngle_rad = pos.getHeading(AngleUnit.RADIANS);
//
//
//        // DO NOT CHANGE THIS LINE RR
//        //SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);
//        // DO NOT CHANGE THIS LINE Pinpoint
//        SpeedOmeter.update(odo.getVelY(DistanceUnit.INCH), odo.getVelX(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//
//        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//
//
//
//        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//        telemetry.addData("World X", worldXPosition);
//        telemetry.addData("World Y", worldYPosition);
//        telemetry.addData("World Angle Rad", (worldAngle_rad));
//        telemetry.addData("World Angle Deg", Math.toDegrees(worldAngle_rad));
//
//        //       RR Vel Telemetery
////        telemetry.addData("vel x", currentPoseVel.linearVel.x);
////        telemetry.addData("vel y", currentPoseVel.linearVel.y);
////        telemetry.addData("vel heading", currentPoseVel.angVel);
//
//        //  pinpoint tlelemetry
//        telemetry.addData("vel x", odo.getVelX(DistanceUnit.INCH));
//        telemetry.addData("vel y", odo.getVelY(DistanceUnit.INCH));
//        telemetry.addData("vel heading", odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//
////        telemetry.addData("x", pose.position.x);
////        telemetry.addData("y", pose.position.y);
////        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//
//        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
//        telemetry.update();
//        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
//    }
//
//    public void initializeStateVariables() {
//        stateStartingX = worldXPosition;
//        stateStartingY = worldYPosition;
//        stateStartingAngle_rad = worldAngle_rad;
//        stateStartTime = SystemClock.uptimeMillis();
//        Movement.initCurve();
//        stageFinished = false;
//    }
//
//    private void mainAutoLoop() {
//        if (inDebugState) {
//            drive.stopAllMovementDirectionBased();
//            // ControlMovement(); CHANGE THIS
//
//            telemetry.addLine("in debug state");
//            if (gamepad1.a) {
//                incrementStage();
//                inDebugState = false;
//            }
//        } else {
//            mainLoop();
//        }
//    }
//
//    public void mainLoop() {
//    }
//
//
//}
