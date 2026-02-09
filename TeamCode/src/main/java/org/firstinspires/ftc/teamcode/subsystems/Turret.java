package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.RobotMaster;


/**
 * IMPROVED Turret System
 *
 * Key improvements:
 * 1. Kalman filtering for smoother vision data
 * 2. Target prediction using odometry
 * 3. Feedforward for faster response
 * 4. Motion profiling for slew limiting
 * 5. Better compensation strategies
 */
public class Turret {

    // Hardware limits
    private final double TURRET_TICKS_PER_DEGREE = 2.34426;
    private final double TURRET_MIN_LIMIT_TICKS = -400;
    private final double TURRET_MAX_LIMIT_TICKS = 400;
    private static final double FLYWHEEL_TICKS_PER_REVOLUTION = 28;

    // Motion constraints
    private final double TURRET_MAX_VELOCITY = 800; // deg/s - increase if your turret can handle it
    private final double TURRET_MAX_ACCELERATION = 2000; // deg/s^2
    private final double TURRET_SLEW_RATE = 0.15; // Slightly increased from 0.115

    // Hardware
    private final DcMotorEx turret;
    DcMotorEx flywheelMaster;
    DcMotorEx flywheelFollower;
    private final Servo hood;
    private VoltageSensor voltageSensor;

    // State tracking
    private double turretDeg;
    private double turretPower;
    private double lastTurretPower = 0.0;
    private double hoodPos;
    private double targetPower;
    private double currVoltage;
    private boolean isFlywheelOn;
    public static boolean isFlywheelRunning = false;

    // IMPROVED: Kalman filter for vision
    private final KalmanFilter visionFilter;

    // IMPROVED: Target prediction
    private final TargetPredictor targetPredictor;

    // IMPROVED: Feedforward controller
    private final double kV = 0.0015; // Velocity feedforward gain (tune this!)
    private final double kA = 0.0005; // Acceleration feedforward gain (tune this!)

    // PID Controllers - TUNED for faster response
    private final PIDFController autoAimClose = new PIDFController(0.008, 0, 0.002, 0.12);
    private final PIDFController autoAimFar = new PIDFController(0.009, 0.0002, 0.0012, 0.10);
    private final PIDFController autoAimSlow = new PIDFController(0.006, 0.0001, 0.001, 0.0);
    private PIDFController activeAutoAimController;

    // Thresholds
    private static final double GAIN_SCHEDULING_DISTANCE_THRESHOLD = 67.0;
    private static final double VELOCITY_THRESHOLD = 0.1;
    private static final double VISION_DROPOUT_TIMEOUT_MS = 150; // Reduced from 200ms
    private static final double HEADING_COMPENSATION_DURATION_MS = 800; // Reduced from 1000ms
    private static final double AUTO_HEADING_COMPENSATION_DURATION_MS = 150; // Reduced from 200ms
    private static final double RETURN_TO_CENTER_DURATION_MS = 1200; // Reduced from 1400ms
    private static final double ROTATIONAL_COMPENSATION_GAIN = 0.40; // Increased from 0.34

    // Compensation state
    private enum CompensationMode {
        VISION,
        VELOCITY_COMP,
        HEADING_COMP,
        PREDICTIVE,
        RETURN_TO_CENTER,
        IDLE
    }

    private CompensationMode currentMode = CompensationMode.IDLE;
    private final ElapsedTime compensationTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double robotHeadingOnTargetLoss = 0;
    private boolean hasSeenTargetThisSession = false;
    private boolean wasTargetVisibleLastLoop = false;
    private double headingVelocityAtEndOfVeloCompensation = 0;
    private final double HEADING_VELOCITY_COMPENSATION_FACTOR = 0.5; // Increased from 0.4

    // NEW: Vision quality tracking
    private int consecutiveGoodVisionFrames = 0;
    private static final int MIN_GOOD_FRAMES_FOR_LOCK = 3;

    // Lookup table
    private final double[][] launchAngleLookupTable = {
            { 21, .72, 3360}, //inches, servo, rpm
            { 31, .72, 3390},
            {36, .74, 3450},
            {40, .74, .3540},
            {50, .820, .3810},
            {60, .86, 3900},
            {71, .87, 4110},
            {80, .94, 4470},
            {96, .97, 4740},
            {103, .98, 4770},
            {115, .99, 5000}
    };

    public Turret(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        flywheelMaster = hwMap.get(DcMotorEx.class, "flywheelRight");
        flywheelFollower = hwMap.get(DcMotorEx.class, "flywheelLeft");

        hood = hwMap.get(Servo.class, "hood");
        initVoltageSensor(hwMap);

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelFollower.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMaster.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelFollower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMaster.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelMaster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelFollower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMaster.setVelocityPIDFCoefficients(
                45,   // P
                0,    // I
                0,    // D
                14    // F
        );


        // Initialize new components
        visionFilter = new KalmanFilter();
        targetPredictor = new TargetPredictor();

        activeAutoAimController = autoAimClose;
        autoAimFar.setReference(0);
        autoAimClose.setReference(0);
        autoAimSlow.setReference(0);

        loopTimer.reset();
        resetTracking();
    }

    public void updateTurret() {
        turretDeg = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        turretPower = turret.getPower();
        hoodPos = hood.getPosition();
        double currentDistance = Limelight.getDistance();
        currVoltage = voltageSensor.getVoltage();

        hood.setPosition(getServoPositionFromDistance(currentDistance));
        targetPower = getRPMFromDistance(currentDistance);

        if (isFlywheelOn) {
            isFlywheelRunning = true;
            setFlywheelRPM(targetPower);
        } else {
            isFlywheelRunning = false;
            setFlywheelRPM(0);
        }
    }

    /**
     * IMPROVED: Enhanced aiming with Kalman filtering and prediction
     */
    public void aimTurret(boolean hasValidTarget, double limelightError, double manualTurnInput,
                          double distance, double robotHeading, double robotHeadingVelocity,
                          double avgXYVelocity) {

        double dt = Math.max(loopTimer.seconds(), 0.01); // 10ms floor

        loopTimer.reset();

        double calculatedPower;
        boolean useAutoAim = !(Math.abs(manualTurnInput) > 0.05) &&
                (hasValidTarget || hasSeenTargetThisSession);

        if (useAutoAim) {
            double errorToUse;

            if (hasValidTarget) {
                // --- VISION MODE with Kalman filtering ---
                currentMode = CompensationMode.VISION;
                consecutiveGoodVisionFrames++;

                // Normalize vision error
                if (limelightError > 180) limelightError -= 360;
                else if (limelightError < -180) limelightError += 360;

                // IMPROVED: Filter vision data
                double filteredError = visionFilter.update(limelightError, dt);

                double predictedError = targetPredictor.predict(
                        filteredError,
                        robotHeadingVelocity,
                        avgXYVelocity,
                        turretDeg,
                        dt
                );

                // Use predicted error if we have good vision lock
                errorToUse = (consecutiveGoodVisionFrames >= MIN_GOOD_FRAMES_FOR_LOCK) ?
                        predictedError : filteredError;

                // Gain scheduling based on robot state
                double angularVelocity = Math.abs(robotHeadingVelocity);

                if (angularVelocity < VELOCITY_THRESHOLD && avgXYVelocity < VELOCITY_THRESHOLD) {
                    if (activeAutoAimController != autoAimSlow) {
                        switchController(autoAimSlow);
                    }
                } else if (distance < GAIN_SCHEDULING_DISTANCE_THRESHOLD && distance > 0) {
                    if (activeAutoAimController != autoAimClose) {
                        switchController(autoAimClose);
                    }
                } else {
                    if (activeAutoAimController != autoAimFar) {
                        switchController(autoAimFar);
                    }
                }

                compensationTimer.reset();
                hasSeenTargetThisSession = true;

            } else {
                // Target lost - multi-stage compensation
                consecutiveGoodVisionFrames = 0;

                if (wasTargetVisibleLastLoop) {
                    compensationTimer.reset();
                    robotHeadingOnTargetLoss = robotHeading;
                    targetPredictor.onVisionLost(turretDeg, robotHeadingVelocity);
                }

                double timeSinceLost = compensationTimer.milliseconds();

                if (timeSinceLost < VISION_DROPOUT_TIMEOUT_MS) {
                    // --- VELOCITY COMPENSATION ---
                    currentMode = CompensationMode.VELOCITY_COMP;

                    double compensationPower = robotHeadingVelocity * ROTATIONAL_COMPENSATION_GAIN;
                    applyTurretPowerWithFeedforward(compensationPower, robotHeadingVelocity, 0, dt);

                    wasTargetVisibleLastLoop = hasValidTarget;
                    headingVelocityAtEndOfVeloCompensation = robotHeadingVelocity;
                    return;

                } else if (timeSinceLost < VISION_DROPOUT_TIMEOUT_MS +
                        ((RobotMaster.isAuto) ? AUTO_HEADING_COMPENSATION_DURATION_MS :
                                HEADING_COMPENSATION_DURATION_MS)) {
                    // --- PREDICTIVE COMPENSATION using odometry ---
                    currentMode = CompensationMode.PREDICTIVE;

                    // Use target predictor to estimate where target should be
                    errorToUse = targetPredictor.getPredictedError(
                            robotHeading - robotHeadingOnTargetLoss,
                            timeSinceLost / 1000.0
                    );

                    if (activeAutoAimController != autoAimClose) {
                        switchController(autoAimClose);
                    }

                } else if (timeSinceLost < VISION_DROPOUT_TIMEOUT_MS +
                        HEADING_COMPENSATION_DURATION_MS + RETURN_TO_CENTER_DURATION_MS) {
                    // --- RETURN TO CENTER ---
                    currentMode = CompensationMode.RETURN_TO_CENTER;
                    errorToUse = -turretDeg;

                    if (activeAutoAimController != autoAimFar) {
                        switchController(autoAimFar);
                    }

                } else {
                    // --- IDLE ---
                    currentMode = CompensationMode.IDLE;
                    applyTurretPowerWithFeedforward(0.0, 0, 0, dt);
                    wasTargetVisibleLastLoop = hasValidTarget;
                    return;
                }
            }

            // Calculate PID output
            calculatedPower = -activeAutoAimController.calculatePIDF(errorToUse);

            // IMPROVED: Add feedforward for derivative of error
            double errorVelocity = (hasValidTarget) ?
                    visionFilter.getVelocity() : targetPredictor.getErrorVelocity();

            applyTurretPowerWithFeedforward(calculatedPower, errorVelocity, 0, dt);

        } else {
            // Manual mode
            currentMode = CompensationMode.IDLE;
            calculatedPower = manualTurnInput;
            resetControllers();
            applyTurretPower(calculatedPower);
        }

        wasTargetVisibleLastLoop = hasValidTarget;

    }

    /**
     * IMPROVED: Apply power with feedforward compensation
     */
    private void applyTurretPowerWithFeedforward(double pidPower, double targetVelocity,
                                                 double targetAcceleration, double dt) {
        // Feedforward terms
        double velocityFF = kV * targetVelocity;
        double accelerationFF = kA * targetAcceleration;

        double totalPower = pidPower + velocityFF + accelerationFF;

        applyTurretPower(totalPower);
    }

    /**
     * Apply power to turret with slew rate limiting and soft limits
     */
    private void applyTurretPower(double calculatedPower) {
        // Slew rate limiting
        double delta = calculatedPower - lastTurretPower;
        if (Math.abs(delta) > TURRET_SLEW_RATE) {
            calculatedPower = lastTurretPower + Math.signum(delta) * TURRET_SLEW_RATE;
        }
        lastTurretPower = calculatedPower;

        // Soft limits
        int currentPosition = turret.getCurrentPosition();
        if (currentPosition >= TURRET_MAX_LIMIT_TICKS && calculatedPower > 0) {
            turret.setPower(0);
        } else if (currentPosition <= TURRET_MIN_LIMIT_TICKS && calculatedPower < 0) {
            turret.setPower(0);
        } else {
            turret.setPower(calculatedPower);
        }
    }

    /**
     * Switch controllers smoothly
     */
    private void switchController(PIDFController newController) {
        autoAimClose.reset();
        autoAimFar.reset();
        autoAimSlow.reset();
        activeAutoAimController = newController;
    }

    /**
     * Reset all controllers
     */
    private void resetControllers() {
        autoAimClose.reset();
        autoAimFar.reset();
        autoAimSlow.reset();
        visionFilter.reset();
        targetPredictor.reset();
    }

    public void resetTracking() {
        hasSeenTargetThisSession = false;
        currentMode = CompensationMode.IDLE;
        compensationTimer.reset();
        robotHeadingOnTargetLoss = 0;
        wasTargetVisibleLastLoop = false;
        consecutiveGoodVisionFrames = 0;
        visionFilter.reset();
        targetPredictor.reset();
    }

    public String getTrackingModeForTelemetry() {
        switch (currentMode) {
            case VISION:
                return String.format("VISION (lock:%d)", consecutiveGoodVisionFrames);
            case VELOCITY_COMP:
                return String.format("VELOCITY (%.0fms)", compensationTimer.milliseconds());
            case HEADING_COMP:
                return String.format("HEADING (%.0fms)", compensationTimer.milliseconds());
            case PREDICTIVE:
                return String.format("PREDICTIVE (%.0fms)", compensationTimer.milliseconds());
            case RETURN_TO_CENTER:
                return String.format("CENTERING (%.0fms)", compensationTimer.milliseconds());
            case IDLE: return "IDLE";
            default: return "UNKNOWN";
        }
    }

    // [Rest of the methods remain the same: flywheel control, lookup tables, etc.]

    public void turnOnFlywheel() { isFlywheelOn = true; }
    public void turnOffFlywheel() { isFlywheelOn = false; }

    public double getServoPositionFromDistance(double distance) {
        if (distance <= launchAngleLookupTable[0][0]) {
            return launchAngleLookupTable[0][1];
        }
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) {
            return launchAngleLookupTable[launchAngleLookupTable.length - 1][1];
        }

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lowerBound = launchAngleLookupTable[i];
            double[] upperBound = launchAngleLookupTable[i+1];

            if (distance >= lowerBound[0] && distance <= upperBound[0]) {
                double distanceRange = upperBound[0] - lowerBound[0];
                double servoRange = upperBound[1] - lowerBound[1];
                double distanceRatio = (distance - lowerBound[0]) / distanceRange;

                return lowerBound[1] + (distanceRatio * servoRange);
            }
        }
        return 0.5;
    }

    public double getRPMFromDistance(double distance) {
        if (distance <= launchAngleLookupTable[0][0]) {
            return launchAngleLookupTable[0][2];
        }
        if (distance >= launchAngleLookupTable[launchAngleLookupTable.length - 1][0]) {
            return launchAngleLookupTable[launchAngleLookupTable.length - 1][2];
        }

        for (int i = 0; i < launchAngleLookupTable.length - 1; i++) {
            double[] lowerBound = launchAngleLookupTable[i];
            double[] upperBound = launchAngleLookupTable[i+1];

            if (distance >= lowerBound[0] && distance <= upperBound[0]) {
                double distanceRange = upperBound[0] - lowerBound[0];
                double powerRange = upperBound[2] - lowerBound[2];
                double distanceRatio = (distance - lowerBound[0]) / distanceRange;

                return Range.clip((lowerBound[2] + (distanceRatio * powerRange)), 3000, 6000);
            }
        }
        return 5000;
    }

    public void showAimTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Aim ---");
        telemetry.addData("Turret PID Power", turret.getPower());
        telemetry.addData("Turret Degrees", "%.2f", turretDeg);
        telemetry.addData("Distance to Goal", "%.2f inches", Limelight.getDistance());
        telemetry.addData("Filtered Error", "%.2f°", visionFilter.getState());
        telemetry.addData("Predicted Error", "%.2f°", targetPredictor.getPredictedError(0, 0));
        telemetry.addData("Target Power", "%.3f", targetPower);
        telemetry.addData("Turret Tracking Mode", getTrackingModeForTelemetry());


    }

    public double getTurretDeg() { return turretDeg; }
    public double getTurretPower() { return turretPower; }
    public void setTurretPower(double turretPower) { turret.setPower(turretPower); }
    public double getHoodPos() { return hoodPos; }
    public void setHoodPos(double hoodPos) { hood.setPosition(hoodPos); }

    public void setFlywheelRPM(double rpm) {
        double ticksPerSecond = rpm / 60.0 * FLYWHEEL_TICKS_PER_REVOLUTION;

        flywheelMaster.setVelocity(ticksPerSecond);

        // Follower matches power output
        double masterPower = flywheelMaster.getPower();
        flywheelFollower.setPower(masterPower);
    }

    private void initVoltageSensor(HardwareMap hwMap) {
        voltageSensor = hwMap.voltageSensor.iterator().next();
    }

    public void resetPID() {
        resetControllers();
    }

    /**
     * IMPROVED: Simple Kalman filter for vision smoothing
     */
    private static class KalmanFilter {
        private double state = 0;
        private double velocity = 0;
        private double covariance = 1.0;

        private final double processNoise = 0.01;  // Tune this
        private final double measurementNoise = 0.5; // Tune this

        public double update(double measurement, double dt) {
            // Predict
            state = state + velocity * dt;
            covariance = covariance + processNoise;

            // Update
            double kalmanGain = covariance / (covariance + measurementNoise);
            double innovation = measurement - state;
            state = state + kalmanGain * innovation;
            velocity = innovation / dt; // Estimate velocity from innovation
            covariance = (1 - kalmanGain) * covariance;

            return state;
        }

        public double getState() { return state; }
        public double getVelocity() { return velocity; }

        public void reset() {
            state = 0;
            velocity = 0;
            covariance = 1.0;
        }
    }

    /**
     * IMPROVED: Target predictor using odometry
     */
    private static class TargetPredictor {
        private double lastKnownError = 0;
        private double lastKnownTurretAngle = 0;
        private double robotVelocityAtLoss = 0;
        private double errorVelocity = 0;

        public double predict(double currentError, double robotHeadingVelocity,
                              double robotXYVelocity, double turretAngle, double dt) {
            // Simple prediction: assume target is stationary, compensate for robot motion
            double compensation = -robotHeadingVelocity * dt * 0.1; // Tune this
            errorVelocity = (currentError - lastKnownError) / dt;
            lastKnownError = currentError;
            lastKnownTurretAngle = turretAngle;
            return currentError + compensation;
        }

        public void onVisionLost(double turretAngle, double robotVelocity) {
            lastKnownTurretAngle = turretAngle;
            robotVelocityAtLoss = robotVelocity;
        }

        public double getPredictedError(double headingChange, double timeSinceLoss) {
            // Predict where the target should be based on robot motion
            return lastKnownError - headingChange +
                    robotVelocityAtLoss * timeSinceLoss * 0.5;
        }

        public double getErrorVelocity() { return errorVelocity; }

        public void reset() {
            lastKnownError = 0;
            lastKnownTurretAngle = 0;
            robotVelocityAtLoss = 0;
            errorVelocity = 0;
        }
    }
}