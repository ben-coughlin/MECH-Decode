package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Lightweight sensor fusion filter combining odometry and Limelight pose estimates.
 * Works in 2D (x, y, heading). Uses radians for angles and inches for position by default.
 * This is a complementary filter, not a true Kalman filter, but serves a similar purpose without matrix math.
 */
public class PoseFusion {

    private Pose2D fusedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0); // Default units are inches and radians
    private Pose2D lastOdomPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    private long lastTime = -1; 

    private double odomWeight = 0.98;      // trust odometry this much for position
    private double limelightWeight = 1 - odomWeight;

    private double limelightHeadingWeight = 0.5;


    private Pose2D lastValidVisionPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    public PoseFusion() {}

    /** Update odometry and Limelight estimates and produce a fused pose. */
    public void update(Pose2D odomPose) {
        if (lastTime == -1) { // Handle the first run
            lastTime = System.nanoTime();
            lastOdomPose = odomPose;
            fusedPose = odomPose;
            return;
        }

        double dx = odomPose.getX(DistanceUnit.INCH) - lastOdomPose.getX(DistanceUnit.INCH);
        double dy = odomPose.getY(DistanceUnit.INCH) - lastOdomPose.getY(DistanceUnit.INCH);
        double dTheta = normalizeAngle(odomPose.getHeading(AngleUnit.RADIANS) - lastOdomPose.getHeading(AngleUnit.RADIANS));

        Pose2D predictedPose = new Pose2D(
                DistanceUnit.INCH, fusedPose.getX(DistanceUnit.INCH) + dx,
                fusedPose.getY(DistanceUnit.INCH) + dy, AngleUnit.RADIANS,
                normalizeAngle(fusedPose.getHeading(AngleUnit.RADIANS) + dTheta)
        );

        lastOdomPose = odomPose;
        fusedPose = predictedPose; // Assume prediction is correct until a vision update proves otherwise.

        //check to make sure vision data is valid and makes sense
        Pose3D limePose3D = Limelight.getPose();
        long llLatency = Limelight.getCurrLatency();

        final double MAX_ALLOWED_LATENCY_MS = 60.0;
        final double MAX_ALLOWED_JUMP_INCHES = 6.0;

        boolean isVisionDataValid = limePose3D != null &&
                Limelight.getCurrResult().isValid() &&
                llLatency < MAX_ALLOWED_LATENCY_MS;

        if (isVisionDataValid) {
            // Convert Limelight's meters-based Pose3D to an inches-based Pose2D
            double limeX = limePose3D.getPosition().x * 39.3701; // meters → inches
            double limeY = limePose3D.getPosition().y * 39.3701;
            double limeHeading = limePose3D.getOrientation().getYaw(AngleUnit.RADIANS);
            Pose2D limePose2D = new Pose2D(DistanceUnit.INCH, limeX, limeY, AngleUnit.RADIANS, limeHeading);

            double jumpDist = Math.hypot(limePose2D.getX(DistanceUnit.INCH) - lastValidVisionPose.getX(DistanceUnit.INCH), limePose2D.getY(DistanceUnit.INCH) - lastValidVisionPose.getY(DistanceUnit.INCH));

            if (jumpDist < MAX_ALLOWED_JUMP_INCHES) {
                lastValidVisionPose = limePose2D;

                //adapt limelight weight based on latency
                double latencyPenalty = Math.min(1.0, llLatency / MAX_ALLOWED_LATENCY_MS);
                double adaptiveLimeWeight = limelightWeight * (1.0 - latencyPenalty);
                double adaptiveOdomWeight = 1.0 - adaptiveLimeWeight;

                //actually fuse the x & y positions using the adaptive weights
                double fusedX = adaptiveOdomWeight * predictedPose.getX(DistanceUnit.INCH) + adaptiveLimeWeight * limePose2D.getX(DistanceUnit.INCH);
                double fusedY = adaptiveOdomWeight * predictedPose.getY(DistanceUnit.INCH) + adaptiveLimeWeight * limePose2D.getY(DistanceUnit.INCH);

                // fuse heading separately because ll heading is a lot more accurate than ll cartesian position
                double headingError = normalizeAngle(limePose2D.getHeading(AngleUnit.RADIANS) - predictedPose.getHeading(AngleUnit.RADIANS));
                double fusedHeading = normalizeAngle(predictedPose.getHeading(AngleUnit.RADIANS) + (limelightHeadingWeight * headingError));

                fusedPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.RADIANS, fusedHeading);
            }
            // If the jump is too large, we do nothing and trust the odometry prediction from step 1.
        }
        // If vision is not valid, we do nothing and trust the odometry prediction from step 1.
    }

    /** Returns the current fused pose. */
    public Pose2D getPoseEstimate() {
        return fusedPose;
    }

    /** Reset the filter's state to a known pose. Call this at the start of autonomous. */
    public void reset(Pose2D startPose) {
        fusedPose = startPose;
        lastOdomPose = startPose;
        lastTime = -1;
    }

    /** Normalize angle to [-π, π] */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /** Adjust relative trust between odometry and Limelight */
    public void setWeights(double odomWeight, double limeWeight, double limeHeadingWeight) {
        this.odomWeight = odomWeight;
        this.limelightWeight = limeWeight;
        this.limelightHeadingWeight = limeHeadingWeight;
    }
}
