package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

/**
 * Lightweight sensor fusion filter combining odometry and Limelight pose estimates.
 * Works in 2D (x, y, heading). Uses radians for angles and inches for position by default.
 * This is a complementary filter, not a true Kalman filter, but serves a similar purpose without matrix math.
 */
public class PoseFusion {

    private Pose2D fusedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    private double odomWeight = 0.98;      // trust odometry this much for position
    private double limelightWeight = 1 - odomWeight;

    private double limelightHeadingWeight = 0.5;
    private Pose2D lastValidVisionPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    public PoseFusion() {}



    /**
     * Update odometry and Limelight estimates and produce a fused pose.
     * @param odomPose The robot's current pose from odometry.
     * @param turretAngleRad The turret's current angle in radians relative to the robot's front.
     */
    public void calculateFusedPose(Pose2D odomPose, double turretAngleRad) {
        Pose3D limePose3D = Limelight.getPose();
        long llLatency = Limelight.getCurrLatency();

        final double MAX_ALLOWED_LATENCY_MS = 60.0;
        final double MAX_ALLOWED_JUMP_INCHES = 8.0;

        boolean isVisionDataValid = limePose3D != null &&
                Limelight.getCurrResult().isValid() &&
                llLatency < MAX_ALLOWED_LATENCY_MS;

        if (isVisionDataValid) {

            double rawLimeX = limePose3D.getPosition().x;
            double rawLimeY = limePose3D.getPosition().y;
            double rawLimeYaw = limePose3D.getOrientation().getYaw(AngleUnit.RADIANS);

            // We need to rotate the Limelight's (x, y) measurement back by the turret's angle.
            double sinTurret = Math.sin(turretAngleRad);
            double cosTurret = Math.cos(turretAngleRad);

            // Apply the 2D rotation matrix transformation
            double transformedX_meters = rawLimeX * cosTurret - rawLimeY * sinTurret;
            double transformedY_meters = rawLimeX * sinTurret + rawLimeY * cosTurret;

            // The robot's actual heading is the Limelight's heading plus the turret's angle.
            double transformedHeading_rad = normalizeAngle(rawLimeYaw + turretAngleRad);

            double limeX = transformedX_meters * 39.3701;
            double limeY = transformedY_meters * 39.3701;

            Pose2D limePose2D = new Pose2D(DistanceUnit.INCH, limeX, limeY, AngleUnit.RADIANS, transformedHeading_rad);

            double jumpDist = Math.hypot(limePose2D.getX(DistanceUnit.INCH) - lastValidVisionPose.getX(DistanceUnit.INCH), limePose2D.getY(DistanceUnit.INCH) - lastValidVisionPose.getY(DistanceUnit.INCH));

            if (jumpDist < MAX_ALLOWED_JUMP_INCHES) {
                lastValidVisionPose = limePose2D;

                double latencyPenalty = Math.min(1.0, llLatency / MAX_ALLOWED_LATENCY_MS);
                double adaptiveLimeWeight = limelightWeight * (1.0 - latencyPenalty);
                double adaptiveOdomWeight = 1.0 - adaptiveLimeWeight;

                double fusedX = adaptiveOdomWeight * odomPose.getX(DistanceUnit.INCH) + adaptiveLimeWeight * limePose2D.getX(DistanceUnit.INCH);
                double fusedY = adaptiveOdomWeight * odomPose.getY(DistanceUnit.INCH) + adaptiveLimeWeight * limePose2D.getY(DistanceUnit.INCH);

                double robotHeadingFromVision_rad = normalizeAngle(-limePose3D.getOrientation().getYaw(AngleUnit.RADIANS) + turretAngleRad);
                double headingError = normalizeAngle(robotHeadingFromVision_rad - odomPose.getHeading(AngleUnit.RADIANS));
                double fusedHeading = normalizeAngle(odomPose.getHeading(AngleUnit.RADIANS) + (limelightHeadingWeight * headingError));

                //heading is super inaccurate so we just won't use it lol
                fusedPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.RADIANS, odomPose.getHeading(AngleUnit.RADIANS));

            } else {
                fusedPose = odomPose;
            }
        } else {
            fusedPose = odomPose;
        }
    }

    public void applyFusedPose()
    {
        worldXPosition = fusedPose.getX(DistanceUnit.INCH);
        worldYPosition = fusedPose.getY(DistanceUnit.INCH);
        worldAngle_rad = fusedPose.getHeading(AngleUnit.RADIANS);

    }





    /** Returns the current fused pose. */
    public Pose2D getPoseEstimate() {
        return fusedPose;
    }

    /** Reset the filter's state to a known pose. Call this at the start of autonomous. */
    public void reset(Pose2D startPose) {
        fusedPose = startPose;
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
    public void showPoseFusionTelemetry(Telemetry telemetry)
    {

        telemetry.addData("Fused X", fusedPose.getX(DistanceUnit.INCH));
        telemetry.addData("Fused Y", fusedPose.getY(DistanceUnit.INCH));
        telemetry.addData("Fused Heading", "%.2f rad  |  %.2f deg",
                fusedPose.getHeading(AngleUnit.RADIANS),
                fusedPose.getHeading(AngleUnit.DEGREES));
    }

}
