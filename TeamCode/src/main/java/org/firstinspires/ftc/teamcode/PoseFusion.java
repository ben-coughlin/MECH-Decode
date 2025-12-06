package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * PoseFusion.java
 * <hr>
 * 3-state EKF using velocity-based prediction:
 *  State: [x (in), y (in), heading (rad CCW)]
 * <hr>
 *     Usage
 * <ul>
 *     <li> PoseFusion ekf = new PoseFusion(cameraForwardOffsetInches, tiltDegrees);</li>
 *     <li> ekf.predict(vx_in_per_s, vy_in_per_s, omega_rad_per_s, dt_seconds);</li>
 *     <li> ekf.updateFromLimelight(limePose3D, turretAngleRad, limelightLatencyMs);</li>
 * </ul>

 */
public class PoseFusion {

    // STATE
    // x = [x, y, heading]
    private final double[] x = new double[3]; // x[0]=x(in), x[1]=y(in), x[2]=heading(rad)
    private final double[][] P = new double[3][3]; // covariance

    // Process noise (std dev per second)
    private double qPos = 0.08;      // inches/sec (stddev)
    private double qHeading = 0.015; // radians/sec (stddev)

    // Vision measurement noise (std dev)
    private double rPos = 1.5;       // inches
    private double rHeading = 0.15;  // radians

    private final double tiltRad = Math.toRadians(22); // camera pitch in radians (positive = upward)

    // Safety / rejection params
    private double maxAllowedJumpInches = 8.0; // discard vision if camera jump too large
    private double maxAllowedLatencyMs = 120.0; // latency threshold

    // temporary storage to avoid allocation
    private final double[] z = new double[3];
    private final double[] yInno = new double[3];



    /** Primary constructor. tiltDegrees positive = upward tilt. */
    public PoseFusion() {


        // initialize state and covariance
        x[0] = 0.0; x[1] = 0.0; x[2] = 0.0;
        // small initial uncertainty
        P[0][0] = 4.0; P[0][1] = 0;   P[0][2] = 0;
        P[1][0] = 0;   P[1][1] = 4.0; P[1][2] = 0;
        P[2][0] = 0;   P[2][1] = 0;   P[2][2] = 0.04; // ~0.2 rad std
    }

    // -------------------- Prediction (velocity-based) --------------------

    /**
     * Predict using body-frame velocities.
     * <ul>
     *  <li>vx, vy: robot-frame velocities in inches/sec (forward = vx, left = vy)</li>
     *  <li>omega: angular velocity in rad/sec (continuous, do not normalize)</li>
     *  <li>dt: elapsed time in seconds since last predict</li>
     * </ul>
     *
     */
    public void predict(double vx, double vy, double omega, double dt) {
        if (dt <= 0) return;

        // rotate velocities from robot frame to field frame using current heading
        double heading = x[2];
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double vxField = vx * cosH - vy * sinH;
        double vyField = vx * sinH + vy * cosH;

        // propagate state
        x[0] += vxField * dt;
        x[1] += vyField * dt;
        x[2] = normalizeAngle(x[2] + omega * dt);

        // Process noise Q scaled by dt (assuming qPos/qHeading are stddev per second)
        double[][] Q = new double[3][3];
        Q[0][0] = Math.pow(qPos * dt, 2);
        Q[1][1] = Math.pow(qPos * dt, 2);
        Q[2][2] = Math.pow(qHeading * dt, 2);

        addInPlace(P, Q);
    }




    /**
     * Update from Limelight Pose3D. Assumes limePose3D provides field-frame
     * camera position in meters and yaw in radians (Limelight standard Pose3D).
     * turretAngleRad is the turret yaw (radians) where 0 means camera points forward.
     * limelightLatencyMs is the reported vision latency in milliseconds.
     */
    public void updateFromLimelight(Pose3D limePose3D, double turretAngleRad, double limelightLatencyMs) {
        if (limePose3D == null) return;

        // quick validity check (caller may want to check too)
        if (!Limelight.getCurrResult().isValid()) return;

        // Extract raw camera pose from Limelight
        double rawCamX_m = limePose3D.getPosition().x; // meters
        double rawCamY_m = limePose3D.getPosition().y; // meters
        double rawYawLL = limePose3D.getOrientation().getYaw(AngleUnit.RADIANS); // LL yaw (left-handed)

        // Convert camera yaw from LL (left-handed) to CCW+ radians
        double cameraYaw = -rawYawLL; // now CCW positive

        // Compute robot heading: cameraYaw (world) minus turret yaw
        double robotHeadingMeas = normalizeAngle(cameraYaw - turretAngleRad);

        // Project camera forward offset into XY plane using cos(tilt)
        // Camera geometry
        // physical forward offset
        double cameraForwardOffsetInches = 6.895;
        double effectiveForwardOffsetMeters = (cameraForwardOffsetInches * Math.cos(tiltRad)) * 0.0254;

        // Rotate that offset into field frame using turretAngle (camera faces turretRotation)
        double dx_m = effectiveForwardOffsetMeters * Math.cos(turretAngleRad);
        double dy_m = effectiveForwardOffsetMeters * Math.sin(turretAngleRad);

        // Convert Limelight camera field pose to robot field pose by subtracting rotated offset
        double robotX_m = rawCamX_m - dx_m;
        double robotY_m = rawCamY_m - dy_m;

        // Convert to inches for kinematic state
        double robotX_in = robotX_m * 39.37007874015748; // meters -> inches
        double robotY_in = robotY_m * 39.37007874015748;



        // Jump rejection: ensure vision hasn't teleported
        double lastX = x[0];
        double lastY = x[1];
        double jump = Math.hypot(robotX_in - lastX, robotY_in - lastY);
        if (jump > maxAllowedJumpInches) {
            // skip update
            return;
        }

        // Measurement vector z = [x, y, heading]
        z[0] = robotX_in;
        z[1] = robotY_in;
        z[2] = robotHeadingMeas;

        // Build measurement noise R and inflate based on latency.
        double latencyScale = 1.0 + Math.min(limelightLatencyMs / maxAllowedLatencyMs, 3.0); // up to 4x
        double[][] R = new double[3][3];
        R[0][0] = (rPos * latencyScale) * (rPos * latencyScale);
        R[1][1] = (rPos * latencyScale) * (rPos * latencyScale);
        R[2][2] = (rHeading * latencyScale) * (rHeading * latencyScale);

        // Innovation y = z - x
        yInno[0] = z[0] - x[0];
        yInno[1] = z[1] - x[1];
        yInno[2] = normalizeAngle(z[2] - x[2]);

        double scale = Math.max(0, 1 - Math.abs(limePose3D.getOrientation().getYaw(AngleUnit.RADIANS)) / Math.toRadians(90));
        yInno[2] *= scale;

        // S = P + R
        double[][] S = copyMatrix(P);
        addInPlace(S, R);

        // Compute K = P * inv(S)
        double[][] Sinv = invert3x3(S);
        if (Sinv == null) {
            // matrix singular? skip update
            return;
        }

        double[][] K = multiply(P, Sinv); // 3x3 * 3x3

        // x = x + K * y
        double[] Ky = multiply(K, yInno);
        x[0] += Ky[0];
        x[1] += Ky[1];
        x[2] = normalizeAngle(x[2] + Ky[2]);

        // P = (I - K) P
        double[][] I = identity3();
        double[][] IK = subtract(I, K);
        double[][] newP = multiply(IK, P);
        copyInto(P, newP);
    }

    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.INCH, x[0], x[1], AngleUnit.RADIANS, x[2]);
    }
    public void updateMotionComponents()
    {
        worldXPosition = x[0];
        worldYPosition = x[1];
        worldAngle_rad = x[2];
    }

    // -------------------- Tuning / helpers --------------------

    /** qPos/qHeading are interpreted as stddev per second for velocity-driven predict. */
    public void setProcessNoise(double posStdInchesPerSec, double headingStdRadPerSec) {
        this.qPos = posStdInchesPerSec;
        this.qHeading = headingStdRadPerSec;
    }

    public void setVisionNoise(double posStdInches, double headingStdRad) {
        this.rPos = posStdInches;
        this.rHeading = headingStdRad;
    }

    public void setMaxAllowedJumpInches(double v) { this.maxAllowedJumpInches = v; }
    public void setMaxAllowedLatencyMs(double v) { this.maxAllowedLatencyMs = v; }

    /**
     * Display fused EKF pose and optional odometry/vision info on telemetry.
     *
     * @param telemetry FTC telemetry object
     * @param ekf       Your PoseFusion EKF instance
     * @param vx        Robot field-frame velocity X (in/s)
     * @param vy        Robot field-frame velocity Y (in/s)
     * @param omega     Robot angular velocity (rad/s)
     */
    public void displayPoseTelemetry(Telemetry telemetry, PoseFusion ekf, double vx, double vy, double omega) {
        Pose2D fused = ekf.getPose();
        telemetry.addLine("--- PoseFusion ---");

        // Fused EKF pose
        telemetry.addData("Fused X",  fused.getX(DistanceUnit.INCH));
        telemetry.addData("Fused Y",  fused.getY(DistanceUnit.INCH));
        telemetry.addData("Fused Heading deg",  Math.toDegrees(fused.getHeading(AngleUnit.RADIANS)));

        // Velocities
        telemetry.addData("Vel X",  vx);
        telemetry.addData("Vel Y",  vy);
        telemetry.addData("Omega deg/s",  Math.toDegrees(omega));

    }


    // -------------------- Small matrix library for 3x3 --------------------

    private static double[][] identity3() {
        double[][] m = new double[3][3];
        m[0][0]=1; m[1][1]=1; m[2][2]=1; return m;
    }

    private static void addInPlace(double[][] A, double[][] B) {
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) A[i][j]+=B[i][j];
    }

    private static double[][] copyMatrix(double[][] A) {
        double[][] r = new double[3][3];
        for (int i=0;i<3;i++) System.arraycopy(A[i],0,r[i],0,3);
        return r;
    }

    private static void copyInto(double[][] dest, double[][] src) {
        for (int i=0;i<3;i++) System.arraycopy(src[i],0,dest[i],0,3);
    }

    private static double[][] subtract(double[][] A, double[][] B) {
        double[][] r = new double[3][3];
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) r[i][j] = A[i][j]-B[i][j];
        return r;
    }

    private static double[][] multiply(double[][] A, double[][] B) {
        double[][] r = new double[3][3];
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
            double s=0;
            for (int k=0;k<3;k++) s += A[i][k]*B[k][j];
            r[i][j]=s;
        }
        return r;
    }

    private static double[] multiply(double[][] A, double[] v) {
        double[] r = new double[3];
        for (int i=0;i<3;i++) {
            double s=0;
            for (int j=0;j<3;j++) s += A[i][j]*v[j];
            r[i]=s;
        }
        return r;
    }

    /**
     * Invert a 3x3 matrix using classical adjugate/determinant method.
     * Returns null if matrix is singular.
     */
    private static double[][] invert3x3(double[][] m) {
        double a = m[0][0], b=m[0][1], c=m[0][2];
        double d = m[1][0], e=m[1][1], f=m[1][2];
        double g = m[2][0], h=m[2][1], i=m[2][2];

        double A = e*i - f*h;
        double B = -(d*i - f*g);
        double C = d*h - e*g;
        double D = -(b*i - c*h);
        double E = a*i - c*g;
        double F = -(a*h - b*g);
        double G = b*f - c*e;
        double H = -(a*f - c*d);
        double I = a*e - b*d;

        double det = a*A + b*B + c*C;
        if (Math.abs(det) < 1e-9) return null;
        double invDet = 1.0 / det;

        double[][] adj = new double[3][3];
        adj[0][0]=A; adj[0][1]=D; adj[0][2]=G;
        adj[1][0]=B; adj[1][1]=E; adj[1][2]=H;
        adj[2][0]=C; adj[2][1]=F; adj[2][2]=I;

        double[][] inv = new double[3][3];
        for (int r=0;r<3;r++) for (int c2=0;c2<3;c2++) inv[r][c2] = adj[r][c2] * invDet;
        return inv;
    }

    // -------------------- Utilities --------------------
    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a <= -Math.PI) a += 2*Math.PI;
        return a;
    }
}
