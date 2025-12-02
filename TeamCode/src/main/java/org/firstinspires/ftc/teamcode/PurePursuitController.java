package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * PurePursuitController
 *
 * A small, easy-to-understand Pure Pursuit path follower that produces a desired
 * chassis velocity (vx, vy, omega) for a mecanum robot.
 *
 * Key properties:
 * - Path is a list of Waypoints (x,y) in the same coordinate frame as your robot pose.
 * - Controller computes a lookahead point on the path, converts that point into robot frame,
 *   computes curvature κ = 2*y_r / (L^2), and sets ω = κ * v.
 * - Linear speed v is determined by a simple trapezoidal-like rule:
 *      - If far from goal -> maxSpeed
 *      - If within slowDownRadius -> scales down linearly to minSpeed at goal
 * - Outputs a chassis velocity vector in field-coordinates (vx, vy, omega).
 *
 * Note: The controller itself is stateless between calls except for the path and an
 * optional 'lastIndex' hint used to speed up the lookup on long paths.
 *
 * Usage summary:
 * 1) Construct controller with lookahead and speed limits
 * 2) setPath(...) once per path
 * 3) call update(currentPoseX, currentPoseY, currentHeadingRad) each loop => returns Command
 *
 */
public class PurePursuitController {

    /** A simple point/waypoint on the path. */
    public static class Waypoint {
        public final double x;
        public final double y;
        public Waypoint(double x, double y) { this.x = x; this.y = y; }
    }

    /** Result object returned by update(...) */
    public static class Command {
        /** Field-relative velocities (m/s or arbitrary units consistent with path units) */
        public final double vx;
        public final double vy;
        /** Angular velocity (rad/s) - positive CCW */
        public final double omega;
        /** true when path is complete (within finishTolerance) */
        public final boolean finished;

        public Command(double vx, double vy, double omega, boolean finished) {
            this.vx = vx; this.vy = vy; this.omega = omega; this.finished = finished;
        }
    }

    private final double lookahead;          // lookahead distance (same units as path, e.g., cm)
    private final double maxLinearSpeed;     // max forward speed (units / second)
    private final double minLinearSpeed;     // minimum commanded linear speed when within slowdown radius
    private final double slowDownRadius;     // radius around final waypoint to start slowing
    private final double finishTolerance;    // distance to final waypoint to consider path finished

    // Path storage
    private final ArrayList<Waypoint> path = new ArrayList<>();
    private final ArrayList<Double> segmentLengths = new ArrayList<>(); // cached lengths for performance
    private double pathTotalLength = 0.0;

    private int lastClosestSegment = 0;

    /**
     * Construct a PurePursuitController.
     *
     * @param lookahead meters (or same units as path) - typical: 20..60 (cm) depending on speed
     * @param maxLinearSpeed maximum linear speed (units/sec)
     * @param minLinearSpeed minimum linear speed near goal
     * @param slowDownRadius radius to start slowing (units)
     * @param finishTolerance how close to final waypoint to be considered finished (units)
     */
    public PurePursuitController(double lookahead,
                                 double maxLinearSpeed,
                                 double minLinearSpeed,
                                 double slowDownRadius,
                                 double finishTolerance) {
        if (lookahead <= 0) throw new IllegalArgumentException("lookahead must be > 0");
        this.lookahead = lookahead;
        this.maxLinearSpeed = Math.max(0.0, maxLinearSpeed);
        this.minLinearSpeed = Math.max(0.0, minLinearSpeed);
        this.slowDownRadius = Math.max(0.0, slowDownRadius);
        this.finishTolerance = Math.max(0.0, finishTolerance);
    }

    /**
     * Set a new path (waypoints). Clears previous path.
     * Waypoints should be in the same coordinate frame as robot pose inputs to update().
     *
     * @param newPath list of Waypoint (x,y)
     */
    public void setPath(List<Waypoint> newPath) {
        this.path.clear();
        this.segmentLengths.clear();
        this.pathTotalLength = 0.0;
        this.lastClosestSegment = 0;
        if (newPath == null || newPath.isEmpty()) return;
        this.path.addAll(newPath);
        // compute segment lengths
        for (int i = 0; i < path.size() - 1; i++) {
            Waypoint a = path.get(i);
            Waypoint b = path.get(i + 1);
            double segLen = dist(a.x, a.y, b.x, b.y);
            this.segmentLengths.add(segLen);
            this.pathTotalLength += segLen;
        }
    }

    /**
     * Very small helper for length of path (read-only)
     */
    public int getPathSize() { return path.size(); }

    /**
     * Clear the current path.
     */
    public void clearPath() {
        this.path.clear();
        this.segmentLengths.clear();
        this.pathTotalLength = 0.0;
        this.lastClosestSegment = 0;
    }

    /**
     * MAIN UPDATE — call every control loop iteration.
     *
     * @param robotX field X of robot
     * @param robotY field Y of robot
     * @param robotHeadingRad robot heading in radians (0 points +X, increasing CCW)
     * @return Command containing (vx, vy, omega) in field-coordinates and finished flag
     */
    public Command update(double robotX, double robotY, double robotHeadingRad) {
        if (path.isEmpty()) {
            return new Command(0.0, 0.0, 0.0, true);
        }
        if (path.size() == 1) {
            Waypoint p = path.get(0);
            double d = dist(robotX, robotY, p.x, p.y);
            boolean finished = d <= finishTolerance;
            double speed = finished ? 0.0 : computeLinearSpeed(d);
            // drive straight to single waypoint heading = direction to it
            double angleTo = Math.atan2(p.y - robotY, p.x - robotX);
            // convert to vx/vy (field frame) at requested speed
            double vx = Math.cos(angleTo) * speed;
            double vy = Math.sin(angleTo) * speed;
            // simple omega to point toward path tangent = 0 here
            return new Command(vx, vy, 0.0, finished);
        }

        // If robot near final point => finished
        Waypoint finalPoint = path.get(path.size() - 1);
        double distToGoal = dist(robotX, robotY, finalPoint.x, finalPoint.y);
        if (distToGoal <= finishTolerance) {
            return new Command(0.0, 0.0, 0.0, true);
        }

        // 1) Find the lookahead point on the path (furthest intersection with lookahead circle).
        //    This also advances lastClosestSegment for efficiency.
        boolean found = false;
        double lx = 0.0, ly = 0.0;
        // start searching segments near lastClosestSegment
        int startIdx = Math.max(0, lastClosestSegment);
        int endIdx = path.size() - 2;
        double bestAlong = Double.NEGATIVE_INFINITY; // parameter along path (larger = further along)
        for (int i = startIdx; i <= endIdx; i++) {
            Waypoint a = path.get(i);
            Waypoint b = path.get(i + 1);
            // compute intersections between circle(center robot, radius lookahead) and segment ab
            // parametric segment: P(t)=a + t*(b-a), t in [0,1]
            // solve |P(t) - robot|^2 = lookahead^2
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double fx = a.x - robotX;
            double fy = a.y - robotY;

            double A = dx * dx + dy * dy;
            double B = 2 * (fx * dx + fy * dy);
            double C = fx * fx + fy * fy - lookahead * lookahead;

            double disc = B * B - 4 * A * C;
            if (disc < 0) continue; // no intersection on infinite line
            double sqrtD = Math.sqrt(disc);
            double t1 = (-B - sqrtD) / (2 * A);
            double t2 = (-B + sqrtD) / (2 * A);
            // check t's within [0,1]
            if (t1 >= 0 && t1 <= 1) {
                double px = a.x + t1 * dx;
                double py = a.y + t1 * dy;
                // measure how far along the whole path this intersection is:
                double along = computePathDistanceUpToSegment(i) + t1 * segmentLengths.get(i);
                if (along > bestAlong) {
                    bestAlong = along;
                    lx = px;
                    ly = py;
                    found = true;
                    lastClosestSegment = i;
                }
            }
            if (t2 >= 0 && t2 <= 1) {
                double px = a.x + t2 * dx;
                double py = a.y + t2 * dy;
                double along = computePathDistanceUpToSegment(i) + t2 * segmentLengths.get(i);
                if (along > bestAlong) {
                    bestAlong = along;
                    lx = px;
                    ly = py;
                    found = true;
                    lastClosestSegment = i;
                }
            }
        }

        // If no intersection found (e.g., lookahead too small), fall back to a point ahead on path:
        if (!found) {
            // choose the closest point on path (clip to path) then advance lookahead distance along path
            lastClosestSegment = clamp(lastClosestSegment, 0, path.size() - 2);
            double closestAlong = computeClosestPointAlongPath(robotX, robotY, lastClosestSegment);
            double targetAlong = closestAlong + lookahead;
            // clamp to total length
            targetAlong = Math.min(targetAlong, pathTotalLength);
            // convert targetAlong to an XY point on path
            PointOnPath pop = getPointAtPathDistance(targetAlong);
            lx = pop.x;
            ly = pop.y;
        }

        // 2) Compute curvature: transform lookahead point to robot frame (x_r, y_r)
        // Robot frame: origin at robot, x forward, y left
        double relX =  Math.cos(-robotHeadingRad) * (lx - robotX) - Math.sin(-robotHeadingRad) * (ly - robotY);
        double relY =  Math.sin(-robotHeadingRad) * (lx - robotX) + Math.cos(-robotHeadingRad) * (ly - robotY);
        // distance L (should be approximately lookahead)
        double L = Math.hypot(relX, relY);
        // Avoid division by zero
        if (L < 1e-6) L = 1e-6;

        // Standard pure pursuit curvature: kappa = 2 * y_r / L^2
        double curvature = 2.0 * relY / (L * L);

        // 3) Linear speed selection (simple): scale down when nearing final waypoint
        double linearSpeed = computeLinearSpeed(distToGoal);

        // 4) Convert curvature to angular velocity (omega = curvature * v)
        double omega = curvature * linearSpeed;

        // 5) Convert desired forward speed (robot forward axis) into field-frame vx, vy
        // vx_field = v * cos(heading), vy_field = v * sin(heading)
        // Note: forward direction is robotHeadingRad
        double vxField = Math.cos(robotHeadingRad) * linearSpeed;
        double vyField = Math.sin(robotHeadingRad) * linearSpeed;

        // return command
        return new Command(vxField, vyField, omega, false);
    }

    // -------------------------- Helper functions ------------------------------

    private static double dist(double ax, double ay, double bx, double by) {
        double dx = ax - bx;
        double dy = ay - by;
        return Math.hypot(dx, dy);
    }

    // clamp helper
    private static int clamp(int v, int a, int b) {
        return Math.min(b, Math.max(a, v));
    }

    // compute the linear speed based on distance to final point
    private double computeLinearSpeed(double distanceToGoal) {
        if (distanceToGoal >= slowDownRadius) return maxLinearSpeed;
        // map [0, slowDownRadius] -> [minLinearSpeed, maxLinearSpeed]
        double t = distanceToGoal / Math.max(1e-8, slowDownRadius);
        // linear interpolation
        return minLinearSpeed + t * (maxLinearSpeed - minLinearSpeed);
    }

    // compute the distance (along path) up to the start of segment idx
    private double computePathDistanceUpToSegment(int segmentIdx) {
        if (segmentIdx <= 0) return 0.0;
        double sum = 0.0;
        // this is small, cached hopefully ok; if performance is needed, keep prefix sums
        for (int i = 0; i < segmentIdx && i < segmentLengths.size(); i++) {
            sum += segmentLengths.get(i);
        }
        return sum;
    }

    // small struct for point on path by distance
    private static class PointOnPath {
        double x; double y;
        PointOnPath(double x, double y){ this.x = x; this.y = y; }
    }

    // given a distance along path (0..pathTotalLength), return exact XY by linear interpolation on segments
    private PointOnPath getPointAtPathDistance(double along) {
        if (path.isEmpty()) return new PointOnPath(0, 0);
        if (along <= 0) {
            Waypoint p = path.get(0); return new PointOnPath(p.x, p.y);
        }
        double remaining = along;
        for (int i = 0; i < segmentLengths.size(); i++) {
            double seg = segmentLengths.get(i);
            if (remaining <= seg) {
                Waypoint a = path.get(i);
                Waypoint b = path.get(i + 1);
                double t = seg <= 1e-9 ? 0 : (remaining / seg);
                double x = a.x + (b.x - a.x) * t;
                double y = a.y + (b.y - a.y) * t;
                return new PointOnPath(x, y);
            }
            remaining -= seg;
        }
        // fall back to final point
        Waypoint p = path.get(path.size() - 1);
        return new PointOnPath(p.x, p.y);
    }

    // returns parameter (distance along path from start) of the closest point on path to robot.
    // starts searching from guessSegment for speed.
    private double computeClosestPointAlongPath(double robotX, double robotY, int guessSegment) {
        double bestDist = Double.POSITIVE_INFINITY;
        double bestAlong = 0.0;
        int start = clamp(guessSegment - 2, 0, Math.max(0, path.size() - 2));
        int end = Math.max(0, path.size() - 2);
        double prefix = 0.0;
        for (int i = 0; i < start; i++) prefix += segmentLengths.get(i);

        for (int i = start; i <= end; i++) {
            Waypoint a = path.get(i);
            Waypoint b = path.get(i + 1);
            // project point onto segment
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double segLen2 = dx*dx + dy*dy;
            double t = 0.0;
            if (segLen2 > 0) {
                t = ((robotX - a.x) * dx + (robotY - a.y) * dy) / segLen2;
                t = Math.max(0.0, Math.min(1.0, t));
            }
            double px = a.x + t * dx;
            double py = a.y + t * dy;
            double d = dist(robotX, robotY, px, py);
            if (d < bestDist) {
                bestDist = d;
                bestAlong = prefix + t * segmentLengths.get(i);
                lastClosestSegment = i;
            }
            prefix += segmentLengths.get(i);
        }
        return bestAlong;
    }
}
