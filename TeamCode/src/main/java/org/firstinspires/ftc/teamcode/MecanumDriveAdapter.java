package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;

/**
 * MecanumDriveAdapter
 *
 * Small, stateless helper to convert field-relative chassis velocity (vx, vy, omega)
 * into four wheel powers for a standard mecanum robot (X drive mixing).
 *
 * Convention:
 * - vx: forward speed (positive forward)
 * - vy: left speed (positive left)
 * - omega: CCW angular velocity (positive CCW)
 *
 * Wheel order returned: [frontLeft, frontRight, backLeft, backRight]
 */
public class MecanumDriveAdapter {

    /**
     * Convert speeds into wheel powers.
     *
     * @param vx forward speed (units)
     * @param vy leftward speed (units)
     * @param omega angular velocity (rad/sec) scaled to wheel-power units already
     * @return array of 4 wheel powers [fl, fr, bl, br] normalized to max abs 1
     */
    public static double[] toWheelPowers(double vx, double vy, double omega) {


        double cos = Math.cos(-worldAngle_rad);
        double sin = Math.sin(-worldAngle_rad);
        double robotVx = cos * vx - sin * vy;  //
        double robotVy = sin * vx + cos * vy;

        double fl = robotVx + robotVy + omega;
        double fr = robotVx - robotVy - omega;
        double bl = robotVx - robotVy + omega;
        double br = robotVx + robotVy - omega;

        // normalize
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }
        return new double[]{fl, fr, bl, br};
    }

    public static void toMovementComponents(double vx, double vy, double omega)
    {
        double cos = Math.cos(-worldAngle_rad);
        double sin = Math.sin(-worldAngle_rad);
        double robotVx = cos * vx - sin * vy;
        double robotVy = sin * vx + cos * vy;

        MovementVars.movement_x = robotVx;
        MovementVars.movement_y = robotVy;
        MovementVars.movement_turn = omega;
    }

}

