package org.firstinspires.ftc.teamcode.subsystems;



import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.utils.MovementVars.worldYPosition;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

public class Odo
{
    GoBildaPinpointDriver odo;
    Pose2D pos = null;

    public Odo(HardwareMap hwMap)
    {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(178.50, -125.077, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void updateOdo()
    {
        odo.update();
        pos = odo.getPosition();

        worldXPosition = pos.getX(DistanceUnit.INCH);
        worldYPosition = pos.getY(DistanceUnit.INCH);
        worldAngle_rad = pos.getHeading(AngleUnit.RADIANS);

    }



    public void showOdoTelemetry(Telemetry telemetry)
    {
        telemetry.addLine("--- Odo ---");
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Odo Velocity", velocity);
    }

    //showOdoTelemetry and showWorldPositionTelemetry should have the same values but i'll include both just in case
    public void showWorldPositionTelemetry(Telemetry telemetry)
    {
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, Hdeg: %.3f, Hrad: %.3f}", worldXPosition , worldYPosition, Math.toDegrees(worldAngle_rad), worldAngle_rad);
        telemetry.addData("World Position", data);
    }

    public void showRawTelemetry(Telemetry telemetry) {
        String data = String.format(Locale.US, "{X: %d, Y: %d}", odo.getEncoderX(), odo.getEncoderY());
        telemetry.addData("Raw Encoder Values", data);
    }

}
