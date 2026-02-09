package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.Pattern;
import org.firstinspires.ftc.teamcode.RobotMaster;

import java.util.HashMap;
import java.util.List;

public class Limelight extends RobotMaster
{
    public static final int[] obeliskAprilTagIDs = {21, 22, 23};
    public final HashMap<Integer, Pattern> idToPatternList= new HashMap<>();
    public static int currObeliskAprilTag;
    public static final int redGoalID = 24;
    public static final int blueGoalID = 20;
    private final Limelight3A limelight;
    private static double distance;
    private static final double CAMERA_HEIGHT_INCHES = 13.53; // (h1)
    private static final double GOAL_APRILTAG_HEIGHT_INCHES = 29.0; // (h2)
    private static final double CAMERA_MOUNTING_ANGLE_DEGREES = 22.0; // (a1)

    private static LLResult currResult = null;
    private static long currLatency = 0;
    private static double tx;


    public Limelight(HardwareMap hwMap)
    {
        idToPatternList.put(21, new Pattern(Pattern.Ball.GREEN, Pattern.Ball.PURPLE, Pattern.Ball.PURPLE));
        idToPatternList.put(22, new Pattern(Pattern.Ball.PURPLE, Pattern.Ball.GREEN, Pattern.Ball.PURPLE));
        idToPatternList.put(23, new Pattern(Pattern.Ball.PURPLE, Pattern.Ball.PURPLE, Pattern.Ball.GREEN));

        limelight = hwMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();


    }
    public double getTx()
    {
        return currResult.getTx();
    }



    /**
     * Gets the tag ID from the latest result
     *
     * @param result the most recent LLResult object
     * @return the ID of the tag, or -1 if no tag is detected
     */
    public static int getTagId(LLResult result)
    {
        int id = -1;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags)
        {
            id = tag.getFiducialId(); // the ID number of the tag
            break;
        }

        return id;
    }

    public static boolean isTagRedGoal(int tag)
    {
        return tag == redGoalID;
    }

    public static boolean isTagBlueGoal(int tag)
    {
        return tag == blueGoalID;
    }

    public static boolean isTagObelisk(int tag)
    {
        for (int id : obeliskAprilTagIDs)
        {
            if (tag == id)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Calculates the horizontal distance to the first detected AprilTag using trigonometry.
     * Formula: d = (h2 - h1) / tan(a1 + a2)
     * @param result The latest LLResult from the Limelight.
     * @return The horizontal distance to the AprilTag in inches, or a default value if no tag is seen.
     */
    public double getDistanceToTag(LLResult result) {
        if (result != null && result.isValid()) {
            // Get the vertical angle (ty) to the target from the Limelight. This is 'a2'.
            double ty_degrees = result.getTy();

            // Convert angles from degrees to radians
            double a1_radians = Math.toRadians(CAMERA_MOUNTING_ANGLE_DEGREES);
            double a2_radians = Math.toRadians(ty_degrees);

            double heightDifference = GOAL_APRILTAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

            return heightDifference / Math.tan(a1_radians + a2_radians);
        }
        return -1;
    }



    public void updateLimelight()
    {
        currResult = limelight.getLatestResult();
        tx = currResult.getTx();

        int tagId = getTagId(currResult);

        // Check if the detected tag is a goal
        if (isTagRedGoal(tagId) || isTagBlueGoal(tagId)) {
             distance = getDistanceToTag(currResult);
        }
        currLatency = limelight.getTimeSinceLastUpdate();
    }
    public Pattern updateObelisk(boolean canObeliskBeChanged)
    {
        int tagId = getTagId(currResult);

        if(isTagObelisk(tagId) && canObeliskBeChanged)
        {
            currObeliskAprilTag = tagId;
            return new Pattern(idToPatternList.get(currObeliskAprilTag));
        }

          return idToPatternList.get(currObeliskAprilTag);


    }

    /**
     * Calculate the field position of the AprilTag based on vision data
     * @param robotX - robot's X position on field (inches or meters)
     * @param robotY - robot's Y position on field
     * @param robotHeading - robot's heading in degrees (0 = along positive X axis)
     * @param currentTurretAngle - turret angle relative to robot in degrees (0 = robot forward)
     * @param limelightTx - Limelight horizontal angle error in degrees
     * @param distance - distance to AprilTag from Limelight
     * @return array [tagX, tagY] in field coordinates
     */
    public double[] calculateTagFieldPosition(double robotX, double robotY, double robotHeading,
                                              double currentTurretAngle, double limelightTx,
                                              double distance) {
        // Step 1: Calculate the absolute angle to the target in field coordinates
        // Start with robot heading, add turret angle, add limelight error
        double totalAngleDegrees = robotHeading + currentTurretAngle + limelightTx;
        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

        // Step 2: Project the distance along this angle to get field coordinates
        // Using standard trig: x = distance * cos(angle), y = distance * sin(angle)
        double tagX = robotX + distance * Math.cos(totalAngleRadians);
        double tagY = robotY + distance * Math.sin(totalAngleRadians);

        return new double[]{tagX, tagY};
    }

    /**
     * Convenience overload if you have a Pose2d object (RoadRunner style)
     */
    public double[] calculateTagFieldPosition(Pose2D robotPose, double currentTurretAngle,
                                              double limelightTx, double distance) {
        return calculateTagFieldPosition(
                robotPose.getX(DistanceUnit.INCH),
                robotPose.getY(DistanceUnit.INCH),
                Math.toDegrees(robotPose.getHeading(AngleUnit.RADIANS)),
                currentTurretAngle,
                limelightTx,
                distance
        );
    }


    public static LLResult getCurrResult()
    {
        return currResult;
    }
    public static double getDistance() {
        return distance;
    }
    public Pose3D getPose() {return currResult.getBotpose();}
    public  long getCurrLatency() {return currLatency;}


}
