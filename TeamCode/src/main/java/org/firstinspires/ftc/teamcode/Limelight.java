package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight extends RobotMasterPinpoint
{
    public static final int[] obeliskAprilTagIDs = {21, 22, 23};
    public static final int redGoalID = 24;
    public static final int blueGoalID = 20;
    private final Limelight3A limelight;
    private static double distance;
    private static final double CAMERA_HEIGHT_INCHES = 13.53; // (h1)
    private static final double GOAL_APRILTAG_HEIGHT_INCHES = 29.0; // (h2)
    private static final double CAMERA_MOUNTING_ANGLE_DEGREES = 22.0; // (a1)

    private static LLResult currResult = null;
    private static long currLatency = 0;


    public Limelight(HardwareMap hwMap)
    {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
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


            // Convert angles from degrees to radians for Math.tan()
            double a1_radians = Math.toRadians(CAMERA_MOUNTING_ANGLE_DEGREES);
            double a2_radians = Math.toRadians(ty_degrees);

            // Calculate height difference
            double heightDifference = GOAL_APRILTAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

            // Calculate the distance using the formula

            return heightDifference / Math.tan(a1_radians + a2_radians);
        }
        return -1; // Return -1 or another indicator that no tag was found
    }



    public void updateLimelight()
    {
        currResult = limelight.getLatestResult();

        int tagId = getTagId(currResult);

        // Check if the detected tag is a goal
        if (isTagRedGoal(tagId) || isTagBlueGoal(tagId)) {
             distance = getDistanceToTag(currResult);
        }
        currLatency = limelight.getTimeSinceLastUpdate();
    }

    public static LLResult getCurrResult()
    {
        return currResult;
    }
    public static double getDistance() {
        return distance;
    }
    public static Pose3D getPose() {return currResult.getBotpose();}
    public static long getCurrLatency() {return currLatency;}


}
