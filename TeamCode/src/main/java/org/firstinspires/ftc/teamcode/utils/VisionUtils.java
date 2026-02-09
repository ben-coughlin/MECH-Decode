package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

public class VisionUtils {
    public static final int[] obeliskAprilTagIDs = {21, 22, 23};
    public static final int redGoalID = 24;
    public static final int blueGoalID = 20;




    /**
     * Gets the tag ID from the latest result
     * @param result the most recent LLResult object
     * @return the ID of the tag, or -1 if no tag is detected
     */
    public static int getTagId(LLResult result) {
        int id = -1;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags) {
            id = tag.getFiducialId(); // The ID number of the tag
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
        for(int id : obeliskAprilTagIDs) {
            if (tag == id) {
                return true;
            }
        }
        return false;
    }







}
