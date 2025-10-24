package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight extends RobotMasterPinpoint
{
    public static final int[] obeliskAprilTagIDs = {21, 22, 23};
    public static final int redGoalID = 24;
    public static final int blueGoalID = 20;
    public final Limelight3A limelight;

    private LLResult currResult = null;


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
        for (int id : obeliskAprilTagIDs)
        {
            if (tag == id)
            {
                return true;
            }
        }
        return false;
    }


    public void updateLimelight()
    {
        currResult = limelight.getLatestResult();
    }

    public LLResult getCurrResult()
    {
        return currResult;
    }

    public void setCurrResult(LLResult currResult)
    {
        this.currResult = currResult;
    }


}
