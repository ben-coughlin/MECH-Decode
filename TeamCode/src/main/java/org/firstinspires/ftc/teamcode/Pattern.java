package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public class Pattern {

    Ball spindexSlotOne;
    Ball spindexSlotTwo;
    Ball spindexSlotThree;


    public enum Ball
    {
        GREEN,
        PURPLE,
        EMPTY
    }

    /**
     * Creates a Pattern using three Ball objects
     * @param spindexSlotOne Ball object in spindex slot one
     * @param spindexSlotTwo Ball object in spindex slot two
     * @param spindexSlotThree Ball object in spindex slot three
     */

    public Pattern(@NonNull Ball spindexSlotOne, @NonNull Ball spindexSlotTwo, @NonNull Ball spindexSlotThree)
    {
        this.spindexSlotOne = spindexSlotOne;
        this.spindexSlotTwo = spindexSlotTwo;
        this.spindexSlotThree = spindexSlotThree;
    }

    /**
     * Creates a Pattern using another Pattern object
     * @param p Pattern object that is then split into balls
     */
    public Pattern(Pattern p)
    {
        this.spindexSlotOne = p.spindexSlotOne;
        this.spindexSlotTwo = p.spindexSlotTwo;
        this.spindexSlotThree = p.spindexSlotThree;
    }


    /**
     * Creates the Obelisk pattern from a tag ID
     * @param tag ID of the tag
     * @return Pattern containing the balls in the correct order, returns a pattern of empty balls if the tag is not an obelisk
     */
    public static Pattern getObeliskPatternFromTag(int tag)
    {

        switch(tag)
        {
            case 21:
            {
                return new Pattern(Ball.GREEN, Ball.PURPLE, Ball.PURPLE);
            }
            case 22:
            {
                return new Pattern(Ball.PURPLE, Ball.GREEN, Ball.PURPLE);
            }
            case 23:
            {
                return new Pattern(Ball.PURPLE, Ball.PURPLE, Ball.GREEN);
            }
            default:
                return new Pattern(Ball.EMPTY, Ball.EMPTY, Ball.EMPTY);
        }
    }

    public void updatePattern(Ball spindexSlotOne, Ball spindexSlotTwo, Ball spindexSlotThree)
    {
        this.spindexSlotOne = spindexSlotOne;
        this.spindexSlotTwo = spindexSlotTwo;
        this.spindexSlotThree = spindexSlotThree;
    }




}
