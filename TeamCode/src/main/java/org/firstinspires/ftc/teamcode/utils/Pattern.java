package org.firstinspires.ftc.teamcode.utils;



import androidx.annotation.NonNull;

public class Pattern
{

    private Ball upper;
    private Ball middle;
    private Ball lower;
    private int numBalls;



    public enum Ball
    {
        GREEN,
        PURPLE,
        EMPTY,
        BALL_NOT_DETECTED;
    }

    /**
     * Creates a Pattern using three Ball objects
     *
     * @param upper   Ball object in spindex slot one
     * @param middle   Ball object in spindex slot two
     * @param lower Ball object in spindex slot three
     */

    public Pattern(@NonNull Ball upper, @NonNull Ball middle, @NonNull Ball lower)
    {
        this.upper = upper;
        this.middle = middle;
        this.lower = lower;
    }

    /**
     * Creates a Pattern using another Pattern object
     *
     * @param p Pattern object that is then split into balls
     */
    public Pattern(Pattern p)
    {
        this.upper = p.upper;
        this.middle = p.middle;
        this.lower = p.lower;
        this.numBalls = p.numBalls;
    }


    /**
     * Creates the Obelisk pattern from a tag ID
     *
     * @param tag ID of the tag
     * @return Pattern containing the balls in the correct order, returns a pattern of empty balls if the tag is not an obelisk
     */
    public static Pattern getObeliskPatternFromTag(int tag)
    {

        switch (tag)
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

    public void updatePattern(Ball upper, Ball middle, Ball lower)
    {
        this.upper = upper;
        this.middle = middle;
        this.lower = lower;

        countBalls();
    }

    public void countBalls()
    {
        if ((upper != Ball.EMPTY)) {
            numBalls++;
        } else {
            numBalls--;
        }
        if ((middle != Ball.EMPTY)) {
            numBalls++;
        } else {
            numBalls--;
        }
        if ((lower != Ball.EMPTY)) {
            numBalls++;
        } else {
            numBalls--;
        }
        
    }

    public int getNumBalls() {
        return numBalls;
    }

    public void setNumBalls(int numBalls) {
        this.numBalls = numBalls;
    }

    public Ball getUpper() {
        return upper;
    }

    public void setUpper(Ball upper) {
        this.upper = upper;
    }

    public Ball getMiddle() {
        return middle;
    }

    public void setMiddle(Ball middle) {
        this.middle = middle;
    }

    public Ball getLower() {
        return lower;
    }

    public void setLower(Ball lower) {
        this.lower = lower;
    }
}
