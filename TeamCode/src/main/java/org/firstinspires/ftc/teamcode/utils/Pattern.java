package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

public class Pattern {

    private Ball upper;
    private Ball middle;
    private Ball lower;
    private int numBalls;

    public enum Ball {
        GREEN, PURPLE, EMPTY, BALL_NOT_DETECTED;
    }

    public Pattern(@NonNull Ball upper, @NonNull Ball middle, @NonNull Ball lower) {
        this.upper = upper;
        this.middle = middle;
        this.lower = lower;
        this.numBalls = calculateInstantCount();
    }

    public Pattern(Pattern p) {
        this.upper = p.upper;
        this.middle = p.middle;
        this.lower = p.lower;
        this.numBalls = p.numBalls;
    }

    public static Pattern getObeliskPatternFromTag(int tag) {
        switch (tag) {
            case 21: return new Pattern(Ball.GREEN, Ball.PURPLE, Ball.PURPLE);
            case 22: return new Pattern(Ball.PURPLE, Ball.GREEN, Ball.PURPLE);
            case 23: return new Pattern(Ball.PURPLE, Ball.PURPLE, Ball.GREEN);
            default: return new Pattern(Ball.EMPTY, Ball.EMPTY, Ball.EMPTY);
        }
    }

    public void updatePattern(Ball upper, Ball middle, Ball lower) {
        this.upper = upper;
        this.middle = middle;
        this.lower = lower;

        this.numBalls = calculateInstantCount();
    }

    /**
     * Loops through current slots and cleanly maps the total item count
     */
    private int calculateInstantCount() {
        int count = 0;
        if (upper != Ball.EMPTY && upper != Ball.BALL_NOT_DETECTED)  count++;
        if (middle != Ball.EMPTY && middle != Ball.BALL_NOT_DETECTED) count++;
        if (lower != Ball.EMPTY && lower != Ball.BALL_NOT_DETECTED)  count++;
        return count;
    }

    public int getNumBalls() { return numBalls; }
    public void setNumBalls(int numBalls) { this.numBalls = numBalls; }
    public Ball getUpper() { return upper; }
    public void setUpper(Ball upper) { this.upper = upper; }
    public Ball getMiddle() { return middle; }
    public void setMiddle(Ball middle) { this.middle = middle; }
    public Ball getLower() { return lower; }
    public void setLower(Ball lower) { this.lower = lower; }
}