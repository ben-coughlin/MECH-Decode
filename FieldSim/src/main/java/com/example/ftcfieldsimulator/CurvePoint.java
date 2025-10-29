package com.example.ftcfieldsimulator;

import java.util.Objects;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    public double pointLength;


    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance, double slowDownTurnRadians, double slowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }


    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint nextPoint) {
        x = nextPoint.x;
        y = nextPoint.y;
        moveSpeed = nextPoint.moveSpeed;
        turnSpeed = nextPoint.turnSpeed;
        followDistance = nextPoint.followDistance;
        slowDownTurnRadians = nextPoint.slowDownTurnRadians;
        slowDownTurnAmount = nextPoint.slowDownTurnAmount;
        pointLength = nextPoint.pointLength;

    }

    public PointDouble toPoint(){
        return new PointDouble(x,y);
    }

    public void setPoint(PointDouble p){
        x = p.x;
        y = p.y;
    }

    @Override
    public boolean equals(Object o) {
        // This is a standard, robust way to check for equality.
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        CurvePoint that = (CurvePoint) o;

        // Compare all the fields that define if two CurvePoints are the same.
        return Double.compare(that.x, x) == 0 &&
                Double.compare(that.y, y) == 0 &&
                Double.compare(that.moveSpeed, moveSpeed) == 0 &&
                Double.compare(that.turnSpeed, turnSpeed) == 0 &&
                Double.compare(that.followDistance, followDistance) == 0 &&
                Double.compare(that.pointLength, pointLength) == 0 &&
                Double.compare(that.slowDownTurnRadians, slowDownTurnRadians) == 0 &&
                Double.compare(that.slowDownTurnAmount, slowDownTurnAmount) == 0;
    }

    @Override
    public int hashCode() {
        // This is required whenever you override equals().
        // It generates a unique number based on the object's values.
        return Objects.hash(x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount);
    }
}
