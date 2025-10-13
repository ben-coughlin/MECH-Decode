package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {
    private final double kP, kI, kD;
    private double reference = 0;
    private double integralSum = 0;
    private double lastError;

    private double outputMin = -1;
    private double outputMax = 1.0;
    private double integralMin = -1;
    private double integralMax = 1.0;
    private boolean isFirstLoop = true;
    public double[] pidConstants = new double[3];

    private final ElapsedTime loop = new ElapsedTime();

    /**
     *
     * @param kp proportional constant; controls how quickly the system responds to errors
     * @param ki integral constant; controls how quickly the system responds to accumulated errors
     * @param kd derivative constant; controls how the system uses the rate of change of error to predict future error
     */
    public PIDController(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;

        pidConstants[0] = 0;
        pidConstants[1] = 0;
        pidConstants[2] = 0;

        loop.reset();

    }

    /**
     * @param reference
     * Sets the reference for the error calculation - initialized to zero by default
     */
    public void setReference(double reference) {
        this.reference = reference;

    }

    /**
     * @param outputMin: minimum power the pid loop can output
     * @param outputMax: maximum power
     *                 initialized to -1, 1 by default
     */
    public void setOutputLimits(double outputMin, double outputMax) {
        this.outputMin = outputMin;
        this.outputMax = outputMax;

    }

    /**
     * @param integralMin initialized to -1
     * @param integralMax initialized to 1
     * @note these set the integral limits to prevent the integral term from getting too large
     */
    public void setIntegralLimits(double integralMin, double integralMax) {
        this.integralMin = integralMin;
        this.integralMax = integralMax;
    }

    /**
     * resets the integral sum to 0 and the error to zero - use this for changes in target/direction
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        isFirstLoop = true;
        loop.reset();
    }

    /**
     *
     * @param currentValue whatever current value that we are applying pid to is
     * @return  the adjusted output
     */
    public double calculatePID(double currentValue) {
        double loopTime = loop.milliseconds();
        loop.reset();
        double error = reference - currentValue;

        double proportional = kP * error;

        integralSum += error * loopTime;
        integralSum = clamp(integralSum, integralMin, integralMax);

        double integral = kI * integralSum;

        double derivative = 0.0;

        if (!isFirstLoop) {
            derivative = kD * (error - lastError) / loopTime;
        }
        lastError = error;
        isFirstLoop = false;

        updatePIDConstants(proportional, integral, derivative);

        double output = proportional + integral + derivative;


        return clamp(output, outputMin, outputMax);

    }

    /**
     * updates the values of the error so they can be read for debugging
     * @param proportional current proportional error
     * @param integral current integral error
     * @param derivative current derivative error
     */
   public void updatePIDConstants(double proportional, double integral, double derivative)
   {
       pidConstants[0] = proportional;
       pidConstants[1] = integral;
       pidConstants[2] = derivative;

   }

   public double getProportional()
   {
       return pidConstants[0];
   }
    public double getIntegral()
    {
        return pidConstants[1];
    }
    public double getDerivative()
    {
        return pidConstants[2];
    }

    /**
     * returns a double between min or max, will return min if value < min and max if value > max
     * @param value input
     * @param min minimum output
     * @param max max output
     * @return value between min/max inclusive
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }








}
