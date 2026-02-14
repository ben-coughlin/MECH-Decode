package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double kP, kI, kD, kF;
    private double pError, iError, dError;
    private double setpoint;
    private double prevError;
    private double outputMin, outputMax;
    private final ElapsedTime timer;

    /**
     * A PIDF Controller to calculate the necessary output for a mechanism.
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feedforward gain - provides a constant power boost to overcome static friction
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        kP = Kp;
        kI = Ki;
        kD = Kd;
        kF = Kf;
        pError = 0.0;
        iError = 0.0;
        dError = 0.0;
        setpoint = 0.0;
        prevError = 0.0;
        outputMin = -1.0;
        outputMax = 1.0;
        timer = new ElapsedTime();
    }

    /**
     * Overloaded constructor for a standard PID controller (kF defaults to 0).
     */
    public PIDFController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0.0);
    }

    public double calculatePIDF(double current_value) {
        // Calculate P
        pError = setpoint - current_value;

        // Calculate I
        iError += pError * timer.seconds();

        // Calculate D
        dError = (pError - prevError) / timer.seconds();


        timer.reset();
        prevError = pError;


        double output = (kP * pError) + (kI * iError) + (kD * dError) + (kF * Math.signum(pError));

        return Math.max(outputMin, Math.min(output, outputMax));
    }

    public void setReference(double sp) {
        setpoint = sp;
    }

    public void setOutputLimits(double min, double max) {
        outputMin = min;
        outputMax = max;
    }

    public void setGains(double Kp, double Ki, double Kd, double Kf) {
        kP = Kp;
        kI = Ki;
        kD = Kd;
        kF = Kf;
    }

    // Overloaded setGains for just PID
    public void setGains(double Kp, double Ki, double Kd) {
        setGains(Kp, Ki, Kd, this.kF);
    }

    public void setF(double kF) { this.kF = kF; }

    public double getF() { return kF; }



    public void reset() {
        iError = 0.0;
        prevError = pError;
        timer.reset();
    }
}
