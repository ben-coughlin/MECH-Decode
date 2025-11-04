package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double kP, kI, kD, kF; // Added kF for Feedforward
    private double p_error, i_error, d_error;
    private double setpoint;
    private double last_error;
    private double output_min, output_max;
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
        kF = Kf; // Initialize kF
        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;
        setpoint = 0.0;
        last_error = 0.0;
        output_min = -1.0;
        output_max = 1.0;
        timer = new ElapsedTime();
    }

    /**
     * Overloaded constructor for a standard PID controller (kF defaults to 0).
     */
    public PIDFController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0.0); // Call the main constructor with kF as 0
    }

    public double calculatePIDF(double current_value) {
        // Calculate P
        p_error = setpoint - current_value;

        // Calculate I
        i_error += p_error * timer.seconds();

        // Calculate D
        d_error = (p_error - last_error) / timer.seconds();

        // Reset timer and update last_error
        timer.reset();
        last_error = p_error;

        // Calculate the total output, now including the feedforward term
        double output = (kP * p_error) + (kI * i_error) + (kD * d_error) + (kF * Math.signum(p_error));

        // Clamp the output to the defined min/max
        return Math.max(output_min, Math.min(output, output_max));
    }

    public void setReference(double sp) {
        setpoint = sp;
    }

    public void setOutputLimits(double min, double max) {
        output_min = min;
        output_max = max;
    }

    public void setGains(double Kp, double Ki, double Kd, double Kf) {
        kP = Kp;
        kI = Ki;
        kD = Kd;
        kF = Kf;
    }

    // Overloaded setGains for just PID
    public void setGains(double Kp, double Ki, double Kd) {
        setGains(Kp, Ki, Kd, this.kF); // Keep existing kF
    }

    public void reset() {
        i_error = 0.0;
        last_error = p_error;
        timer.reset();
    }
}
