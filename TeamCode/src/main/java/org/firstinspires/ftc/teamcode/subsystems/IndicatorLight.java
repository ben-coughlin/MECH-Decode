package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotMaster.inventory;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IndicatorLight {
    private final Servo indicatorLight;

    // Subsystem States (Inputs from your OpModes)
    private int ballCount = 0;
    private boolean isIntaking = false;
    private boolean isOuttaking = false;
    private boolean isShotReady = false;

    public IndicatorLight(HardwareMap hwMap) {
        indicatorLight = hwMap.get(Servo.class, "light");
    }

    // Input Setters
    public void setBallCount(int count) { this.ballCount = count; }
    public void setIntakeRunning(boolean running) { this.isIntaking = running; }
    public void setShotReady(boolean ready) { this.isShotReady = ready; }
    public void setOuttakeRunning(boolean running) {this.isOuttaking = running;}

    /**
     * Call this inside your main OpMode loop.
     * Evaluates conditions from LOWEST priority to HIGHEST priority.
     */
    public void update() {

        setBallCount(inventory.getNumBalls());
        setShotReady(ShooterSubsystem.isShotReady);
        setOuttakeRunning(Intake.isOuttakeRunning);
        setIntakeRunning(Intake.isIntakeRunning);

        double colorSelected = getOff();

        if (isIntaking) {
            colorSelected = getBlue();
        }
        if(isOuttaking)
        {
            colorSelected = getViolet();
        }


        if (ballCount < 2) {
            colorSelected = getRed();
        } else if (ballCount == 2) {
            colorSelected = getOrange();
        }
        else if (ballCount == 3) {
            colorSelected = getGreen();
        }

        if(isShotReady)
        {
            colorSelected = getWhite();
        }


        indicatorLight.setPosition(colorSelected);
    }


    // Reference Lookup Positions from https://www.gobilda.com/rgb-indicator-light-pwm-controlled

    public double getRed()    { return 0.277; }
    public double getOrange() { return 0.333; }
    public double getYellow() { return 0.388; }
    public double getGreen()  { return 0.500; }
    public double getAzure()  { return 0.555; }
    public double getBlue()   { return 0.611; }
    public double getIndigo() { return 0.666; }
    public double getViolet() { return 0.722; }
    public double getWhite()  { return 1.000; }
    public double getOff()    { return 0.000; }
}