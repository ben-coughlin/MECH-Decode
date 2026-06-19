package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotMaster.inventory;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IndicatorLight {
    private final Servo indicatorLight;

    // Subsystem States (Inputs from your OpModes)
    private int ballCount = 0;
    private ElapsedTime colorSwitchTimer = new ElapsedTime();

    private boolean isShotReady = false;

    public IndicatorLight(HardwareMap hwMap) {
        colorSwitchTimer.reset();
        indicatorLight = hwMap.get(Servo.class, "light");
    }

    // Input Setters
    public void setBallCount(int count) { this.ballCount = count; }
    public void setShotReady(boolean ready) { this.isShotReady = ready; }
    /**
     * Call this inside your main OpMode loop.
     * Evaluates conditions from LOWEST priority to HIGHEST priority.
     */
    public void update() {
        setBallCount(inventory.getNumBalls());
        setShotReady(ShooterSubsystem.isShotReady);

        double phase = colorSwitchTimer.seconds() % 3.0; // 3-second repeating cycle
        double colorSelected = getOff();

        if (phase < 1.0) {
            if (ballCount < 2)       colorSelected = getRed();
            else if (ballCount == 2) colorSelected = getOrange();
            else colorSelected = getGreen();

        } else if (phase < 2.5) {
            if (isShotReady) colorSelected = getWhite();

        } else {
            if (Intake.isOuttakeRunning)      colorSelected = getViolet();
            else if (Intake.isIntakeRunning)  colorSelected = getBlue();
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