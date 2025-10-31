package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A smart spindexer subsystem that manages its inventory using the Pattern class.
 * Controlled by a CR Servo and a separate through-bore encoder, using a PID loop for precision.
 */
public class Spindexer {

    // --- Hardware ---
    private final CRServo spindexerServo;
    private final DcMotorEx encoder;
    private final ColorSensor colorSensor;



    private final PIDController pidController = new PIDController(0.005, 0, 0.0001); //todo: tune
    private static final double TICKS_PER_REVOLUTION = 8192;
    private static final double TICKS_PER_SLOT = 2500; //todo: tune
    private static final int POSITION_TOLERANCE = 25; //todo: tune



    // --- State and Inventory ---
    private final Pattern currentInventory;
    private int currentSlot = 0;
    private double targetPositionTicks = 0;

    public Spindexer(HardwareMap hwMap, ColorSensor colorSensor) {
        spindexerServo = hwMap.get(CRServo.class, "spindexer");
        encoder = hwMap.get(DcMotorEx.class, "encoderMotorName"); //todo: remember what port this is in

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.colorSensor = colorSensor;
        this.currentInventory = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);

        // Configure the PID controller
        pidController.setReference(0);
    }

    /**
     * Call this continuously in your main OpMode loop.
     * This is the core of the PID closed-loop control.
     */
    public void update() {
        int currentPosition = encoder.getCurrentPosition();


        double pidPower = pidController.calculatePID(currentPosition);

        double error = targetPositionTicks - currentPosition;
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            spindexerServo.setPower(0);
            return; // Exit if we're close enough - no jitter here
        }

        spindexerServo.setPower(pidPower);
    }

    /**
     * Rotates the spindexer to the next logical slot.
     */
    public void rotateToNextSlot() {
        currentSlot = (currentSlot + 1) % 3;
        targetPositionTicks = currentSlot * TICKS_PER_SLOT;
        // Set the new target for the PID controller
        pidController.setReference(targetPositionTicks);
    }



    /**
     * Call this when a ball is shot. It marks the current slot as EMPTY.
     */
    public void recordShotPixel() {
        updateSlot(currentSlot, Pattern.Ball.EMPTY);
    }

    /**
     * Call this when intaking. It detects the ball color, adds it to the current slot,
     * and then rotates to the next slot.
     */
    public void intakeNewPixel() {
        if (getPixelInShootingPosition() != Pattern.Ball.EMPTY) return; // Don't overwrite

        Pattern.Ball detectedColor = detectPixelColor();

        if (detectedColor != Pattern.Ball.EMPTY) {
            updateSlot(currentSlot, detectedColor);
            rotateToNextSlot();
        }
    }

    /**
     * Checks if the spindexer has reached its target position.
     * @return true if the spindexer is at its target.
     */
    public boolean isAtTargetPosition() {
        return Math.abs(targetPositionTicks - encoder.getCurrentPosition()) <= POSITION_TOLERANCE;
    }

    // --- Getters for Telemetry and Logic ---
    public Pattern getCurrentInventory() { return currentInventory; }
    public int getCurrentEncoderPosition() { return encoder.getCurrentPosition(); }
    public double getTargetPosition() { return targetPositionTicks; }

    public Pattern.Ball getPixelInShootingPosition() {
        switch (currentSlot) {
            case 0: return currentInventory.spindexSlotOne;
            case 1: return currentInventory.spindexSlotTwo;
            case 2: return currentInventory.spindexSlotThree;
            default: return Pattern.Ball.EMPTY;
        }
    }

    // --- Private Helper Methods ---
    private void updateSlot(int slot, Pattern.Ball newBall) {
        if (slot == 0) currentInventory.spindexSlotOne = newBall;
        if (slot == 1) currentInventory.spindexSlotTwo = newBall;
        if (slot == 2) currentInventory.spindexSlotThree = newBall;
    }

    private Pattern.Ball detectPixelColor() {
        // HSV is generally more reliable for color detection under varying light conditions.
        // Hue (H): The color itself (0-360 degrees).
        // Saturation (S): The intensity of the color (0-1). 0 is grayscale, 1 is pure color.
        // Value (V): The brightness of the color (0-1). 0 is black, 1 is bright.

        float[] hsv = colorSensor.getHsvValues();
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        //todo: tune hsv vals

        // Green is typically around 120 on the hue scale.
        if (hue > 90 && hue < 150 && saturation > 0.5 && value > 0.1) {
            return Pattern.Ball.GREEN;
        // Purple is around 270. We check for a wide range around it.
        } else if ((hue > 240 && hue < 300) && saturation > 0.5 && value > 0.1) {
            return Pattern.Ball.PURPLE;
        } else {
            return Pattern.Ball.EMPTY;
        }
    }
}

