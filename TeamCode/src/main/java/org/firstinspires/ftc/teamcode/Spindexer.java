package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A smart spindexer subsystem that manages its inventory using the Pattern class.
 * Controlled by a CR Servo and a separate through-bore encoder, using a PID loop for precision.
 */
public class Spindexer {

    // --- Hardware ---
    private final CRServo spindexerServo;
    private final DcMotorEx encoder;
    private final ColorSensor colorSensor;

    // --- Control ---
    private final PIDFController spindexerController = new PIDFController(0.0003, 0, 0.00008, 0.08);

    // --- Constants ---
    private static final double TICKS_PER_REVOLUTION = 8192.0;
    private static final double TICKS_PER_SLOT = TICKS_PER_REVOLUTION / 3.0;
    private static final int POSITION_TOLERANCE = 50;

    // --- State and Inventory ---
    private final Pattern currentInventory;
    private int currentSlot = 0;
    private double targetPositionTicks = 0;
    private boolean isHoldingPosition = true;
    private boolean intakeCycleComplete = true;

    public Spindexer(HardwareMap hwMap, ColorSensor colorSensor) {
        spindexerServo = hwMap.get(CRServo.class, "spindexer");
        encoder = hwMap.get(DcMotorEx.class, "intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.colorSensor = colorSensor;
        this.currentInventory = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);

        // Configure the PID controller
        spindexerController.setReference(0);
        spindexerController.setOutputLimits(-1.0, 1.0);
    }

    public void startIntakeCycle() {
        intakeCycleComplete = false;
    }

    /**
     * Call this continuously in your main OpMode loop.
     * This is the core of the state-based PID control.
     */
    public void update() {
        if (isHoldingPosition) {
            spindexerServo.setPower(0);
            return;
        }

        int currentPosition = encoder.getCurrentPosition();
        double error = targetPositionTicks - currentPosition;

        if (Math.abs(error) <= POSITION_TOLERANCE) {
            stopSpindexer();
            return;
        }


        spindexerServo.setPower(spindexerController.calculatePIDF(currentPosition));
    }

    /**
     * Rotates the spindexer to the next logical slot.
     */
    public void rotateToNextSlot() {
        isHoldingPosition = false;

        currentSlot = (currentSlot + 1) % 3;
        targetPositionTicks = currentSlot * TICKS_PER_SLOT;
        spindexerController.setReference(targetPositionTicks);
        spindexerController.reset();
    }

    /**
     * Immediately stops all spindexer movement and enters a holding state.
     * This is now the ONLY method that should stop the motor.
     */
    public void stopSpindexer() {
        targetPositionTicks = encoder.getCurrentPosition();
        spindexerController.setReference(targetPositionTicks);
        spindexerController.reset();
        spindexerServo.setPower(0);
        isHoldingPosition = true;
    }

    /**
     * DECOUPLED: This method now ONLY updates the inventory.
     * The decision to rotate must be made separately in your TeleOp logic.
     * This prevents race conditions.
     */
    public void recordShotBall() {
        updateSlot(currentSlot, Pattern.Ball.EMPTY);
    }

    /**
     * MODIFIED: Checks for a new ball and rotates, but only runs once per intake cycle.
     */
    public void intakeNewBall() {
        // We can only intake if the spindexer is holding, the slot is empty, AND an intake cycle has been commanded.
        if (!isAtTargetPosition() || getBallInShootingPosition() != Pattern.Ball.EMPTY || intakeCycleComplete) {
            return;
        }

        Pattern.Ball detectedColor = detectBallColor();

        if (detectedColor != Pattern.Ball.EMPTY) {
            updateSlot(currentSlot, detectedColor);
            rotateToNextSlot();
            intakeCycleComplete = true;
        }
    }

    /**
     * Checks if the spindexer is settled at its target position.
     * @return true if the spindexer is stopped and holding its position.
     */
    public boolean isAtTargetPosition() {
        // The system is considered "at target" only when it is in the holding state.
        return isHoldingPosition;
    }

    public Pattern getCurrentInventory() { return currentInventory; }
    public int getCurrentEncoderPosition() { return encoder.getCurrentPosition(); }
    public double getTargetPosition() { return targetPositionTicks; }

    public Pattern.Ball getBallInShootingPosition() {
        switch (currentSlot) {
            case 0: return currentInventory.spindexSlotOne;
            case 1: return currentInventory.spindexSlotTwo;
            case 2: return currentInventory.spindexSlotThree;
            default: return Pattern.Ball.EMPTY;
        }
    }

    private void updateSlot(int slot, Pattern.Ball newBall) {
        if (slot == 0) currentInventory.spindexSlotOne = newBall;
        if (slot == 1) currentInventory.spindexSlotTwo = newBall;
        if (slot == 2) currentInventory.spindexSlotThree = newBall;
    }

    private Pattern.Ball detectBallColor() {
        float[] hsv = colorSensor.getHsvValues();
        float hue = hsv[0];
        float saturation = hsv[1];
        if (hue > 90 && hue < 190 && saturation > 0.5) {
            return Pattern.Ball.GREEN;
        } else if ((hue > 220 && hue < 300) && saturation > 0.5) {
            return Pattern.Ball.PURPLE;
        } else {
            return Pattern.Ball.EMPTY;
        }
    }

    public void showTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Spindexer Tuning ---");
        telemetry.addData("Target Ticks", "%.2f", getTargetPosition());
        telemetry.addData("Current Ticks", getCurrentEncoderPosition());
        double error = getTargetPosition() - getCurrentEncoderPosition();
        telemetry.addData("Position Error", "%.2f", error);
        telemetry.addData("Servo Power", "%.2f", spindexerServo.getPower());
        telemetry.addData("Is Holding Position?", isAtTargetPosition());
        telemetry.addData("Intake Cycle Complete?", intakeCycleComplete);
        telemetry.addLine();
        telemetry.addLine("--- Spindexer Inventory ---");
        telemetry.addData("Current Logical Slot", currentSlot);
        telemetry.addData("Ball in Chamber", getBallInShootingPosition());
        telemetry.addData("Spindexer Inventory", "[%s] [%s] [%s]",
                currentInventory.spindexSlotOne,
                currentInventory.spindexSlotTwo,
                currentInventory.spindexSlotThree);
    }
    public boolean intakeCycleIsComplete() {
        return intakeCycleComplete;
    }

}
