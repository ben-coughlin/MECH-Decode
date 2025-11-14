package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A smart spindexer subsystem that uses a CR Servo with an external encoder and PIDF control.
 * Handles automatic rotation, color-based detection, and inventory management.
 */
public class Spindexer {

    // --- Hardware ---
    private final CRServo spindexerServo;
    private final DcMotorEx encoder;
    private final ColorSensor colorSensor;

    // --- Control ---
    private final PIDFController pid = new PIDFController(0.0002, 0, 0.00008, 0.08);

    // --- Constants ---
    private static final double TICKS_PER_REV = 8192.0;
    private static final double TICKS_PER_SLOT = TICKS_PER_REV / 3.0;
    private static final int POSITION_TOLERANCE = 30;

    // --- State ---
    private final Pattern inventory;
    private int currentSlot = 0;
    private double targetTicks = 0;
    private boolean holdingPosition = true;
    private boolean intakeCycleActive = false;
    private boolean ballDetectedThisCycle = false;

    public Spindexer(HardwareMap hwMap, ColorSensor colorSensor) {
        this.spindexerServo = hwMap.get(CRServo.class, "spindexer");
        this.encoder = hwMap.get(DcMotorEx.class, "intake");
        this.colorSensor = colorSensor;

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        inventory = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);

        pid.setReference(0);
        pid.setOutputLimits(-1.0, 1.0);
    }


    public void update() {
        double currentPos = encoder.getCurrentPosition();
        double error = targetTicks - currentPos;

        if (Math.abs(error) <= POSITION_TOLERANCE) {
            stopAndHold();
        } else {
            double output = pid.calculatePIDF(currentPos);
            spindexerServo.setPower(output);
            holdingPosition = false;
        }
    }


    public void startIntakeCycle() {
        if (!intakeCycleActive) {
            intakeCycleActive = true;
            ballDetectedThisCycle = false;
        }
    }

    public void intakeNewBall() {
        if (!IntakeSubsystem.isIntakeRunning|| !isAtTargetPosition() || ballDetectedThisCycle) return;

        Pattern.Ball detected = detectBallColor();

        if (detected != Pattern.Ball.EMPTY) {
            // Update the current slot with the detected ball color
            updateSlot(currentSlot, detected);
            ballDetectedThisCycle = true;

            int targetSlot = getPreferredSlotForColor(detected);

            if (currentSlot != targetSlot) {
                rotateToSlot(targetSlot);
            }

            intakeCycleActive = false;
        }
    }


    public boolean intakeCycleIsComplete() {
        return !intakeCycleActive;
    }

    public void recordShotBall()
    {

        updateSlot(currentSlot, Pattern.Ball.EMPTY);
        rotateToNextFullSlot();
    }
    public void rotateToNextFullSlot() {
        Pattern.Ball[] slots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree
        };

        //
        for (int i = 0; i <= 2; i++) {
            int slotToCheck = (currentSlot + i) % 3;

            if (slots[slotToCheck] != Pattern.Ball.EMPTY) {
                rotateToSlot(slotToCheck);
                return;
            }
        }

    }
    public void rotateToNextEmptySlot() {
        Pattern.Ball[] slots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree
        };

        //
        for (int i = 0; i <= 2; i++) {
            int slotToCheck = (currentSlot + i) % 3;

            if (slots[slotToCheck] == Pattern.Ball.EMPTY) {
                rotateToSlot(slotToCheck);
                return;
            }
        }

    }




    private void rotateToNextSlot() {
        holdingPosition = false;
        currentSlot = (currentSlot + 1) % 3;
        targetTicks = currentSlot * TICKS_PER_SLOT;
        pid.setReference(targetTicks);
        pid.reset();
    }


    private void stopAndHold() {
        targetTicks = encoder.getCurrentPosition();
        pid.setReference(targetTicks);
        pid.reset();
        spindexerServo.setPower(0);
        holdingPosition = true;
    }

    public boolean isAtTargetPosition() {
        return holdingPosition;
    }

    public Pattern.Ball getBallInShootingPosition() {
        switch (currentSlot) {
            case 0:
                return inventory.spindexSlotOne;
            case 1:
                return inventory.spindexSlotTwo;
            case 2:
                return inventory.spindexSlotThree;
            default:
                return Pattern.Ball.EMPTY;
        }
    }

    public void updateSlot(int slot, Pattern.Ball newBall) {
        switch (slot) {
            case 0:
                inventory.spindexSlotOne = newBall;
                break;
            case 1:
                inventory.spindexSlotTwo = newBall;
                break;
            case 2:
                inventory.spindexSlotThree = newBall;
                break;
        }
    }

    private Pattern.Ball detectBallColor() {
        float[] hsv = colorSensor.getHsvValues();
        float hue = hsv[0];
        float sat = hsv[1];

        if (hue > 90 && hue < 190 && sat > 0.5) {
            return Pattern.Ball.GREEN;
        } else if (hue > 220 && hue < 300 && sat > 0.5) {
            return Pattern.Ball.PURPLE;
        } else {
            return Pattern.Ball.EMPTY;
        }
    }

    private void rotateToSlot(int slot) {
        holdingPosition = false;
        currentSlot = slot;
        targetTicks = slot * TICKS_PER_SLOT;
        pid.setReference(targetTicks);
        pid.reset();
    }

    private int getPreferredSlotForColor(Pattern.Ball color) {
        switch (color) {
            case GREEN:
                if (inventory.spindexSlotOne == Pattern.Ball.EMPTY) return 0;
                else if (inventory.spindexSlotTwo == Pattern.Ball.EMPTY) return 1;
                else return 2;
            case PURPLE:
                if (inventory.spindexSlotThree == Pattern.Ball.EMPTY) return 2;
                else if (inventory.spindexSlotTwo == Pattern.Ball.EMPTY) return 1;
                else return 0;
            default:
                // fallback: first available empty slot
                if (inventory.spindexSlotOne == Pattern.Ball.EMPTY) return 0;
                if (inventory.spindexSlotTwo == Pattern.Ball.EMPTY) return 1;
                if (inventory.spindexSlotThree == Pattern.Ball.EMPTY) return 2;
                return currentSlot;
        }
    }


    public void showTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Spindexer ---");
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Target Pos", "%.1f", targetTicks);
        telemetry.addData("Encoder Pos", "%.1f", encoder.getCurrentPosition());
        telemetry.addData("At Target?", isAtTargetPosition());
        telemetry.addData("Holding", holdingPosition);
        telemetry.addData("Intake Active", intakeCycleActive);
        telemetry.addData("Detected This Cycle", ballDetectedThisCycle);
        telemetry.addData("Inventory", "[%s] [%s] [%s]",
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree);
    }
}
