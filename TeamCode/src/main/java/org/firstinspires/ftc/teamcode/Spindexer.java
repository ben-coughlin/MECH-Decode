package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotMasterPinpoint.obelisk;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

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
    private VoltageSensor voltageSensor;

    // --- Control ---
    private final PIDFController pid = new PIDFController(0.000173, 0.00002, 0.000091, 0.093);

    // --- Constants ---
    private static final double TICKS_PER_REV = 8192.0;
    private static final double TICKS_PER_SLOT = (TICKS_PER_REV / 3.0);
    private static final int POSITION_TOLERANCE = 50;
    // --- State ---
    private final Pattern inventory;
    private int currentSlot = 0;
    private double targetTicks = 0;
    private boolean holdingPosition = true;
    private boolean intakeCycleActive = false;
    private boolean ballDetectedThisCycle = false;
    private int nudgeOffset = 0;
    private double lastOutput = 0;            // last power set to servo
    private static double MAX_SLEW = 0.0258;
    private Pattern.Ball detected = null;

    public Spindexer(HardwareMap hwMap, ColorSensor colorSensor) {
        this.spindexerServo = hwMap.get(CRServo.class, "spindexer");
        this.encoder = hwMap.get(DcMotorEx.class, "intake");
        this.colorSensor = colorSensor;

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        initVoltageSensor(hwMap);

        inventory = new Pattern(Pattern.Ball.EMPTY, Pattern.Ball.EMPTY, Pattern.Ball.EMPTY);

        pid.setReference(0);
        pid.setOutputLimits(-1.0, 1.0);
    }


    public void update() {
        double currentPos = encoder.getCurrentPosition();
        double error = targetTicks - currentPos;
        detected = colorSensor.detectBallColor();
        MAX_SLEW += (14.5 - voltageSensor.getVoltage()) / 0.1 * 0.00013;


        // PIDF calculation
        double output = pid.calculatePIDF(currentPos);
        output = Range.clip(output, -1, 1);

        // --- Slew rate limiting ---
        double delta = output - lastOutput;
        if (delta > MAX_SLEW) delta = MAX_SLEW;
        if (delta < -MAX_SLEW) delta = -MAX_SLEW;
        output = lastOutput + delta;

        double distance = targetTicks - currentPos;

        double scale = Range.clip(Math.abs(distance) / 200.0, 0, 1); // 200 ticks = full power
        output *= scale;
        // Send to motor
        spindexerServo.setPower(output);

        // Update last output for next cycle
        lastOutput = output;

        // Holding position logic
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            spindexerServo.setPower(0);
            holdingPosition = true;
        } else {
            holdingPosition = false;
        }


    }


    public void startIntakeCycle() {
        if (!intakeCycleActive) {
            intakeCycleActive = true;
            ballDetectedThisCycle = false;
            Log.i("Spindexer", "Intake cycle started");
        }
    }

    public void intakeNewBall() {
        if (!IntakeSubsystem.isIntakeRunning) return;

        // Only process if we are holding position (rotation finished)
        if (!isAtTargetPosition()) return;

        // Only allow one ball per cycle
        if (ballDetectedThisCycle) return;
        Log.i("Spindexer", " inside intake new ball Detected Ball: " + detected + "color booleans " + colorSensor.detectBallColor());


        if (detected == Pattern.Ball.PURPLE || detected == Pattern.Ball.GREEN) {
            updateSlot(currentSlot, detected);
            ballDetectedThisCycle = true;
            Log.i("Spindexer", "updated slot: current (" + currentSlot + ") + detected (" + detected + ")");


            rotateToNextEmptySlot();

            intakeCycleActive = false;
        }
    }



    public boolean intakeCycleIsComplete() {
        Log.i("Spindexer", "intake cycle is complete");
        return !intakeCycleActive;

    }
    public void shotPrep()
    {
        if(getBallInShootingPosition() == Pattern.Ball.EMPTY)
        {
            Log.i("Spindexer", "current slot is empty, rotating to next full slot to shoot");
            rotateToNextFullSlot();
        }

    }


    /**
     *
     * @return true if the ball was shot, rotates automatically. false otherwise - does nothing
     */
    public boolean recordShotBall(boolean useObeliskSlot)
    {
        Log.i("Spindexer", "recording shot ball");

        if(detected == Pattern.Ball.EMPTY)
        {
            Log.i("Spindexer", "shot was successs yahoo yippe");
            updateSlot(currentSlot, Pattern.Ball.EMPTY);
            if(useObeliskSlot)
            {
                rotateToNextSlotInPattern();
            }
            else
            {
                rotateToNextFullSlot();
            }
        }
        Log.i("Spindexer", "shot failed aw shucks");

        return false;

    }
    public void rotateToNextFullSlot() {
        Pattern.Ball[] slots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree
        };

        //
        for (int i = 1; i <= 3; i++) {
            int slotToCheck = (currentSlot + i) % 3;

            if (slots[slotToCheck] != Pattern.Ball.EMPTY && slotToCheck != currentSlot) {
                Log.i("Spindexer", "rotating to next full slot" + slotToCheck);
                rotateToSlot(slotToCheck);
                return;
            }
        }

    }
    public void rotateToNextEmptySlot() {
        Pattern.Ball[] slots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree,
        };

        for (int i = 1; i <= 3; i++) { // start from 1 to avoid current slot
            int slotToCheck = (currentSlot + i) % 3;

            if (slots[slotToCheck] == Pattern.Ball.EMPTY) {
                Log.i("Spindexer", "rotating to next empty slot " + slotToCheck);
                rotateToSlot(slotToCheck);
                ballDetectedThisCycle = false;
                return;
            }
        }

        Log.i("Spindexer", "no empty slot found, staying at currentSlot " + currentSlot);
    }


    public void chooseShotColor(Pattern.Ball desiredColor)
    {
        if(desiredColor == Pattern.Ball.GREEN)
        {
            rotateToNextGreenSlot();
        }
        else if(desiredColor == Pattern.Ball.PURPLE)
        {
            rotateToNextPurpleSlot();
        }
    }
    public void rotateToNextGreenSlot() {
        Pattern.Ball[] slots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree
        };

        //
        for (int i = 1; i <= 3; i++) {
            int slotToCheck = (currentSlot + i) % 3;

            if (slots[slotToCheck] == Pattern.Ball.GREEN && slotToCheck != currentSlot) {
                Log.i("Spindexer", "rotating to next green slot" + slotToCheck);
                rotateToSlot(slotToCheck);
                return;
            }
        }

    }
    public void rotateToNextPurpleSlot() {
        Pattern.Ball[] slots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree
        };

        //
        for (int i = 1; i <= 3; i++) {
            int slotToCheck = (currentSlot + i) % 3;

            if (slots[slotToCheck] == Pattern.Ball.PURPLE && slotToCheck != currentSlot) {
                Log.i("Spindexer", "rotating to next purple slot" + slotToCheck);
                rotateToSlot(slotToCheck);
                return;
            }
        }

    }

    private void rotateToNextSlot() {
        holdingPosition = false;
        currentSlot = (currentSlot + 1) % 3;
        targetTicks = currentSlot * TICKS_PER_SLOT + nudgeOffset;
        pid.setReference(targetTicks);
        pid.reset();
    }


    private void stopAndHold() {
        targetTicks = encoder.getCurrentPosition() + nudgeOffset;
        pid.setReference(targetTicks);
        pid.reset();
        spindexerServo.setPower(0);
        holdingPosition = true;
        Log.i("Spindexer", "stopping and holding");
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
    private void rotateToSlot(int slot) {

        Log.i("Spindexer", "rotating to slot " + slot);

        holdingPosition = false;
        currentSlot = slot;

        double baseTarget = slot * TICKS_PER_SLOT + nudgeOffset;

        int current = encoder.getCurrentPosition();  // your encoder position

        // Generate candidate positions (same slot, different revolutions)
        double target1 = baseTarget;
        double target2 = baseTarget + TICKS_PER_REV;   // one full rev ahead
        double target3 = baseTarget - TICKS_PER_REV;   // one full rev behind

        // Pick the closest
        double bestTarget = target1;
        if (Math.abs(current - target2) < Math.abs(current - bestTarget)) bestTarget = target2;
        if (Math.abs(current - target3) < Math.abs(current - bestTarget)) bestTarget = target3;

        targetTicks = bestTarget;

        pid.reset();
        pid.setReference(targetTicks);
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
    public void rotateToNextSlotInPattern() {
        Pattern.Ball[] invSlots = {
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree
        };
        Pattern.Ball[] obeliskSlots = {
                obelisk.spindexSlotOne,
                obelisk.spindexSlotTwo,
                obelisk.spindexSlotThree
        };

        // Try each slot starting from the next one after currentSlot
        for (int i = 1; i <= 3; i++) {
            int slotToCheck = (currentSlot + i) % 3;

            // Skip empty inventory slots
            if (invSlots[slotToCheck] == Pattern.Ball.EMPTY) continue;

            // Check if the slot matches the obelisk or if the obelisk slot is empty
            if (obeliskSlots[slotToCheck] != Pattern.Ball.EMPTY &&
                    invSlots[slotToCheck] == obeliskSlots[slotToCheck]) {

                Log.i("Spindexer", "rotating to next matching obelisk slot " + slotToCheck);
                rotateToSlot(slotToCheck);
                return;
            }
        }

        // If no matching obelisk slot found, rotate to next non-empty inventory slot
        rotateToNextFullSlot();

        // If inventory is empty, stay in place
        Log.i("Spindexer", "inventory empty, staying at currentSlot " + currentSlot);
    }


    public void nudgeLeft() {
        nudgeOffset -= 25;
        holdingPosition = false;
        pid.setReference(targetTicks + nudgeOffset);
        pid.reset();
    }

    public void nudgeRight() {
        nudgeOffset += 25;
        holdingPosition = false; // allow PID to move
        pid.setReference(targetTicks + nudgeOffset);
        pid.reset();
    }





    public void showTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Spindexer ---");
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Target Pos",  targetTicks);
        telemetry.addData("Encoder Pos",  encoder.getCurrentPosition());
        telemetry.addData("At Target?", isAtTargetPosition());
        telemetry.addData("Holding", holdingPosition);
        telemetry.addData("Intake Active", intakeCycleActive);
        telemetry.addData("Detected This Cycle", ballDetectedThisCycle);
        telemetry.addData("Inventory", "[%s] [%s] [%s]",
                inventory.spindexSlotOne,
                inventory.spindexSlotTwo,
                inventory.spindexSlotThree);

    }
    public void setInventory(Pattern p)
    {
        inventory.updatePattern(p.spindexSlotOne, p.spindexSlotTwo, p.spindexSlotThree);
    }
    private void initVoltageSensor(HardwareMap hwMap)
    {
        voltageSensor = hwMap.voltageSensor.iterator().next();
    }



}
