package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

/**
 * Selectable Auto Menu
 * Allows you to choose which autonomous to run at init time
 * Configures gate settings before creating the auto instance
 */
@Autonomous(name = "Auto Selector", group = "Menu")
public class AutoSelector extends SelectableOpMode {

    public AutoSelector() {
        super("Select Autonomous", s -> {
            s.folder("Blue Side Close", blue -> {
                blue.add("Blue - Full Auto Gate Park", () -> {
                    // Set all gates ON
                    AutoBlue.GATE_HIT = true;
                    AutoBlue.DO_FIRST_CYCLE = true;
                    AutoBlue.DO_SECOND_CYCLE = true;
                    AutoBlue.DO_THIRD_CYCLE = true;
                    AutoBlue.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Blue Close Full Auto Gate Park";
                    return new AutoBlue();
                });
                blue.add("Blue - Full Auto Zone Park", () -> {
                    // Set all gates ON
                    AutoBlue.GATE_HIT = true;
                    AutoBlue.DO_FIRST_CYCLE = true;
                    AutoBlue.DO_SECOND_CYCLE = true;
                    AutoBlue.DO_THIRD_CYCLE = true;
                    AutoBlue.DO_GATE_PARK = false;
                    AutoMaster.selectedProgram = "Blue Close Full Auto Zone Park";
                    return new AutoBlue();
                });

                blue.add("Blue - No Gate", () -> {
                    // Skip gate hit only
                    AutoBlue.GATE_HIT = false;
                    AutoBlue.DO_FIRST_CYCLE = true;
                    AutoBlue.DO_SECOND_CYCLE = true;
                    AutoBlue.DO_THIRD_CYCLE = true;
                    AutoBlue.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Blue Close No Gate";
                    return new AutoBlue();
                });

                blue.add("Blue - 2 Cycle", () -> {
                    // Skip third cycle
                    AutoBlue.GATE_HIT = true;
                    AutoBlue.DO_FIRST_CYCLE = true;
                    AutoBlue.DO_SECOND_CYCLE = true;
                    AutoBlue.DO_THIRD_CYCLE = false;
                    AutoBlue.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Blue Close 2 Cycle";
                    return new AutoBlue();
                });

                blue.add("Blue - 1 Cycle", () -> {
                    // Skip second and third cycles
                    AutoBlue.GATE_HIT = true;
                    AutoBlue.DO_FIRST_CYCLE = true;
                    AutoBlue.DO_SECOND_CYCLE = false;
                    AutoBlue.DO_THIRD_CYCLE = false;
                    AutoBlue.DO_GATE_PARK = false;
                    AutoMaster.selectedProgram = "Blue Close 1 Cycle";
                    return new AutoBlue();
                });

                blue.add("Blue - Preload Only", () -> {
                    // Just score preload and stay there
                    AutoBlue.GATE_HIT = false;
                    AutoBlue.DO_FIRST_CYCLE = false;
                    AutoBlue.DO_SECOND_CYCLE = false;
                    AutoBlue.DO_THIRD_CYCLE = false;
                    AutoBlue.DO_GATE_PARK = false;
                    AutoMaster.selectedProgram = "Blue Close Preload Only";
                    return new AutoBlue();
                });
            });

            s.folder("Red Side Close", red -> {
                red.add("Red - Full Auto", () -> {
                    AutoRed.GATE_HIT = true;
                    AutoRed.DO_FIRST_CYCLE = true;
                    AutoRed.DO_SECOND_CYCLE = true;
                    AutoRed.DO_THIRD_CYCLE = true;
                    AutoRed.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Red Close Full Auto";
                    return new AutoRed();
                });

                red.add("Red - No Gate", () -> {
                    AutoRed.GATE_HIT = false;
                    AutoRed.DO_FIRST_CYCLE = true;
                    AutoRed.DO_SECOND_CYCLE = true;
                    AutoRed.DO_THIRD_CYCLE = true;
                    AutoRed.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Red Close No Gate";
                    return new AutoRed();
                });

                red.add("Red - 2 Cycle", () -> {
                    AutoRed.GATE_HIT = true;
                    AutoRed.DO_FIRST_CYCLE = true;
                    AutoRed.DO_SECOND_CYCLE = true;
                    AutoRed.DO_THIRD_CYCLE = false;
                    AutoRed.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Red Close 2 Cycle";
                    return new AutoRed();
                });

                red.add("Red - 1 Cycle", () -> {
                    AutoRed.GATE_HIT = true;
                    AutoRed.DO_FIRST_CYCLE = true;
                    AutoRed.DO_SECOND_CYCLE = false;
                    AutoRed.DO_THIRD_CYCLE = false;
                    AutoRed.DO_GATE_PARK = false;
                    AutoMaster.selectedProgram = "Red Close 1 Cycle";
                    return new AutoRed();
                });

                red.add("Red - Preload Only", () -> {
                    AutoRed.GATE_HIT = false;
                    AutoRed.DO_FIRST_CYCLE = false;
                    AutoRed.DO_SECOND_CYCLE = false;
                    AutoRed.DO_THIRD_CYCLE = false;
                    AutoRed.DO_GATE_PARK = true;
                    AutoMaster.selectedProgram = "Red Close Preload Only";
                    return new AutoRed();
                });
            });
            s.folder("Blue Side Far", blueFar -> {
                blueFar.add("Blue Far - Full Auto", () -> {
                    AutoBlueFar.DO_FIRST_CYCLE = true;
                    AutoBlueFar.DO_SECOND_CYCLE = true;
                    AutoBlueFar.DO_PARK = true;
                    AutoMaster.selectedProgram = "Blue Far Full Auto";
                    return new AutoBlueFar();
                });

                blueFar.add("Blue Far - 1 Cycle", () -> {
                    AutoBlueFar.DO_FIRST_CYCLE = true;
                    AutoBlueFar.DO_SECOND_CYCLE = false;
                    AutoBlueFar.DO_PARK = true;
                    AutoMaster.selectedProgram = "Blue Far 1 Cycle";
                    return new AutoBlueFar();
                });

                blueFar.add("Blue Far - Preload Only", () -> {
                    AutoBlueFar.DO_FIRST_CYCLE = false;
                    AutoBlueFar.DO_SECOND_CYCLE = false;
                    AutoBlueFar.DO_PARK = true;
                    AutoMaster.selectedProgram = "Blue Far Preload Only";
                    return new AutoBlueFar();
                });
            });
            s.folder("Red Side Far", redFar -> {
                redFar.add("Red Far - Full Auto", () -> {
                    AutoRedFar.DO_FIRST_CYCLE = true;
                    AutoRedFar.DO_SECOND_CYCLE = true;
                    AutoRedFar.DO_PARK = true;
                    AutoMaster.selectedProgram = "Red Far Full Auto";
                    return new AutoRedFar();
                });

                redFar.add("Red Far - 1 Cycle", () -> {
                    AutoRedFar.DO_FIRST_CYCLE = true;
                    AutoRedFar.DO_SECOND_CYCLE = false;
                    AutoRedFar.DO_PARK = true;
                    AutoMaster.selectedProgram = "Red Far 1 Cycle";
                    return new AutoRedFar();
                });

                redFar.add("Red Far - Preload Only", () -> {
                    AutoRedFar.DO_FIRST_CYCLE = false;
                    AutoRedFar.DO_SECOND_CYCLE = false;
                    AutoRedFar.DO_PARK = true;
                    AutoMaster.selectedProgram = "Red Far Preload Only";
                    return new AutoRedFar();
                });
            });
        });
    }
}

