package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

/**
 * Selectable Auto Menu
 * Allows you to choose which autonomous to run at init time
 */
@Autonomous(name = "Auto Selector", group = "Menu")
public class AutoSelector extends SelectableOpMode {

    public AutoSelector() {
        super("Select Autonomous", s -> {
            s.folder("Blue Side Close", blue -> {
                blue.add("Blue - Full Auto Gate Park", () -> {
                    AutoBlue.DO_ZONE_PARK = false;
                    AutoMaster.selectedProgram = "Blue Close Full Auto Gate Park";
                    return new AutoBlue();
                });
                blue.add("Blue - Full Auto Zone Park", () -> {
                    //all paths are true by default so only add the ones we DON'T want here
                    AutoMaster.selectedProgram = "Blue Close Full Auto Zone Park";
                    return new AutoBlue();
                });
//don't think we'll ever need this
//                blue.add("Blue - Close Spike & Preload Gate Park", () -> {
//                    // Skip second and third cycles
//                    AutoBlue.GATE_HIT = false;
//                    AutoBlue.DO_FIRST_CYCLE = false;
//                    AutoBlue.DO_SECOND_CYCLE = false;
//                    AutoBlue.DO_THIRD_CYCLE = false;
//                    AutoBlue.DO_ZONE_PARK = false;
//                    AutoMaster.selectedProgram = "Blue Close Spike & Preload Gate Park";
//                    return new AutoBlue();
//                });
                blue.add("Blue - Close Spike & Preload Zone Park", () -> {
                    // Skip second and third cycles
                     AutoBlue.GATE_HIT = false;
                    AutoBlue.DO_FIRST_CYCLE = false;
                    AutoBlue.DO_SECOND_CYCLE = false;
                    AutoBlue.DO_THIRD_CYCLE = false;
                    AutoMaster.selectedProgram = "Blue Close Spike & Preload Zone Park";
                    return new AutoBlue();
                });

                blue.add("Blue - Preload Only", () -> {
                    // Just score preload and stay there
                    AutoBlue.GATE_HIT = false;
                    AutoBlue.DO_FIRST_CYCLE = false;
                    AutoBlue.DO_SECOND_CYCLE = false;
                    AutoBlue.DO_THIRD_CYCLE = false;
                    AutoBlue.DO_FOURTH_CYCLE = false;
                    AutoMaster.selectedProgram = "Blue Close Preload Only";
                    return new AutoBlue();
                });
            });

            s.folder("Red Side Close", red -> {
                red.add("Red - Full Auto Gate Park", () -> {
                    AutoRed.DO_ZONE_PARK = false;
                    AutoMaster.selectedProgram = "Red Close Full Auto Gate Park";
                    return new AutoRed();
                });
                red.add("Red - Full Auto Zone Park", () -> {
                    AutoMaster.selectedProgram = "Red Close Full Auto Zone Park";
                    return new AutoRed();
                });


//                red.add("Red - Close Spike & Preload Gate Park", () -> {
//                    AutoRed.DO_FIRST_CYCLE = false;
//                    AutoRed.DO_SECOND_CYCLE = false;
//                    AutoRed.DO_THIRD_CYCLE = false;
//                    AutoRed.DO_ZONE_PARK = false;
//                    AutoMaster.selectedProgram = "Red Close Spike & Preload Gate Park";
//                    return new AutoRed();
//                });
                red.add("Red - Close Spike & Preload Zone Park", () -> {
                    AutoRed.DO_FIRST_CYCLE = false;
                    AutoRed.DO_SECOND_CYCLE = false;
                    AutoRed.DO_THIRD_CYCLE = false;
                    AutoMaster.selectedProgram = "Red Close Spike & Preload Zone Park";
                    return new AutoRed();
                });

                red.add("Red - Preload Only", () -> {
                    AutoRed.GATE_HIT = false;
                    AutoRed.DO_FIRST_CYCLE = false;
                    AutoRed.DO_SECOND_CYCLE = false;
                    AutoRed.DO_THIRD_CYCLE = false;
                    AutoRed.DO_ZONE_PARK = false;
                    AutoRed.DO_FOURTH_CYCLE = false;
                    AutoMaster.selectedProgram = "Red Close Preload Only";
                    return new AutoRed();
                });
            });
            s.folder("Blue Side Far", blueFar -> {
                blueFar.add("Blue Far - Full Auto", () -> {
                    //if paths are null they're skipped so we don't explicitly have to say false here
                    AutoMaster.selectedProgram = "Blue Far Full Auto";
                    return new AutoBlueFar();
                });

                blueFar.add("Blue Far - 1 Cycle", () -> {
                    AutoBlueFar.DO_SECOND_CYCLE = false;
                    AutoMaster.selectedProgram = "Blue Far 1 Cycle";
                    return new AutoBlueFar();
                });

                blueFar.add("Blue Far - Preload Only", () -> {
                    AutoBlueFar.DO_FIRST_CYCLE = false;
                    AutoBlueFar.DO_SECOND_CYCLE = false;
                    AutoMaster.selectedProgram = "Blue Far Preload Only";
                    return new AutoBlueFar();
                });
            });
            s.folder("Red Side Far", redFar -> {
                redFar.add("Red Far - Full Auto", () -> {
                    AutoMaster.selectedProgram = "Red Far Full Auto";
                    return new AutoRedFar();
                });

                redFar.add("Red Far - 1 Cycle", () -> {
                    AutoRedFar.DO_SECOND_CYCLE = false;
                    AutoMaster.selectedProgram = "Red Far 1 Cycle";
                    return new AutoRedFar();
                });

                redFar.add("Red Far - Preload Only", () -> {
                    AutoRedFar.DO_FIRST_CYCLE = false;
                    AutoRedFar.DO_SECOND_CYCLE = false;
                    AutoMaster.selectedProgram = "Red Far Preload Only";
                    return new AutoRedFar();
                });
            });
        });
    }
}

