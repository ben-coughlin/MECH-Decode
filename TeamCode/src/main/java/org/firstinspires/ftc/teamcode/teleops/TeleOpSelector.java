package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TeleOpMaster;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;

@Autonomous(name = "TeleOp Selector", group = "Menu")
public class TeleOpSelector extends SelectableOpMode {

    public TeleOpSelector() {
        super("Select TeleOp", s -> {
            s.folder("Blue", blue -> {
                blue.add("Blue", () -> {
                    TeleOpMaster.selectedProgram = "BLUE TELEOP";
                    return new TeleOpBlue();
                });

            });

            s.folder("Red", red -> {
                red.add("Red", () -> {
                    TeleOpMaster.selectedProgram = "RED TELEOP";
                    return new TeleOpRed();
                });

            });

        });
    }
}

