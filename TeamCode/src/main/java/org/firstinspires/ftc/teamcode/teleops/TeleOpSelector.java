package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpMaster;
import org.firstinspires.ftc.teamcode.utils.VisionUtils;
@Disabled
@TeleOp(name = "TeleOp Selector", group = "Menu")
public class TeleOpSelector extends SelectableOpMode {

    public TeleOpSelector() {
        super("Select TeleOp", s -> {
                s.add("Blue", () -> {
                    TeleOpMaster.selectedProgram = "BLUE TELEOP";
                    return new TeleOpBlue();
                });

                s.add("Red", () -> {
                    TeleOpMaster.selectedProgram = "RED TELEOP";
                    return new TeleOpRed();
                });

        });
    }
}

