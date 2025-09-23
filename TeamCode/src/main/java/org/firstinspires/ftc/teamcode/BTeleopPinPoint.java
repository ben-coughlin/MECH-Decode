/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.HashMap;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "BTeleop PinPoint")
public class BTeleopPinPoint extends RobotMasterPinpoint {

    @Override
    public void init() {
        isAuto = false;
        resetEncoders =false;

        super.init();

        //drive = new MecanumDrivePinPoint(hardwareMap);

    }

    public void init_loop() {
        super.init_loop();


    }


    @Override
    public void start() {
        super.start(); //what is super.start?


    }

    @Override
    public void stop() {
        super.stop(); //what is super.stop?

    }


    private boolean autoPilotEnabled = false;
    private boolean atBasket = false;


    private ArrayList<Double> distances = new ArrayList<>();

    private boolean hangAutomation = false;


    private HashMap<Integer, PointDouble> yellowDropBlue = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(30, 118));
        put(1, new PointDouble(36, 118));
        put(2, new PointDouble(42, 118));
    }};



    private boolean autoDriveToHang = false;
    //private PointDouble autoDriveToDroneLaunchPosition = new PointDouble();


    @Override
    public void mainLoop() {
        super.mainLoop();

        double targetX = 0.0;
        double kP = 0.03;
        double limelightX = 0.0;
        double error = 0.0;

        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        movement_y = -gamepad1.left_stick_y;
        movement_x = gamepad1.left_stick_x;

        drive.applyMovementDirectionBased();

        LLResult llResult = limelight.getLatestResult();

        if(llResult.isValid())
        {
            Pose3D pose = llResult.getBotpose();

            //calculate heading error
            limelightX = llResult.getTx();
            error = kP * (targetX - limelightX);

            gamepad1.rumble(1, 1, 20);

            telemetry.addData("tx", llResult.getTx());
            telemetry.addData("ty", llResult.getTy());
            telemetry.addData("pose", pose.toString());
            telemetry.addData("error", error);
            telemetry.update();

        }


        if (gamepad1.a){
            movement_turn = error;
        }
        else {
            movement_turn = -gamepad1.right_stick_x;

        }



    }
}
