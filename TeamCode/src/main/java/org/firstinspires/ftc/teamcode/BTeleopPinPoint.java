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


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "BTeleop PinPoint")
public class BTeleopPinPoint extends RobotMasterPinpoint {

    PIDController autoheading = new PIDController(0.04,0.0005,0);

    final double intakePower = 1;
    final double kickerInit = 0.52;
    final double kickerMin = .52;
    final double kickerMax = .48;
    double kickerPos = kickerInit;

    boolean intakeOn = false;
    boolean circlePressedLast = false;
    boolean trianglePressedLast = false;
    boolean crossPressedLast = false;
    boolean dpadUpPressedLast = false;
    boolean dpadDownPressedLast = false;
    boolean isGreen = false;
    boolean isPurple = false;

    @Override
    public void init() {
        isAuto = false;
        resetEncoders =false;
        super.init();
        kicker.setPosition(kickerInit);


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


    @Override
    public void mainLoop() {
        super.mainLoop();


        double error = 0.0;


        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        movement_y = -gamepad1.left_stick_y;
        movement_x = gamepad1.left_stick_x;

        artifactSensor.getNormalizedColors();

        final double purpleThreshold = 0.003;
        final double greenThreshold = 0.002;

        if (artifactSensor.getNormalizedColors().red > purpleThreshold && artifactSensor.getNormalizedColors().blue > purpleThreshold) {
            isPurple = true;
            isGreen = false;
        }
        else if (artifactSensor.getNormalizedColors().green > greenThreshold && artifactSensor.getNormalizedColors().red < 0.0015) {
            isGreen = true;
            isPurple = false;
        }
        else {
            isGreen = false;
            isPurple = false;
        }
        telemetry.addData("isPurple", isPurple);
        telemetry.addData("isGreen", isGreen);
        telemetry.addData("Red Value", artifactSensor.getNormalizedColors().red);
        telemetry.addData("Blue Value", artifactSensor.getNormalizedColors().blue);
        telemetry.addData("Green Value", artifactSensor.getNormalizedColors().green);
        telemetry.update();
        LLResult llResult = limelight.getLatestResult();

        double spindexCurrentPosition = intake.getCurrentPosition();
        telemetry.addData("Intake Encoder Values", spindexCurrentPosition);
        telemetry.addData("Intake Encoder Angle ", VisionUtils.normalizeAngleFromTicks(spindexCurrentPosition, spindexCountsPerRev));

        if(llResult.isValid() && !VisionUtils.isTagObelisk(VisionUtils.getTagId(llResult)))
        {
            Pose3D pose = llResult.getBotpose();

            //calculate heading error
            double limelightX = llResult.getTx();
            error = autoheading.calculatePID(limelightX);

            gamepad1.rumble(1, 1, 20);

            telemetry.addData("tx", llResult.getTx());
            telemetry.addData("ty", llResult.getTy());
            telemetry.addData("pose", pose.toString());
            telemetry.addData("error", error);
            telemetry.addData("proportional", autoheading.getProportional());
            telemetry.addData("integral", autoheading.getIntegral());
            telemetry.addData("derivative", autoheading.getDerivative());
            telemetry.update();


        }


        if (gamepad1.guide){
            movement_turn = error;
        }
        else {
            movement_turn = gamepad1.right_stick_x;

        }

        drive.applyMovementDirectionBased();


        trianglePressedLast = gamepad1.triangle;
        crossPressedLast = gamepad1.cross;


        if(gamepad1.circle && !circlePressedLast)
        {
            intakeOn = !intakeOn;
        }
        circlePressedLast = gamepad1.circle;

        if(intakeOn) {
            intake.setPower(intakePower);
        } else {
            intake.setPower(0);
        }

        if(gamepad1.square)
        {
            spindexer.setPower(0.7);
        }
        else
        {
            spindexer.setPower(0);

        }
        if(gamepad1.dpad_up && !dpadUpPressedLast)
        {
            kickerPos = kickerMax;
        }
        else if(gamepad1.dpad_down && !dpadDownPressedLast)
        {
            kickerPos = kickerMin;
        }

        dpadUpPressedLast = gamepad1.dpad_up;
        dpadDownPressedLast = gamepad1.dpad_down;



        telemetry.addData("intakePower ", intakePower);
        telemetry.addData("intake ", intake.getPower());
        telemetry.addData("kicker", kicker.getPosition());
        telemetry.addData("kickerInit", kickerInit);




    }
}
