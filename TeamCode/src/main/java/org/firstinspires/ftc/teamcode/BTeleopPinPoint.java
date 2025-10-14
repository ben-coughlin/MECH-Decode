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


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@TeleOp(name = "BTeleop PinPoint")
public class BTeleopPinPoint extends RobotMasterPinpoint {

    PIDController autoheading = new PIDController(0.04,0.0005,0);
    PIDController headingHold = new PIDController(0.03,0,0.0); //TODO: tune these!
    boolean isAutoHeading = false;
    boolean intakeOn = false;
    boolean circlePressedLast = false;
    boolean trianglePressedLast = false;
    boolean crossPressedLast = false;
    boolean dpadUpPressedLast = false;
    boolean dpadDownPressedLast = false;
    double targetHeading = 0.0;



    @Override
    public void init() {
        isAuto = false;
        resetEncoders =false;
        super.init();



    }

    public void init_loop() {
        super.init_loop();


    }


    @Override
    public void start() {
        super.start();


    }

    @Override
    public void stop() {
        super.stop();

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




        if(limelight.currResult.isValid() && !Limelight.isTagObelisk(Limelight.getTagId(limelight.currResult)))
        {

            //calculate heading error
            double limelightX = limelight.currResult.getTx();
            error = autoheading.calculatePID(limelightX);

            gamepad1.rumble(1, 1, 20);


        }



        colorSensor.showColorSensorTelemetry(telemetry);

        double currentHeadingRad = RobotPosition.worldAngle_rad;
        double turnDeadzone = 0.05;

        if (gamepad1.guide){
            movement_turn = error;
            isAutoHeading = true;
        }
        else if(Math.abs(movement_turn) > turnDeadzone)
        {
            movement_turn = gamepad1.right_stick_x;
            isAutoHeading = false;
            targetHeading = currentHeadingRad;
            headingHold.reset();
        }
        else {
            headingHold.setReference(targetHeading);
            movement_turn = headingHold.calculatePID(currentHeadingRad);
        }

        drive.applyMovementDirectionBasedFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, movement_turn, isAutoHeading);


        trianglePressedLast = gamepad1.triangle;
        crossPressedLast = gamepad1.cross;


        if(gamepad1.circle && !circlePressedLast)
        {
            intakeOn = !intakeOn;
        }
        circlePressedLast = gamepad1.circle;

        if(intakeOn) {
            intakeSubsystem.turnIntakeOn();
        } else {
            intakeSubsystem.turnIntakeOff();
        }

        if(gamepad1.square)
        {
            intakeSubsystem.rotateSpindexerOneSlot();
        }
        if(gamepad1.dpad_up && !dpadUpPressedLast)
        {
            intakeSubsystem.turnKickerOn();
        }
        else if(gamepad1.dpad_down && !dpadDownPressedLast)
        {
            intakeSubsystem.turnKickerOff();
        }

        dpadUpPressedLast = gamepad1.dpad_up;
        dpadDownPressedLast = gamepad1.dpad_down;






    }
}
