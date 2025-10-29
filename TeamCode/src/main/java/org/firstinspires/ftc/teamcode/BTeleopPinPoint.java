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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "BTeleop PinPoint")
public class BTeleopPinPoint extends RobotMasterPinpoint
{

    //state machines!!
    enum TeleopStates{
        IDLE,
        DEBUG,
        INTAKE,
        FIRE_BALL,
        OUTTAKE,

    }

    private TeleopStates currState = TeleopStates.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    PIDController headingHold = new PIDController(0.03, 0, 0.0, true); //TODO: tune these!
    Toggle circleToggle = new Toggle(false);
    Toggle squareToggle = new Toggle(false);
    Toggle autoAimToggle = new Toggle(true);

    boolean isAutoHeading = false;
    boolean isAutoAiming = false;
    double targetHeading = 0.0;


    @Override
    public void init()
    {
        isAuto = false;
        resetEncoders = false;
        super.init();


    }

    public void init_loop()
    {
        super.init_loop();


    }


    @Override
    public void start()
    {
        super.start();


    }

    @Override
    public void stop()
    {
        super.stop();

    }


    @Override
    public void mainLoop()
    {
        super.mainLoop();

        //toggles - - - - - - - - - -
        circleToggle.updateToggle(gamepad1.circle);
        squareToggle.updateToggle(gamepad1.square);
        autoAimToggle.updateToggle(gamepad2.square);

        //driving stuff - - - - - - - - - - -
        movement_y = -gamepad1.left_stick_y;
        movement_x = gamepad1.left_stick_x;

        double currentHeadingRad = RobotPosition.worldAngle_rad;
        double turnDeadzone = 0.05;

        if (Math.abs(gamepad1.right_stick_x) > turnDeadzone)
        {
            //if the driver is turning manually then that takes priority
            movement_turn = gamepad1.right_stick_x;
            isAutoHeading = false;
            targetHeading = currentHeadingRad;
            headingHold.reset();

        }
        else
        {
            //if there's no turning at all, use PID to hold us at the same heading
            headingHold.setReference(targetHeading);
            movement_turn = headingHold.calculatePID(currentHeadingRad);
            isAutoHeading = false;

        }

        drive.applyMovementDirectionBasedFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, movement_turn, isAutoHeading);



        //limelight  - - - - - - - - - - - -
        isAutoAiming = autoAimToggle.getState();

        if (limelight.getCurrResult() != null && limelight.getCurrResult().isValid() && !VisionUtils.isTagObelisk(VisionUtils.getTagId(limelight.getCurrResult())))
        {
            double llError = limelight.getCurrResult().getTx();

            turret.aimTurret(isAutoAiming, llError, gamepad2.right_stick_x);
        }
        else
        {
            turret.aimTurret(false, 0, gamepad2.right_stick_x);
        }


        switch(currState)
        {
            case IDLE:
                turret.setFlywheelPower(0);
                intakeSubsystem.turnIntakeOff();
                intakeSubsystem.stopSpindexer();
                intakeSubsystem.moveKickerHorizontal();

                if(circleToggle.getState())
                {
                    resetStateVars();
                    currState = TeleopStates.INTAKE;
                }
                if(squareToggle.getState())
                {
                    resetStateVars();
                    currState = TeleopStates.FIRE_BALL;
                }

                if(gamepad1.dpad_down)
                {
                    resetStateVars();
                    currState = TeleopStates.OUTTAKE;
                }
                break;

            case INTAKE:
                intakeSubsystem.turnIntakeOn();
                if(!circleToggle.getState())
                {
                    //intakeSubsystem.rotateSpindexerOneSlot();
                    intakeSubsystem.turnIntakeOff();
                   currState = TeleopStates.IDLE;

                }

                break;
            case FIRE_BALL:

                turret.setFlywheelPower(1);
                if(stateTimer.seconds() >= 5)
                {
                    intakeSubsystem.moveKickerVertical();

                }
                if(stateTimer.seconds() >= 5.5)
                {
                    intakeSubsystem.moveKickerHorizontal();

                }
                if(!squareToggle.getState())
                {
                    turret.setFlywheelPower(0);
                    currState = TeleopStates.IDLE;
                }

                break;
            case OUTTAKE:
                intakeSubsystem.outtake();
                if(stateTimer.seconds() >= 1)
                {
                    intakeSubsystem.turnIntakeOff();
                    currState = TeleopStates.IDLE;
                }

                telemetry.addData("Robot State", currState.toString());
                telemetry.addData("State Timer (s)", "%.2f", stateTimer.seconds());
        }

    }

    public void resetStateVars()
    {
        stateTimer.reset();
    }



}
