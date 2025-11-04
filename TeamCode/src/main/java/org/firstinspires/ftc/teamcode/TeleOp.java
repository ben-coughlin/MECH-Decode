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


import android.os.SystemClock;

import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends RobotMasterPinpoint
{

    //state machines!!
    enum progStates{
        IDLE,
        DEBUG,
        STOP,
        SHOOT_PREP,
        READY_TO_SHOOT,
        FIRE_BALL,
        ROTATE_AFTER_SHOT,
        OUTTAKE,
        INTAKE,

    }

    private ElapsedTime stateTimer = new ElapsedTime();
    PIDFController headingHold = new PIDFController(0.03, 0, 0.0); //TODO: tune these!
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
        currentState = progStates.values()[programStage].name();
        debugKeyValues.put("Superstructure State", currentState);


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
            movement_turn = headingHold.calculatePIDF(currentHeadingRad);
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



        if (programStage == progStates.IDLE.ordinal()) {
            if (gamepad1.dpad_up) {
                nextStage(progStates.SHOOT_PREP.ordinal());
            }
            else if (gamepad1.circle) {
                nextStage(progStates.OUTTAKE.ordinal());
            }
        }

        if (programStage == progStates.READY_TO_SHOOT.ordinal()) {
            //
            if (gamepad1.dpad_down) {
                nextStage(progStates.FIRE_BALL.ordinal());
            }
            // cancels the shot
            else if (gamepad1.right_bumper) {
                nextStage(progStates.IDLE.ordinal());
            }
        }

        // --- INTAKE CONTROLS ---
        if (gamepad1.right_trigger > 0.1) {
            if (programStage != progStates.INTAKE.ordinal()) {
                nextStage(progStates.INTAKE.ordinal());
                spindexer.startIntakeCycle();
            }
        } else { // WHEN the trigger is RELEASED
            if (programStage == progStates.INTAKE.ordinal()) {
                nextStage(progStates.IDLE.ordinal());
            }
        }
        //estop
        if (gamepad1.left_bumper) {
            nextStage(progStates.STOP.ordinal());
        }





        //safe home state
        if (programStage == progStates.IDLE.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();

                turret.setFlywheelPower(0);
                intakeSubsystem.turnIntakeOff();
                spindexer.stopSpindexer();
                intakeSubsystem.moveKickerHorizontal();
                intakeSubsystem.setKickerPos(0.37);
            }

        }
        if (programStage == progStates.SHOOT_PREP.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                turret.setFlywheelPower(1);
            }

            if (SystemClock.uptimeMillis() - stateStartTime > 5000) {
                nextStage(progStates.READY_TO_SHOOT.ordinal());
            }
        }

        if (programStage == progStates.READY_TO_SHOOT.ordinal()) {
            if (stageFinished) {

                initializeStateVariables();
                turret.setFlywheelPower(1);
            }


        }

        if (programStage == progStates.FIRE_BALL.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                intakeSubsystem.moveKickerVertical();
            }


            if (SystemClock.uptimeMillis() - stateStartTime > 200) {
                intakeSubsystem.moveKickerHorizontal();

            }
            if (SystemClock.uptimeMillis() - stateStartTime > 500) {
                spindexer.recordShotBall();
                nextStage(progStates.ROTATE_AFTER_SHOT.ordinal());
            }

        }
        if (programStage == progStates.ROTATE_AFTER_SHOT.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                turret.setFlywheelPower(1); // Keep the flywheel spinning
                spindexer.rotateToNextSlot(); // Command the rotation
            }


            if (spindexer.isAtTargetPosition()) {
                //
                nextStage(progStates.READY_TO_SHOOT.ordinal());
            }

            // Optional safety timeout
            if (SystemClock.uptimeMillis() - stateStartTime > 1500) {
                // If rotation takes too long, escape to prevent getting stuck
                spindexer.stopSpindexer();
                nextStage(progStates.IDLE.ordinal());
            }
        }

        if (programStage == progStates.OUTTAKE.ordinal()) {
            if (stageFinished) {

                initializeStateVariables();
                intakeSubsystem.outtake();
            }


            if(stateTimer.seconds() >= 1)
            {
                intakeSubsystem.turnIntakeOff();
                nextStage(progStates.IDLE.ordinal());
            }


        }
        if (programStage == progStates.INTAKE.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                intakeSubsystem.turnIntakeOn();
            }
            spindexer.intakeNewBall();

        }



        //kills everything
        if (programStage == progStates.STOP.ordinal()) {
            if (stageFinished) {

                initializeStateVariables();
            }
            drive.stopAllMovementDirectionBased();
            intakeSubsystem.turnIntakeOff();
            spindexer.stopSpindexer();
            intakeSubsystem.moveKickerHorizontal();
            turret.setFlywheelPower(0);

        }

    }



}
