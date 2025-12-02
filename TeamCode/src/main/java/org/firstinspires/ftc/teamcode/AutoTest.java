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

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;


@Autonomous(name="AutoTest")

public class AutoTest extends RobotMasterPinpoint
{
    // Declare OpMode members.

    PurePursuitController pp = new PurePursuitController(
            25.0,   // lookahead (cm) - tune: larger = smoother but wider turn
            0.6,    // max linear speed (units/sec) - tune to your system
            0.12,   // min linear speed when near goal
            30.0,   // slowDownRadius (cm)
            6.0     // finishTolerance (cm)
    );
    public enum AutoStage {
        DRIVE_TO_FIRST_BALL,
        INTAKE_FIRST_BALL,
        DRIVE_TO_SHOOT,
        SHOOT_BALL,
        DONE
    }

    AutoStage currentStage = AutoStage.DRIVE_TO_FIRST_BALL;

    @Override
    public void init() {
     super.init();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        super.start();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void mainLoop() {
        super.mainLoop();
        PurePursuitController.Command cmd = pp.update(worldXPosition, worldYPosition, worldAngle_rad);

        switch (currentStage) {
            case DRIVE_TO_FIRST_BALL:
                // Path for Pure Pursuit
                List<PurePursuitController.Waypoint> path1 = Arrays.asList(
                        new PurePursuitController.Waypoint(0, 0),
                        new PurePursuitController.Waypoint(4, 0)
                );
                pp.setPath(path1);


                // Activate intake while driving
                intakeSubsystem.turnIntakeOn();

                // Stage complete?
                if (cmd.finished) {
                    intakeSubsystem.turnIntakeOff();
                    drive.stopAllMovementDirectionBased();
                    currentStage = AutoStage.DONE;
                }
                else
                {
                    MecanumDriveAdapter.toMovementComponents(cmd.vx, cmd.vy, cmd.omega);
                    drive.applyMovementDirectionBased();
                }
                break;


            case DONE:
                drive.stopAllMovementDirectionBased();
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }

}
