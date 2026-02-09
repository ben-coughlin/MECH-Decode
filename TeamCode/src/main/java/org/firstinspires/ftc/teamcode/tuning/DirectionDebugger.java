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

package org.firstinspires.ftc.teamcode.tuning;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Odo;
import org.firstinspires.ftc.teamcode.RobotMaster;

import java.util.HashMap;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Direction Debugger")
@Disabled
public class DirectionDebugger extends RobotMaster
{
    // Declare OpMode members.
    double MOTOR_POWER = 1.0;
    int mode = -1;
    Odo odo = null;

    HashMap<String, String> motorDirections = new HashMap<>();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //drive init is handled in robotmaster
      //super.init();

    odo = new Odo(hardwareMap);
      //this is to see if any motor is reversed
//      motorDirections.put("leftFront", String.valueOf(drive.leftFront.getDirection()));
//      motorDirections.put("rightFront", String.valueOf(drive.rightFront.getDirection()));
//      motorDirections.put("leftBack", String.valueOf(drive.leftBack.getDirection()));
//      motorDirections.put("rightBack", String.valueOf(drive.rightBack.getDirection()));

      //without reading from i2c i can't pull dead wheel directions so you have to find them manually in the GoBildaPinpointDriver file

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

        //everything is in this one file - make sure to choose motor/dead wheel after init
        telemetry.addLine("Press D-Pad Up to enter Motor Direction Debug\nPress D-Pad Down to enter Dead Wheel Direction Debug");

        if(gamepad1.dpad_up)
        {
            mode = 1;
            telemetry.addLine("Motor Direction Debug Selected.");
        }
        else if(gamepad1.dpad_down)
        {
            mode = 2;
            telemetry.addLine("Dead Wheel Direction Debug Selected.");
        }


    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
       if(mode == -1)
       {
         init_loop();
       }
       else if(mode == 1)
       {
          // motorDirectionDebugger();
       }
       else if(mode == 2)
        {
            deadWheelDirectionDebugger();
        }
        telemetry.addLine("Mode: " + mode);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void motorDirectionDebugger()
    {
        telemetry.addLine("Press each button to turn on its respective motor");
        telemetry.addLine();
        telemetry.addLine("Xbox/PS4 Button - Motor");
        telemetry.addLine("X / ▢ - Front Left");
        telemetry.addLine("Y / Δ - Front Right");
        telemetry.addLine("B / O - Back Right");
        telemetry.addLine("A / X - Back Left");
        telemetry.addLine("Motor Directions: ");

        telemetry.addData("leftFront: ", motorDirections.get("leftFront"));
        telemetry.addData("rightFront: ", motorDirections.get("rightFront"));
        telemetry.addData( "leftBack: ", motorDirections.get("leftBack"));
        telemetry.addData( "rightBack", motorDirections.get("rightBack"));

        telemetry.addLine();

        if (gamepad1.x)
        {
            drive.leftFront.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "Left Front");

        }
        else if (gamepad1.y)
        {
            drive.rightFront.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "Right Front");
        }
        else if (gamepad1.b)
        {
            drive.rightBack.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "Right Back");
        }
        else if (gamepad1.a)
        {
            drive.leftBack.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "Left Back");

        }
        else {
            drive.hardStopMotors();
            telemetry.addData("Running: ", "None");
        }


    }

    public void deadWheelDirectionDebugger()
    {

        odo.updateOdo();

        odo.showOdoTelemetry(telemetry);
        odo.showWorldPositionTelemetry(telemetry);
        odo.showRawTelemetry(telemetry);
        telemetry.addLine("Parallel Dead Wheels (should increase forward)");
        telemetry.addLine("Perpendicular Dead Wheels (should increase leftwards)");







    }



}
