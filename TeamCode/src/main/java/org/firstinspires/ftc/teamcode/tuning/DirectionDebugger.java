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



import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Odo;
import org.firstinspires.ftc.teamcode.RobotMaster;

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
@Disabled
@TeleOp(name="Direction Debugger")
public class DirectionDebugger extends RobotMaster

{

    // Declare OpMode members.
    double MOTOR_POWER = 1.0;
    int mode = -1;
    Odo odo = null;
    DcMotorEx motor0 = null;
    DcMotorEx motor1 = null;
    DcMotorEx motor2 = null;
    DcMotorEx motor3 = null;
    RevColorSensorV3 upperColorSensor = null;
    RevColorSensorV3 middleColorSensor = null;
    RevColorSensorV3 lowerColorSensor = null;
    CRServo leftTurret = null;
    CRServo rightTurret = null;
    Servo kickstand = null;

    double leftTurretPos = 0;
    double rightTurretPos = 0;

    final float[] hsvValues = new float[3];
    final float[] hsvValues2 = new float[3];
    final float[] hsvValues3 = new float[3];


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //drive init is handled in robotmaster
        odo = new Odo(hardwareMap);
      //this is to see if any motor is reversed
//      motorDirections.put("leftFront", String.valueOf(drive.leftFront.getDirection()));
//      motorDirections.put("rightFront", String.valueOf(drive.rightFront.getDirection()));
//      motorDirections.put("leftBack", String.valueOf(drive.leftBack.getDirection()));
//      motorDirections.put("rightBack", String.valueOf(drive.rightBack.getDirection()));
//         motor0 = hardwareMap.get(DcMotorEx.class, "intake");
//         motor1 = hardwareMap.get(DcMotorEx.class, "transfer");
//         motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//         motor2 = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
//         motor3 = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
//         motor3.setDirection(DcMotorSimple.Direction.REVERSE);
         motor0 = hardwareMap.get(DcMotorEx.class, "leftFront");
         motor0.setDirection(DcMotorSimple.Direction.REVERSE);
         motor1 = hardwareMap.get(DcMotorEx.class, "leftBack");
           motor2 = hardwareMap.get(DcMotorEx.class, "rightFront");
         motor3 = hardwareMap.get(DcMotorEx.class, "rightBack");
         motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        //


         kickstand = hardwareMap.get(Servo.class, "kickstand");
         kickstand.setDirection(Servo.Direction.REVERSE);
         leftTurret = hardwareMap.get(CRServo.class, "turretLeft");
         rightTurret = hardwareMap.get(CRServo.class, "turretRight");

         upperColorSensor = hardwareMap.get(RevColorSensorV3.class, "upperColorSensor");
         middleColorSensor = hardwareMap.get(RevColorSensorV3.class, "middleColorSensor");
         lowerColorSensor = hardwareMap.get(RevColorSensorV3.class, "lowerColorSensor");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

        upperColorSensor.getNormalizedColors();
        middleColorSensor.getNormalizedColors();
        lowerColorSensor.getNormalizedColors();

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

        NormalizedRGBA colors1 = upperColorSensor.getNormalizedColors();
        NormalizedRGBA colors2 = middleColorSensor.getNormalizedColors();
        NormalizedRGBA colors3 = lowerColorSensor.getNormalizedColors();

        if(mode == -1)
       {
         init_loop();
       }
       else if(mode == 1)
       {
          motorDirectionDebugger();
       }
       else if(mode == 2)
        {
            deadWheelDirectionDebugger();
        }
        Color.colorToHSV(colors1.toColor(), hsvValues);
        Color.colorToHSV(colors2.toColor(), hsvValues2);
        Color.colorToHSV(colors3.toColor(), hsvValues3);

        telemetry.addLine("Upper");
        telemetry.addLine()
                .addData("Red", "%.3f", colors1.red)
                .addData("Green", "%.3f", colors1.green)
                .addData("Blue", "%.3f", colors1.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors1.alpha);
        telemetry.addLine("Mid");
        telemetry.addLine()
                .addData("Red", "%.3f", colors2.red)
                .addData("Green", "%.3f", colors2.green)
                .addData("Blue", "%.3f", colors2.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues2[0])
                .addData("Saturation", "%.3f", hsvValues2[1])
                .addData("Value", "%.3f", hsvValues2[2]);
        telemetry.addData("Alpha", "%.3f", colors2.alpha);
        telemetry.addLine("Low");
        telemetry.addLine()
                .addData("Red", "%.3f", colors3.red)
                .addData("Green", "%.3f", colors3.green)
                .addData("Blue", "%.3f", colors3.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues3[0])
                .addData("Saturation", "%.3f", hsvValues3[1])
                .addData("Value", "%.3f", hsvValues3[2]);
        telemetry.addData("Alpha", "%.3f", colors3.alpha);

       // telemetry.addLine("Turret Pos " + motor3.getCurrentPosition());
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
        telemetry.addLine("DPad Up - turn left turret servo positive, DPad Down - turn left turret servo negative");
        telemetry.addLine("DPad Left - turn right turret servo positive, DPad Right - turn right turret servo negative");

        telemetry.addLine();

        if(gamepad1.dpad_up)
        {
            leftTurretPos = .15;
        }
        else if(gamepad1.dpad_down)
        {
            leftTurretPos = 0;
        }
        leftTurret.setPower(leftTurretPos);
        rightTurret.setPower(leftTurretPos);

        if(gamepad1.right_trigger > 0.1)
        {
            float power = gamepad1.right_trigger;
            motor0.setPower(power);
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
        }
        if (gamepad1.x)
        {
            motor0.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "0");

        }
        else if (gamepad1.y)
        {
            motor1.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "1");
        }
        else if (gamepad1.b)
        {
            motor2.setPower(MOTOR_POWER);
            telemetry.addData("Running: ", "2");
        }
//        else if (gamepad1.a)
//        {
//            motor3.setPower(MOTOR_POWER);
//            telemetry.addData("Running: ", "3");
//
//        }
        else {
            motor0.setPower(0);
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);

            telemetry.addData("Running: ", "None");
        }

        if(gamepad1.left_bumper)
        {
            kickstand.setPosition(0.2);
        }
        else
        {
            kickstand.setPosition(0);
        }

        telemetry.addData("Servo pwr ", leftTurretPos);
        telemetry.addData("hdng ", leftTurretPos);
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
