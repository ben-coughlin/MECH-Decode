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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Motor and Servo Testing", group="Iterative OpMode")
//@Disabled
public class MotorAndServoTesting extends RobotMasterPinpoint
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final double TICKS_PER_REVOLUTION = 6000;
    private DcMotorEx shooter = null;
    private DcMotor intake = null;
    private CRServo spindexer = null;
    private Servo launcher = null;
    private double shooterRPM = 0;
    private double spindexerSpeed = .3;
    private double launcherSpeed = .3;
    private double intakeSpeed = .3;
    private boolean shooterOn = false;
    private boolean spindexerOn = false;
    private boolean launcherOn = false;
    private boolean intakeOn = false;
    private boolean aLastState = false;
    private boolean bLastState = false;
    private boolean xLastState = false;
    private boolean yLastState = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        limelight.start();
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        launcher = hardwareMap.get(Servo.class, "launcher");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Set the motor to run using encoders to control velocity
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        //Use the booleans to change the speed of the shooter
        boolean shooterSpeedUp = gamepad1.dpad_up;
        boolean shooterSpeedDown = gamepad1.dpad_down;
        boolean bCurrentState = gamepad1.b;
        boolean intakeSpeedUp = gamepad1.dpad_left;
        boolean intakeSpeedDown = gamepad1.dpad_right;
        boolean xCurrentState = gamepad1.x;
        boolean spindexerSpeedUp = gamepad1.left_stick_button;
        boolean spindexerSpeedDown = gamepad1.right_stick_button;
        boolean aCurrentState = gamepad1.a;
        boolean launcherSpeedUp = gamepad1.left_bumper;
        boolean launcherSpeedDown = gamepad1.right_bumper;
        boolean yCurrentState = gamepad1.y;

        if(shooterSpeedUp){
            shooterRPM += 50;
        }
        else if(shooterSpeedDown){
            shooterRPM -= 50;
        }
        
        if(bCurrentState && !bLastState){
            shooterOn = !shooterOn;
        }
        bLastState = bCurrentState;
        
        if(shooterOn){
            double ticksPerSecond = shooterRPM * TICKS_PER_REVOLUTION / 60;
            shooter.setVelocity(ticksPerSecond);
        }
        else{
            shooter.setPower(0);
        }
        if(intakeSpeedUp){
            intakeSpeed += .02;
        }
        else if(intakeSpeedDown){
            intakeSpeed -= .02;
        }

        if(xCurrentState && !xLastState){
            intakeOn = !intakeOn;
        }
        xLastState = xCurrentState;

        if(intakeOn){
            intake.setPower(intakeSpeed);
        }
        else{
            intake.setPower(0);
        }
        if(spindexerSpeedUp){
            spindexerSpeed += .02;
        }
        else if(spindexerSpeedDown){
            spindexerSpeed -= .02;
        }

        if(aCurrentState && !aLastState){
            spindexerOn = !spindexerOn;
        }
        aLastState = aCurrentState;

        if(spindexerOn){
            spindexer.setPower(spindexerSpeed);
        }
        else{
            spindexer.setPower(0);
        }
        if(launcherSpeedUp){
            launcherSpeed += .02;
        }
        else if(launcherSpeedDown){
            launcherSpeed -= .02;
        }

        if(yCurrentState && !yLastState){
            launcherOn = !launcherOn;
        }
        yLastState = yCurrentState;

        if(launcherOn){
            launcher.setPosition(launcherSpeed);
        }
        else{
            launcher.setPosition(0);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Shooter RPM", shooterRPM);
        telemetry.addData("Current Shooter Velocity (ticks/sec)", shooter.getVelocity());
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Spindexer Speed", spindexerSpeed);
        telemetry.addData("Spindexer On", spindexerOn);
        telemetry.addData("Launcher Speed", launcherSpeed);
        telemetry.addData("Launcher On", launcherOn);
        telemetry.addData("How to turn shooter on", "Press B to turn on shooter, press B again to turn off shooter");
        telemetry.addData("How to turn intake on", "Press X to turn on intake, press X again to turn off intake");
        telemetry.addData("How to turn spindexer on", "Press A to turn on spindexer, press A again to turn off spindexer");
        telemetry.addData("How to turn launcher on", "Press Y to turn on launcher, press Y again to turn off launcher");
        telemetry.addData("How to increase or decrease shooter speed", "Press Dpad Up to increase shooter RPM, press Dpad Down to decrease shooter RPM");
        telemetry.addData("How to increase or decrease intake speed", "Press Dpad Left to increase intake speed, press Dpad Right to decrease intake speed");
        telemetry.addData("How to increase or decrease spindexer speed", "Press Left Stick Button to increase spindexer speed, press Right Stick Button to decrease spindexer speed");
        telemetry.addData("How to increase or decrease launcher speed", "Press Left Bumper to increase launcher speed, press Right Bumper to decrease launcher speed");
        telemetry.addData("Distance from April Tag", limelight.getLatestResult().getTy());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
