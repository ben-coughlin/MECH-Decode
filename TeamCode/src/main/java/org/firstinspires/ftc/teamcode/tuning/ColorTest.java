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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Breakbeam;

/**
 * This OpMode cycles through different indicator light colors on a one-second interval.
 * The light will continuously scroll through: Red, Orange, Yellow, Green, Azure, Blue,
 * Indigo, Violet, White, and Off.
 */

@TeleOp(name="Color Cycle OpMode", group="Iterative OpMode")
public class ColorTest extends OpMode
{
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime colorTimer = new ElapsedTime();
    private Servo indicatorLight = null;
    Breakbeam breakbeam;

    private int currentColorIndex = 0;
    private String[] colorNames = {"Red", "Orange", "Yellow", "Green", "Azure", "Blue", "Indigo", "Violet", "White", "Off"};

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        breakbeam = new Breakbeam(hardwareMap);

        // Initialize the hardware variable
        indicatorLight = hardwareMap.get(Servo.class, "light");

        // Tell the driver that initialization is complete
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
        colorTimer.reset();
        currentColorIndex = 0;
        setLightRed();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Check if one second has elapsed
        if (colorTimer.seconds() >= 1.0) {
            colorTimer.reset();
            currentColorIndex++;

            // Loop back to the beginning after going through all colors
            if (currentColorIndex >= colorNames.length) {
                currentColorIndex = 0;
            }

            // Set the appropriate color based on the index
            switch (currentColorIndex) {
                case 0:
                    setLightRed();
                    break;
                case 1:
                    setLightOrange();
                    break;
                case 2:
                    setLightYellow();
                    break;
                case 3:
                    setLightGreen();
                    break;
                case 4:
                    setLightAzure();
                    break;
                case 5:
                    setLightBlue();
                    break;
                case 6:
                    setLightIndigo();
                    break;
                case 7:
                    setLightViolet();
                    break;
                case 8:
                    setLightWhite();
                    break;
                case 9:
                    turnLightOff();
                    break;
            }
        }

        // Show the elapsed game time and current color
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Current Color", colorNames[currentColorIndex]);
        telemetry.addData("Next color in", "%.1f seconds", 1.0 - colorTimer.seconds());
        breakbeam.displayBreakbeamTelemetry(telemetry);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        turnLightOff();
    }

    // Color control methods
    public void setLightRed(){indicatorLight.setPosition(0.277);}
    public void setLightOrange(){indicatorLight.setPosition(0.333);}
    public void setLightYellow(){indicatorLight.setPosition(0.388);}
    public void setLightGreen(){indicatorLight.setPosition(0.500);}
    public void setLightAzure(){indicatorLight.setPosition(0.555);}
    public void setLightBlue(){indicatorLight.setPosition(0.611);}
    public void setLightIndigo(){indicatorLight.setPosition(0.666);}
    public void setLightViolet(){indicatorLight.setPosition(0.722);}
    public void setLightWhite(){indicatorLight.setPosition(1);}
    public void turnLightOff(){indicatorLight.setPosition(0);}
}