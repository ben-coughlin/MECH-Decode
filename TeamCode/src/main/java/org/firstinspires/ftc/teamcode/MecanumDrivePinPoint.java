package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;

import android.os.SystemClock;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public final class MecanumDrivePinPoint {
//    public static class Params {
//    }
//    public static Params PARAMS = new Params();

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;



    public MecanumDrivePinPoint(HardwareMap hwMap) {

        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html


        voltageSensor = hwMap.voltageSensor.iterator().next();


    }

    public void hardStopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    private long lastUpdateTime = 0;

    public void stopAllMovementDirectionBased() {
        movement_x = 0;
        movement_y = 0;
        movement_turn = 0;

        drive(movement_x, movement_y, movement_turn);
    }

    public void applyMovementDirectionBased() {
        long currTime = SystemClock.uptimeMillis();
        if (currTime - lastUpdateTime < 16) {
            return;
        }
        lastUpdateTime = currTime;

        // ALEJANDRO original 1.5 multiplier for strafing
//        double fl_power_raw = movement_y - movement_turn + movement_x * 1.5;
//        double bl_power_raw = movement_y - movement_turn - movement_x * 1.5;
//        double br_power_raw = movement_y + movement_turn + movement_x * 1.5;
//        double fr_power_raw = movement_y + movement_turn - movement_x * 1.5;
        double fl_power_raw = movement_y - movement_turn + movement_x * 1.1;
        double bl_power_raw = movement_y - movement_turn - movement_x * 1.1;
        double br_power_raw = movement_y + movement_turn + movement_x * 1.1;
        double fr_power_raw = movement_y + movement_turn - movement_x * 1.1;


        //find the maximum of the powers
        double maxRawPower = Math.abs(fl_power_raw);
        if (Math.abs(bl_power_raw) > maxRawPower) {
            maxRawPower = Math.abs(bl_power_raw);
        }
        if (Math.abs(br_power_raw) > maxRawPower) {
            maxRawPower = Math.abs(br_power_raw);
        }
        if (Math.abs(fr_power_raw) > maxRawPower) {
            maxRawPower = Math.abs(fr_power_raw);
        }

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if (maxRawPower > 1.0) {
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0 / maxRawPower;
        }
        fl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        fr_power_raw *= scaleDownAmount;


        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        leftFront.setPower(fl_power_raw);
        leftBack.setPower(bl_power_raw);
        rightBack.setPower(br_power_raw);
        rightFront.setPower(fr_power_raw);
    }

    public void applyMovementDirectionBasedFieldRelative(double forward, double right, double rotate, boolean isAutoHeading) {


        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta -= RobotPosition.worldAngle_rad;
        theta = AngleUnit.normalizeRadians(theta);

        if(isAutoHeading) {
             forward = r * Math.sin(theta);
             right = r * Math.cos(theta);
        }
        drive(forward, right, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));


        leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        rightFront.setPower(maxSpeed * (frontRightPower / maxPower));
        leftBack.setPower(maxSpeed * (backLeftPower / maxPower));
        rightBack.setPower(maxSpeed * (backRightPower / maxPower));
    }


}
