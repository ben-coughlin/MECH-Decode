package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Actions;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.DualNum;
//import com.acmerobotics.roadrunner.HolonomicController;
//import com.acmerobotics.roadrunner.MecanumKinematics;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.MotorFeedforward;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Pose2dDual;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.PoseVelocity2dDual;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.ProfileParams;
//import com.acmerobotics.roadrunner.Rotation2d;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.TimeTrajectory;
//import com.acmerobotics.roadrunner.TimeTurn;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
//import com.acmerobotics.roadrunner.TurnConstraints;
//import com.acmerobotics.roadrunner.Twist2d;
//import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
//import com.acmerobotics.roadrunner.ftc.Encoder;
//import com.acmerobotics.roadrunner.ftc.FlightRecorder;
//import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
//import com.acmerobotics.roadrunner.ftc.LazyImu;
//import com.acmerobotics.roadrunner.ftc.LynxFirmware;
//import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
//import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
//import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrivePinPoint {
//    public static class Params {
//    }
//    public static Params PARAMS = new Params();

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;


    public MecanumDrivePinPoint(HardwareMap hardwareMap) {

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html


        voltageSensor = hardwareMap.voltageSensor.iterator().next();


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

        applyMovementDirectionBased();
    }

    public void applyMovementDirectionBased() {
        long currTime = SystemClock.uptimeMillis();
        if (currTime - lastUpdateTime < 16) {
            return;
        }
        lastUpdateTime = currTime;

        double fl_power_raw = movement_y - movement_turn + movement_x * 1.5;
        double bl_power_raw = movement_y - movement_turn - movement_x * 1.5;
        double br_power_raw = movement_y + movement_turn + movement_x * 1.5;
        double fr_power_raw = movement_y + movement_turn - movement_x * 1.5;

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

}
