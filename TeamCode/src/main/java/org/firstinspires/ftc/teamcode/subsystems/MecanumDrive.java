package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public final class MecanumDrive {

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public MecanumDrive(HardwareMap hwMap) {

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


        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html



    }

    public void teleopDrive(double rawY, double rawX, double rawRX, double heading)
    {


        if (Math.abs(rawY) < 0.05) rawY = 0;
        if (Math.abs(rawX) < 0.05) rawX = 0;
        if (Math.abs(rawRX) < 0.05) rawRX = 0;

        double y = rawX * Math.sin(-heading) + rawY * Math.cos(-heading);
        double x = rawX * Math.cos(-heading) - rawY * Math.sin(-heading);
        double rx = -rawRX;

        if(y == 0 && x == 0 && rx == 0) {
            hardStopMotors();
        }
        else {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            applyMotorPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        }
    }


    public void hardStopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void applyMotorPower(double lF, double lB, double rF, double rB)
    {
        leftFront.setPower(lF);
        leftBack.setPower(lB);
        rightFront.setPower(rF);
        rightBack.setPower(rB);

    }


}
