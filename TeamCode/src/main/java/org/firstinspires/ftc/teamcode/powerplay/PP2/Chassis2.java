package org.firstinspires.ftc.teamcode.powerplay.PP2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.powerplay.Chassis;
import org.firstinspires.ftc.teamcode.powerplay.Gyro;
import org.firstinspires.ftc.teamcode.powerplay.Heading;

public class Chassis2 extends Chassis {
    Gyro imu;

    public void initializeHardware(HardwareMap hardwareMap)
    {
        //leftBack
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftFront
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightBack
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightFront
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.initializeHardware(hardwareMap);
    }
    public void xyrMovement(double x, double y){
            double rx = imu.getHeading();

            double leftBackPower;
            double rightBackPower;
            double leftFrontPower;
            double rightFrontPower;

            x = deadZone(x);
            y = deadZone(y);
            rx = deadZone(rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

            leftBackPower = (y - x + rx) / denominator;
            rightBackPower = (y + x - rx) / denominator;
            leftFrontPower = (y + x + rx) / denominator;
            rightFrontPower = (y - x - rx) / denominator;

            leftBackPower = Math.cbrt(leftBackPower);
            rightBackPower = Math.cbrt(rightBackPower);
            leftFrontPower = Math.cbrt(leftFrontPower);
            rightFrontPower = Math.cbrt(rightFrontPower);

            leftBackPower = (leftBackPower * speedCoefficient * 0.9);
            rightBackPower = (rightBackPower * speedCoefficient * 0.9);
            leftFrontPower = (leftFrontPower  * speedCoefficient * 0.9);
            rightFrontPower = (rightFrontPower * speedCoefficient * 0.9);

            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
    }
}
