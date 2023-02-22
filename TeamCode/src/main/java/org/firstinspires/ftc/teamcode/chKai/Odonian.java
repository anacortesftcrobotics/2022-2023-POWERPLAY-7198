package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.chKai.BistroMathics;
import org.firstinspires.ftc.teamcode.powerplay.Gyro;

public class Odonian {
    DcMotor leftFront, rightFront, leftBack, rightBack;
     //imu
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Gyro gyro = new Gyro();

    private double x, y;
    private double lastFL, lastFR, lastBL, lastBR;

    public void initializeHardware(HardwareMap hardwareMap){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastFL = leftFront.getCurrentPosition();
        lastFR = rightFront.getCurrentPosition();
        lastBL = leftBack.getCurrentPosition();
        lastBR = rightBack.getCurrentPosition();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);

        gyro.initializeHardware(hardwareMap);
    }

    public void check(){
        double fl = leftFront.getCurrentPosition() - lastFL;
        double fr = rightFront.getCurrentPosition() - lastFR;
        double bl = leftBack.getCurrentPosition() - lastBL;
        double br = rightBack.getCurrentPosition() - lastBR;
        lastFL = leftFront.getCurrentPosition();
        lastFR = rightFront.getCurrentPosition();
        lastBL = leftBack.getCurrentPosition();
        lastBR = rightBack.getCurrentPosition();

        x += BistroMathics.FLBRx(imu) * (fl + br);
        x += BistroMathics.FRBLx(imu) * (fr + bl);

        y += BistroMathics.FLBRy(imu) * (fl + br);
        y += BistroMathics.FRBLy(imu) * (fr + bl);

        gyro.updateHeading();
    }

    public double getX(){return x;}

    public double getY(){return y;}

    public double getR(){return gyro.getHeading();}
}    