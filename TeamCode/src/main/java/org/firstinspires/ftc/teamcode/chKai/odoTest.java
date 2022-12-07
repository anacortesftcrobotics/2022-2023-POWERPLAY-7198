/*package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name = "odoTest")
public class odoTest extends LinearOpMode {

    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;

    private int positionx = 0;
    private int positiony = 0;
    BNO055IMU imu;


    Orientation angles;
    Acceleration gravity;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     *//*
    @Override
    public void runOpMode() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //left encoder is leftBack; right encoder is rightBack; rear encoder is rightFront.
                rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
                leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
                telemetry.addData("X",getX());
                telemetry.addData("Y",getY());
                telemetry.addData("angle",getAngle());
                telemetry.update();
            }
        }
    }
    public double getAngle(){ //gets angle in radian form. 0* is 90* to the right of the starting orientation, and value adds when turning counterclockwise.
        //use for trig stuff
        float angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double angle2 = Math.toRadians(angle1)

        return radians;
    }
    public int getX(){ //gets current position on the x-axis. Assumes that starting point is 0,0. Must be continually called.
        int x = positionx;
        x += ((leftBack.getCurrentPosition()-rightBack.getCurrentPosition())/2)*Math.cos(getAngle());
        x += ((rightFront.getCurrentPosition())/2)*Math.sin(getAngle());
        positionx = x;
        return x;
    }
    public int getY(){ //gets current position on the y-axis. Assumes that starting point is 0,0. Must be continually called.
        int y = positiony;
        y += ((leftBack.getCurrentPosition()-rightBack.getCurrentPosition())/2)*Math.sin(getAngle());
        y += ((rightFront.getCurrentPosition())/2)*Math.cos(getAngle());
        positiony = y;
        return y;
    }
}
*/