package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Disabled
/*
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading",angles.firstAngle);

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        while ((poseTracker.getAngle() > (-degrees)) && (linearOpMode.opModeIsActive()))
        {
            double correction = 1 - Math.abs(poseTracker.getAngle() / degrees);
            if (correction < 0.45) correction = 0.45;
            lFront.setPower(motorPower * correction);
            lBack.setPower(motorPower * correction);
            rFront.setPower(-motorPower * correction);
            rBack.setPower(-motorPower * correction);

            telemetry.addData("Target:", degrees);
            telemetry.addData("Angle:", poseTracker.getAngle());
            telemetry.addData("Correction:", correction);
            telemetry.update();
        }


 */
public class LemLibrary
{
    //Variables
    boolean grabbed = false;
    int liftHeight = 0;
    int liftChange = 0;
    double motorPower = 0.8;
    int check = 0;
    //1 - red
    //2 - green
    //3 - blue


    //Funky Init
    public static HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;
    public static Telemetry telemetry;
    public LemLibrary (HardwareMap hwMap, OpMode opModeIn, Telemetry telemetryIn)
    {
        hardwareMap = hwMap;
        telemetry = telemetryIn;
        if (opModeIn instanceof LinearOpMode)
        {
            linearOpMode = (LinearOpMode) opModeIn;
        }
    }

    //Hardware Init
    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront, leftLift, rightLift;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;

    //Servos
    private Servo leftGrab, rightGrab, leftMPCR, rightMPCR;

    //Distance Sensors
    private DistanceSensor leftDistance, rightDistance;

    //Color Sensors
    private ColorSensor color;

    //Touch Sensors
    private TouchSensor zero;

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void init()
    {
        //Motors
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
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftLift
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rightLift
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Odometry
        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderBack = rightFront;
        resetDriveEncoder();

        //Servos
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");
        leftMPCR = hardwareMap.get(Servo.class, "leftMPCR");
        rightMPCR = hardwareMap.get(Servo.class, "rightMPCR");

        //Distance Sensors
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

        //Color Sensors
        color = hardwareMap.get(ColorSensor.class, "color");

        //Limit switches
        zero = hardwareMap.get(TouchSensor.class, "bottomLimit");

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void move(int units)
    {
        //~1304 encoder ticks per cm
        //~3312 encoder ticks per inch
        resetDriveEncoder();
        double expected = units * 3312;
        while(encoderRight.getCurrentPosition() <= expected - 10 || encoderRight.getCurrentPosition() >= expected + 10 || encoderLeft.getCurrentPosition() <= expected - 10 || encoderLeft.getCurrentPosition() >= expected + 10)
        {
            double leftPower = deadZone(Math.max(-0.5,Math.min(0.5,(expected - encoderLeft.getCurrentPosition()))));
            double rightPower = deadZone(Math.max(-0.5,Math.min(0.5,(expected - encoderRight.getCurrentPosition()))));
            leftBack.setPower(leftPower);
            leftFront.setPower(leftPower);
            rightBack.setPower(rightPower);
            rightFront.setPower(rightPower);
            telemetry.addData("Target:", units * 3312);
            telemetry.addData("Actual:", (-encoderRight.getCurrentPosition() + encoderLeft.getCurrentPosition()) / 2);
            telemetry.update();
        }
    }
    public void strafe(int units)
    {
        resetDriveEncoder();
        double expected = units * 3312;
        while(encoderBack.getCurrentPosition() <= expected - 10 || encoderBack.getCurrentPosition() >= expected + 10)
        {
            //morop
        }
    }
    public void turn(double degrees)
    {
        double offset = 2;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double  angle = angles.firstAngle;
        double direction = (angle - degrees) / Math.abs(angle - degrees);
        while (angle < degrees - offset / 2.0 || angle > degrees + offset / 2.0 )
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = angles.firstAngle;
            leftBack.setPower(motorPower * direction);
            leftFront.setPower(motorPower * direction);
            rightBack.setPower(motorPower * -direction);
            rightFront.setPower(motorPower * -direction);
        }
    }
    public void exMove(double x, double y, double rx, int time)
    {
        //use regular drive code for x amount of time
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower = (leftBackPower * motorPower);
        rightBackPower = (rightBackPower * motorPower);
        leftFrontPower = (leftFrontPower  * motorPower);
        rightFrontPower = (rightFrontPower * motorPower);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

        linearOpMode.sleep(time);
    }
    public void MPCR(int position)
    {
        if(position == 0)
        {
            rightMPCR.setPosition(0 + 0.02);
            leftMPCR.setPosition(1 - 0 - 0.02);
        }
        if(position == 1)
        {
            rightMPCR.setPosition(0.6 + 0.02);
            leftMPCR.setPosition(1 - 0.6 - 0.02);
        }
    }
    public void liftZero()
    {
        while(!zero.isPressed())
        {
            leftLift.setTargetPosition(liftChange);
            rightLift.setTargetPosition(liftChange);

            leftLift.setPower(0.8);
            rightLift.setPower(0.8);

            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftChange -= 2;
        }
    }
    public void lift(int level)
    {
        //grabbing, 5 stack 1-5, ground, low, medium, high
        /*
        0 - grabbing height
        1 - stack 1
        2 - stack 2
        3 - stack 3
        4 - stack 4
        5 - stack 5
        6 - ground
        7 - low
        8 - medium
        9 - high
         */
        switch(level) {
            case 0:
                liftHeight = 20;
                break;
            case 1:
                liftHeight = 20;
                break;
            case 2:
                liftHeight = 168;
                break;
            case 3:
                liftHeight = 273;
                break;
            case 4:
                liftHeight = 378;
                break;
            case 5:
                liftHeight = 504;
                break;
            case 6:
                liftHeight = 252;
                break;
            case 7:
                liftHeight = 1302;
                break;
            case 8:
                liftHeight = 2142;
                break;
            case 9:
                liftHeight = 2982;
                break;
            default:
                liftHeight = 20;
        }
        leftLift.setTargetPosition((int) liftHeight + liftChange);
        rightLift.setTargetPosition((int) liftHeight + liftChange);

        leftLift.setPower(0.5);
        rightLift.setPower(0.5);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void grab()
    {
        grabbed = !grabbed;
        if(grabbed)
        {
            leftGrab.setPosition(0.6 + 0.02);
            rightGrab.setPosition(1 - 0.6 + 0.02);
        }
        else
        {
            leftGrab.setPosition(0);
            rightGrab.setPosition(1);
        }
    }
    public void check()
    {
        if(color.red() > color.green() && color.red() > color.blue())
        {
            check = 1;
        }
        else if(color.green() > color.red() && color.green() > color.blue())
        {
            check = 2;
        }
        else if(color.blue() > color.green() && color.blue() > color.red())
        {
            check = 3;
        }
    }
    public void poleCenter()
    {
        //cannibalize teleOp pole center code
    }
    public void deposit(int level)
    {
        //combine functions to make an automatic depositing function.
        //try to make it return to the position it was at the beginning by saving variables and driving back.
    }
    public void resetDriveEncoder()
    {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double deadZone(double input)
    {
        if (Math.abs(input) < 0.05)
        {
            return  0;
        }
        else
        {
            return input;
        }
    }
}
