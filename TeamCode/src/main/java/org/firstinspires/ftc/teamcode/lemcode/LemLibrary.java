package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;
import org.firstinspires.ftc.teamcode.kaicode.Odo1Offset;

@Disabled
public class LemLibrary
{
    //Variables
    boolean grabbed = false;
    int liftHeight = 0;
    int liftChange = 0;
    int check = 0;
    //1 - red
    //2 - green
    //3 - blue


    //Funky Init
    public static HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;
    public static Telemetry telemetry;
    public LemLibrary(HardwareMap hwMap, OpMode opModeIn, Telemetry telemetryIn)
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

    //Odometry
    Odo1Offset odometryTracker = new Odo1Offset(-813829,796907,-786263,3.5, 8192);

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
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rightLift
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotor.Direction.FORWARD);
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
    public void move(int inches)
    {
        odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startX = odometryTracker.getX() / 2.54;
        double startY = odometryTracker.getY() / 2.54;
        double startHeadingIMU = angles.firstAngle;
        double x = odometryTracker.getX() / 2.54;
        double y = odometryTracker.getY() / 2.54;
        double headingIMU = angles.firstAngle;
        double headingAVE = headingIMU - startHeadingIMU;

        while(linearOpMode.opModeIsActive() && !((x - startX) <= inches + 0.25 && (x - startX) >= inches - 0.25))
        {
            odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            x = odometryTracker.getX() / 2.54;
            y = odometryTracker.getY() / 2.54;
            headingIMU = angles.firstAngle;
            headingAVE = headingIMU - startHeadingIMU;
            double val = (inches - (x - startX));
            double power = (Math.max(-0.5, Math.min(0.5, (val) * (1 / (5 + Math.abs(val))))));
            double correction  = headingAVE / 50;

            leftBack.setPower(deadZone(power + correction));
            leftFront.setPower(deadZone(power + correction));
            rightBack.setPower(deadZone(power - correction));
            rightFront.setPower(deadZone(power - correction));

            telemetry.addData("expected", inches);
            telemetry.addData("actual", x);
            telemetry.addData("difference", val);
            telemetry.addData("left power", power - correction);
            telemetry.addData("right power", power + correction);
            telemetry.addData("power", power);
            telemetry.addData("heading average", headingAVE);
            telemetry.addData("heading imu", headingIMU);
            telemetry.addData("correction", correction);
            telemetry.update();

        }
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
        linearOpMode.sleep(250);
    }

    public void moveY(int inches) {
        odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startX = odometryTracker.getX() / 2.54;
        double startY = odometryTracker.getY() / 2.54;
        double startHeadingIMU = angles.firstAngle;
        double x = odometryTracker.getX() / 2.54;
        double y = odometryTracker.getY() / 2.54;
        double headingIMU = angles.firstAngle;
        double headingAVE = headingIMU - startHeadingIMU;

        while (linearOpMode.opModeIsActive() && !((y - startY) <= inches + 0.25 && (y - startY) >= inches - 0.25)) {
            odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            x = odometryTracker.getX() / 2.54;
            y = odometryTracker.getY() / 2.54;
            headingIMU = angles.firstAngle;
            headingAVE = headingIMU - startHeadingIMU;
            double val = (inches - (y - startY));
            double power = (Math.max(-0.5, Math.min(0.5, (val) * (1 / (5 + Math.abs(val))))));
            double correction = headingAVE / 50;

            leftBack.setPower(deadZone(power + correction));
            leftFront.setPower(deadZone(power + correction));
            rightBack.setPower(deadZone(power - correction));
            rightFront.setPower(deadZone(power - correction));


            telemetry.addData("expected", inches);
            telemetry.addData("actual", x);
            telemetry.addData("difference", val);
            telemetry.addData("left power", power - correction);
            telemetry.addData("right power", power + correction);
            telemetry.addData("power", power);
            telemetry.addData("heading average", headingAVE);
            telemetry.addData("heading imu", headingIMU);
            telemetry.addData("correction", correction);
            telemetry.update();

        }
    }
    public void moveBad(int inches) {
        odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startX = odometryTracker.getX() / 2.54;
        double startY = odometryTracker.getY() / 2.54;
        double startHeadingIMU = angles.firstAngle;
        double x = odometryTracker.getX() / 2.54;
        double y = odometryTracker.getY() / 2.54;
        double headingIMU = angles.firstAngle;
        double headingAVE = headingIMU - startHeadingIMU;

        while (linearOpMode.opModeIsActive() && !((y - startY) <= inches + 0.25 && (y - startY) >= inches - 0.25)) {
            odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            x = odometryTracker.getX() / 2.54;
            y = odometryTracker.getY() / 2.54;
            headingIMU = angles.firstAngle;
            headingAVE = headingIMU - startHeadingIMU;
            double val = (inches - (y - startY));
            double power = (Math.max(-0.5, Math.min(0.5, (val) * (1 / (5 + Math.abs(val))))));
            double correction = headingAVE / 50;

            leftBack.setPower(deadZone(-power + correction));
            leftFront.setPower(deadZone(-power + correction));
            rightBack.setPower(deadZone(-power - correction));
            rightFront.setPower(deadZone(-power - correction));


            telemetry.addData("expected", inches);
            telemetry.addData("actual", x);
            telemetry.addData("difference", val);
            telemetry.addData("left power", power - correction);
            telemetry.addData("right power", power + correction);
            telemetry.addData("power", power);
            telemetry.addData("heading average", headingAVE);
            telemetry.addData("heading imu", headingIMU);
            telemetry.addData("correction", correction);
            telemetry.update();

        }
    }
    public void strafe(int inches)
    {
        odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startX = odometryTracker.getX() / 2.54;
        double startY = odometryTracker.getY() / 2.54;
        double startHeadingIMU = angles.firstAngle;
        double x = odometryTracker.getX() / 2.54;
        double y = odometryTracker.getY() / 2.54;
        double headingIMU = angles.firstAngle;
        double headingTrue = headingIMU - startHeadingIMU;
        while(linearOpMode.opModeIsActive() && !((y - startY) <= inches + 0.25 && (y - startY) >= inches - 0.25))
        {
            odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            x = odometryTracker.getX() / 2.54;
            y = odometryTracker.getY() / 2.54;
            headingIMU = angles.firstAngle;
            headingTrue = headingIMU - startHeadingIMU;
            double val = (inches - (y - startY));
            double power = (Math.max(-0.5, Math.min(0.5, (val) * (1 / (4 + Math.abs(val))))));
            double correction  = headingTrue / 50;

            leftBack.setPower(deadZone(-power - correction));
            leftFront.setPower(deadZone(power + correction));
            rightBack.setPower(deadZone(power - correction));
            rightFront.setPower(deadZone(-power + correction));

            telemetry.addData("expected", inches);
            telemetry.addData("actual", y);
            telemetry.addData("difference", val);
            telemetry.addData("left power", power - correction);
            telemetry.addData("right power", power + correction);
            telemetry.addData("power", power);
            telemetry.addData("heading imu", headingIMU);
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        linearOpMode.sleep(250);
    }
    public void turn(double degrees)
    {
        odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeadingIMU = angles.firstAngle;
        double headingIMU = angles.firstAngle;
        double headingTrue = headingIMU - startHeadingIMU;
        while(linearOpMode.opModeIsActive() && !((headingTrue) <= degrees + 2 && (headingTrue) >= degrees - 2))
        {
            odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingIMU = angles.firstAngle;
            headingTrue = headingIMU - startHeadingIMU;

            double val = (degrees - headingTrue);
            double power = (Math.max(-0.5, Math.min(0.5, (val) * (1 / (15 + Math.abs(val))))));

            leftBack.setPower(deadZone(-power));
            leftFront.setPower(deadZone(-power));
            rightBack.setPower(deadZone(power));
            rightFront.setPower(deadZone(power));

            telemetry.addData("expected", degrees);
            telemetry.addData("actual", headingTrue);
            telemetry.addData("difference", val);
            telemetry.addData("power", power);
            telemetry.addData("power", power);
            telemetry.addData("heading imu", headingIMU);
            telemetry.update();
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        linearOpMode.sleep(250);
    }
    public void MPCR(int position)
    {
        /*
        0 - 1        - retracted
        1 - 0.45     - rest
        2- 0.14      - cone right
         */
        if(position == 0)
        {
            rightMPCR.setPosition(1 + 0.02);
            leftMPCR.setPosition(1 - 1 - 0.02);
        }
        else if(position == 1)
        {
            rightMPCR.setPosition(0.45 + 0.02);
            leftMPCR.setPosition(1 - 0.45 - 0.02);
        }
        else if(position == 2)
        {
            rightMPCR.setPosition(0.14 + 0.02);
            leftMPCR.setPosition(1 - 0.14 - 0.02);
        }
        else
        {
            rightMPCR.setPosition(0.45 + 0.02);
            leftMPCR.setPosition(1 - 0.45 - 0.02);
        }
    }
    public void liftZero()
    {
        if(linearOpMode.opModeIsActive() && !zero.isPressed())
        {
            liftChange = 0;
            while (linearOpMode.opModeIsActive() && !zero.isPressed())
            {
                leftLift.setTargetPosition(liftChange);
                rightLift.setTargetPosition(liftChange);

                leftLift.setPower(0.8);
                rightLift.setPower(0.8);

                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftChange -= 5;
            }
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
                liftHeight = 30;
                break;
            case 1:
                liftHeight = 30;
                break;
            case 2:
                liftHeight = 148;
                break;
            case 3:
                liftHeight = 253;
                break;
            case 4:
                liftHeight = 358;
                break;
            case 5:
                liftHeight = 484;
                break;
            case 6:
                liftHeight = 260;
                break;
            case 7:
                liftHeight = 1230;
                break;
            case 8:
                liftHeight = 2150;
                break;
            case 9:
                liftHeight = 3000;
                break;
            default:
                liftHeight = 30;
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
        telemetry.addData("color", check);
        telemetry.update();
    }
    public int check()
    {
        if(color.red() > color.green() && color.red() > color.blue())
        {
            //red
            return 1;
        }
        else if(color.green() > color.red() && color.green() > color.blue())
        {
            //green
            return 2;
        }
        else if(color.blue() > color.green() && color.blue() > color.red())
        {
            //blue
            return 3;
        }
        return 1;
    }
    public void poleCenter()
    {
        /*
        while distance sensors aren't about equiv
            move left and right
        while the avg of distance sensors aren't 12 or less
            move forward
         */
        while(leftDistance.getDistance(DistanceUnit.CM) < rightDistance.getDistance(DistanceUnit.CM) - 0.5 || leftDistance.getDistance(DistanceUnit.CM) > rightDistance.getDistance(DistanceUnit.CM) + 0.5)
        {
            //turn a direction
            double value = Math.max(-0.15, Math.min(0.15, (leftDistance.getDistance(DistanceUnit.CM) - rightDistance.getDistance(DistanceUnit.CM)) / 5));
            leftBack.setPower(-value);
            leftFront.setPower(-value);
            rightBack.setPower(value);
            rightFront.setPower(value);
            telemetry.addData("turning portion", value);
        }
        while(((leftDistance.getDistance(DistanceUnit.CM) + rightDistance.getDistance(DistanceUnit.CM)) / 2) > 12)
        {
            //move forward
            //double value = 12 - ((leftDistance.getDistance(DistanceUnit.CM) + rightDistance.getDistance(DistanceUnit.CM)) / 2);
            leftBack.setPower(0.15);
            leftFront.setPower(0.15);
            rightBack.setPower(0.15);
            rightFront.setPower(0.15);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        linearOpMode.sleep(250);
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
        if (Math.abs(input) < 0.075)
        {
            return  0;
        }
        else
        {
            return input;
        }
    }
}
