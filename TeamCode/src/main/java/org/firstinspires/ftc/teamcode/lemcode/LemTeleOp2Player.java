package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name = "LemTeleOp2Player", group = "TeleOps")

public class LemTeleOp2Player extends OpMode
{
    //Variables
    boolean grabbed = false;
    double posMPCR = 1;
    double speedCoefficient = 0.6;
    int liftLevel = 0;
    double liftHeight = 0;
    double artLiftHeight = 0;
    boolean tempMPCR = false;
    int num = 0;
    boolean hasZeroed = false;
    double grabberConstant = 0.60;
    double whenGrabbed;
    /*
    the hasChanged arrays are for the boolean buttons on the controllers.
    0 - a
    1 - b
    2 - x
    3 - y
    4 - dpad_down
    5 - dpad_right
    6 - dpad_left
    7 - dpad_up
    8 - left_stick_button
    9 - right_stick_button
    10 - left_bumper
    11 - right_bumper
    12 - back
    13 - start
    14 - guide
     */
    boolean[] hasChanged1 = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    boolean[] hasChanged2 = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront, leftLift, rightLift;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;

    //Servos
    private Servo leftGrab, rightGrab, leftMPCR, rightMPCR;

    //Distance Sensors
    private DistanceSensor leftDistance, rightDistance;

    //Color Sensors
    private DistanceSensor color;

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
        color = hardwareMap.get(DistanceSensor.class, "color");

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
    public void loop()
    {
        if(!hasZeroed)
        {
            num = 0;
            while (!zero.isPressed())
            {
                leftLift.setTargetPosition(num);
                rightLift.setTargetPosition(num);

                leftLift.setPower(0.8);
                rightLift.setPower(0.8);

                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                num -= 2;
            }
            hasZeroed = true;
        }
        gamepad1();
        gamepad2();
        telemetry();
    }
    public void gamepad1()
    {
        //drive code
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        x = deadZone(x);
        y = deadZone(y);
        rx = deadZone(rx);

        rx += poleCenter();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower = Math.cbrt(leftBackPower);
        rightBackPower = Math.cbrt(rightBackPower);
        leftFrontPower = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBackPower = (leftBackPower * speedCoefficient * 0.75);
        rightBackPower = (rightBackPower * speedCoefficient * 0.75);
        leftFrontPower = (leftFrontPower  * speedCoefficient * 0.75);
        rightFrontPower = (rightFrontPower * speedCoefficient * 0.75);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

        //Speed coefficient dpad up n down
        if(gamepad1.dpad_up)
        {
            if(!hasChanged1[7])
            {
                if(speedCoefficient < 1)
                    speedCoefficient += 0.2;
                hasChanged1[7] = true;
            }
        }
        else
        {
            hasChanged1[7] = false;
        }
        if(gamepad1.dpad_down)
        {
            if(!hasChanged1[4])
            {
                if(speedCoefficient > 0.2)
                    speedCoefficient -= 0.2;
                hasChanged1[4] = true;
            }
        }
        else
        {
            hasChanged1[4] = false;
        }

        //MPCR
        if((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2 >= 1200 + num)
            posMPCR = 0;
        else
            posMPCR = 0.6;
        if(tempMPCR)
            posMPCR = 0.17;
        if(gamepad1.left_bumper)
        {
            if(!hasChanged1[10])
            {
                tempMPCR = !tempMPCR;
                hasChanged1[10] = true;
            }
        }
        else
        {
            hasChanged1[10] = false;
        }
        rightMPCR.setPosition(posMPCR + 0.02);
        leftMPCR.setPosition(1 - posMPCR - 0.02);
    }
    public void gamepad2()
    {
        //Grabber right bumper
        if(gamepad2.right_bumper)
        {
            if(!hasChanged2[11])
            {
                grabbed = !grabbed;
                hasChanged2[11] = true;
            }
        }
        else
        {
            hasChanged2[11] = false;
        }
        if(grabbed)
        {
            whenGrabbed = System.currentTimeMillis();
            leftGrab.setPosition(grabberConstant + 0.02);
            rightGrab.setPosition(1 - grabberConstant + 0.02);
        }
        else
        {
            leftGrab.setPosition(0);
            rightGrab.setPosition(1);
        }
        /*
        if(color.getDistance(DistanceUnit.INCH) < 0.5 && liftLevel == 0 && !grabbed)
        {
            grabbed = true;
        }
        */
        //lift mechanism
        //initialize color sensor as distance sensor, detect when cones are at perfect distance? maybe make controller rumble
        //once a cone is grabbed, move lift to holding height and move MPCR into pole position
        //a is ground, b is low, y is medium, x is high
        //the button press includes moving to pole, depositing on pole, backing away from pole to starting position.
        //the lift then goes back to zero (bottom (grabbing position)) and the MPCR goes to standby position
        if(gamepad2.a)
        {
            if(!hasChanged2[0])
            {
                if(grabbed)
                {
                    liftLevel = 1;
                    liftHeight = (252);
                }
                if(!grabbed)
                {
                    liftLevel = 0;

                }
                hasChanged2[0] = true;
            }
        }
        else
        {
            hasChanged2[0] = false;
        }
        if(gamepad2.b)
        {
            if(!hasChanged2[1])
            {
                if(grabbed)
                {
                    liftLevel = 2;
                    liftHeight = (1302);
                }
                if(!grabbed)
                    liftLevel = 0;
                hasChanged2[1] = true;
            }
        }
        else
        {
            hasChanged2[1] = false;
        }
        if(gamepad2.x)
        {
            if(!hasChanged2[2])
            {
                if(grabbed)
                {
                    liftLevel = 3;
                    liftHeight = (2142);
                }
                if(!grabbed)
                    liftLevel = 0;
                hasChanged2[2] = true;
            }
        }
        else
        {
            hasChanged2[2] = false;
        }
        if(gamepad2.y)
        {
            if(!hasChanged2[3])
            {
                if(grabbed)
                {
                    liftLevel = 4;
                    liftHeight = (3066);
                }
                if(!grabbed)
                    liftLevel = 0;
                hasChanged2[3] = true;
            }
        }
        else
        {
            hasChanged2[3] = false;
        }
        if(liftLevel == 0)
            if(grabbed &&  System.currentTimeMillis() >= whenGrabbed + 500)
                liftHeight = 84;
            else
                liftHeight = 20;

        leftLift.setTargetPosition((int) liftHeight + num);
        rightLift.setTargetPosition((int) liftHeight + num);

        int actual = Math.abs((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2);
        int rActual = Math.abs(rightLift.getCurrentPosition());
        int lActual = Math.abs(leftLift.getCurrentPosition());

        if (Math.abs(liftHeight) > actual)
        {
            if (lActual > rActual) {
                leftLift.setPower(0.715);
                rightLift.setPower(0.685);
            } else if (lActual < rActual) {
                leftLift.setPower(0.685);
                rightLift.setPower(0.715);
            } else {
                leftLift.setPower(0.7);
                rightLift.setPower(0.7);
            }
        }
        if (Math.abs(liftHeight) < actual)
        {
            if (lActual > rActual) {
                leftLift.setPower(0.685);
                rightLift.setPower(0.715);
            } else if (lActual < rActual) {
                leftLift.setPower(0.715);
                rightLift.setPower(0.685);
            } else {
                leftLift.setPower(0.7);
                rightLift.setPower(0.7);
            }
        }
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void telemetry()
    {
        telemetry.addData("grabbed",grabbed);
        telemetry.addData("Speed",100 * speedCoefficient + "%");
        telemetry.addData("dist: ",color.getDistance(DistanceUnit.INCH));
        telemetry.update();
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
        if (Math.abs(input) < 0.1)
        {
            return  0;
        }
        else
        {
            return input;
        }
    }
    public double poleCenter()
    {
        if(grabbed && Math.abs((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2) >= 800 + num)
        {
            if(Math.min(leftDistance.getDistance(DistanceUnit.CM) ,17) < 17)
                return 0.05;
            else if(Math.min(rightDistance.getDistance(DistanceUnit.CM) ,17) < 17)
                return -0.05;
            else
                return 0;
        }
        else
            return 0;
    }
    public double autoClip(double input)
    {
        return 0.5 * Math.max(Math.min(input/200,1),-1);
    }
    public void sleep (double input)
    {
        try
        {
            Thread.sleep((long) input);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
