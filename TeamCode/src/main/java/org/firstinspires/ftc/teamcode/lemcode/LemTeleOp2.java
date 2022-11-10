package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;

@TeleOp(name = "LemTeleOp", group = "TeleOp")

public class LemTeleOp2 extends OpMode
{
    //Variables
    boolean singlePlayer = false;
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
    double lastTime;
    double time;
    double rLast;
    double lLast;
    double bad = 0;
    boolean cvd = false;
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
    15 - left trigger
    16 - right trigger
     */
    boolean[] hasChanged1 = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    boolean[] hasChanged2 = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    Gamepad control1 = gamepad1;
    Gamepad control2 = gamepad2;
    boolean[] changed1 = hasChanged1;
    boolean[] changed2 = hasChanged2;


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
    //private ColorSensor color;

    //Touch Sensors
    private TouchSensor zero;

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Rumbles
    Gamepad.RumbleEffect singlePlayerRumble;
    Gamepad.RumbleEffect theresAConeRumble;

    //Odometry
    Odo1 odometryTracker = new Odo1();

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
        //color = hardwareMap.get(ColorSensor.class, "color");

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

        //Rumbles
        singlePlayerRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)
                .build();

        theresAConeRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0,1.0,1000)
                .build();
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
        playerCheck(gamepad1,hasChanged1);
        if(!singlePlayer)
        {
            control1 = gamepad1;
            changed1 = hasChanged1;
            control2 = gamepad2;
            changed2 = hasChanged2;
        }
        else
        {
            control1 = gamepad1;
            changed1 = hasChanged1;
            control2 = gamepad1;
            changed2 = hasChanged2;
        }
        driveControl(control1,hasChanged1);
        MPCRControl(control1,hasChanged1);
        grabControl(control2,hasChanged2);
        liftControl(control2,hasChanged2);
        telemetry();
    }
    public void telemetry()
    {
        telemetry.addData("grabbed", grabbed);
        if(singlePlayer) {
            telemetry.addData("Single player", singlePlayer);
            telemetry.addData("lift Level", liftLevel);
            odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
            telemetry.addData("X position", odometryTracker.getX());
            telemetry.addData("Y position", odometryTracker.getY());
            telemetry.addData("heading", odometryTracker.getHDeg());
            telemetry.addData("encoderLeft", encoderLeft.getCurrentPosition());
            telemetry.addData("encoderRight", encoderRight.getCurrentPosition());
            telemetry.addData("encoderBack", encoderBack.getCurrentPosition());
            telemetry.addData("color", color.getDistance(DistanceUnit.INCH));
            //telemetry.addData("color", "red: " + color.red() + " green: " + color.green() + " blue: " + color.blue());
            telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("angles", angles);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("MPCR Position", posMPCR);
            telemetry.addData("leftServo", leftMPCR.getPosition());
            telemetry.addData("rightServo", rightMPCR.getPosition());
            telemetry.addData("lift height", "Left: " + leftLift.getCurrentPosition() + " and Right: " + rightLift.getCurrentPosition());
            telemetry.addData("liftpower: ", leftLift.getPower() + " " + rightLift.getPower());
            telemetry.addData("Speed", 100 * speedCoefficient + "%");
            telemetry.addData("zero", zero.isPressed());
            telemetry.addData("left lift current", ((DcMotorEx) leftLift).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right lift current", ((DcMotorEx) rightLift).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left lift velocity", ((DcMotorEx) leftLift).getVelocity());
            telemetry.addData("right lift current", ((DcMotorEx) rightLift).getVelocity());
        }
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
    public void driveControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        //drive code
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double x = gamepadX.left_stick_x;
        double y = -gamepadX.left_stick_y;
        double rx = gamepadX.right_stick_x;

        x = deadZone(x);
        y = deadZone(y);
        rx = deadZone(rx);

        //rx += poleCenter();

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
        if(gamepadX.right_trigger > 0.5)
        {
            if(!hasChangedX[16])
            {
                if(speedCoefficient < 1)
                    speedCoefficient += 0.2;
                hasChangedX[16] = true;
            }
        }
        else
        {
            hasChangedX[16] = false;
        }
        if(gamepadX.left_trigger > 0.5)
        {
            if(!hasChanged1[15])
            {
                if(speedCoefficient > 0.2)
                    speedCoefficient -= 0.2;
                hasChangedX[15] = true;
            }
        }
        else
        {
            hasChangedX[15] = false;
        }
    }
    public void MPCRControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        //MPCR
        if((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2 >= 1300 + num)
            posMPCR = 1;
        else
            posMPCR = 0.45;
        if(tempMPCR)
            posMPCR = 0.14;
        if(gamepadX.left_bumper)
        {
            if(!hasChangedX[10])
            {
                tempMPCR = !tempMPCR;
                hasChangedX[10] = true;
            }
        }
        else
        {
            hasChangedX[10] = false;
        }
        rightMPCR.setPosition(posMPCR + 0.02);
        leftMPCR.setPosition(1 - posMPCR - 0.02);
    }
    public void grabControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        //Grabber right bumper
        if(gamepadX.right_bumper)
        {
            if(!hasChangedX[11])
            {
                grabbed = !grabbed;
                hasChangedX[11] = true;
            }
        }
        else
        {
            hasChangedX[11] = false;
        }
        if(grabbed)
        {
            whenGrabbed = System.currentTimeMillis();
            leftGrab.setPosition(grabberConstant);
            rightGrab.setPosition(1 - grabberConstant);
        }
        else

        {
            leftGrab.setPosition(0);
            rightGrab.setPosition(1);
        }
        if(color.getDistance(DistanceUnit.INCH) < 0.5 && liftLevel == 0 && !grabbed)
        {
            gamepadX.runRumbleEffect(theresAConeRumble);
        }
    }
    public void liftControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        //lift mechanism
        //initialize color sensor as distance sensor, detect when cones are at perfect distance? maybe make controller rumble
        //once a cone is grabbed, move lift to holding height and move MPCR into pole position
        //a is ground, b is low, y is medium, x is high
        //the button press includes moving to pole, depositing on pole, backing away from pole to starting position.
        //the lift then goes back to zero (bottom (grabbing position)) and the MPCR goes to standby position
        if(gamepadX.a)
        {
            if(!hasChangedX[0])
            {
                if(grabbed)
                {
                    liftLevel = 1;
                    liftHeight = (260);
                }
                if(!grabbed)
                {
                    liftLevel = 0;

                }
                hasChangedX[0] = true;
            }
        }
        else
        {
            hasChangedX[0] = false;
        }
        if(gamepadX.b)
        {
            if(!hasChangedX[1])
            {
                if(grabbed)
                {
                    liftLevel = 2;
                    liftHeight = (1330);
                }
                if(!grabbed)
                    liftLevel = 0;
                hasChangedX[1] = true;
            }
        }
        else
        {
            hasChangedX[1] = false;
        }
        if(gamepadX.x)
        {
            if(!hasChangedX[2])
            {
                if(grabbed)
                {
                    liftLevel = 3;
                    liftHeight = (2150);
                }
                if(!grabbed)
                    liftLevel = 0;
                hasChangedX[2] = true;
            }
        }
        else
        {
            hasChangedX[2] = false;
        }
        if(gamepadX.y)
        {
            if(!hasChangedX[3])
            {
                if(grabbed)
                {
                    liftLevel = 4;
                    liftHeight = (3000);
                }
                if(!grabbed)
                    liftLevel = 0;
                hasChangedX[3] = true;
            }
        }
        else
        {
            hasChangedX[3] = false;
        }
        if(liftLevel == 0)
            if(grabbed &&  System.currentTimeMillis() >= whenGrabbed + 500)
                liftHeight = 84;
            else
                liftHeight = 30;


        lastTime = time;
        time = System.currentTimeMillis();

        if(liftLevel == 0 && (leftLift.getCurrentPosition() - lLast) / ((time - lastTime) / 1000) == 0)
        {
            bad = 0.0;
        }
        else
            bad = 1.0;

        lLast = leftLift.getCurrentPosition();

        leftLift.setTargetPosition((int) liftHeight + num);
        rightLift.setTargetPosition((int) liftHeight + num);

        int actual = Math.abs((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2);
        int rActual = Math.abs(rightLift.getCurrentPosition());
        int lActual = Math.abs(leftLift.getCurrentPosition());

        if (Math.abs(liftHeight) > actual)
        {
            if (lActual > rActual) {
                leftLift.setPower(0.715 * bad);
                rightLift.setPower(0.685);
            } else if (lActual < rActual) {
                leftLift.setPower(0.685 * bad);
                rightLift.setPower(0.715);
            } else {
                leftLift.setPower(0.7 * bad);
                rightLift.setPower(0.7);
            }
        }
        if (Math.abs(liftHeight) < actual)
        {
            if (lActual > rActual) {
                leftLift.setPower(0.685 * bad);
                rightLift.setPower(0.715);
            } else if (lActual < rActual) {
                leftLift.setPower(0.715 * bad);
                rightLift.setPower(0.685);
            } else {
                leftLift.setPower(0.7 * bad);
                rightLift.setPower(0.7);
            }
        }
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void playerCheck(Gamepad gamepadX, boolean[] hasChangedX) {
        if (gamepadX.start && gamepadX.y) {
            if (!hasChangedX[13] && !hasChangedX[3]) {

                singlePlayer = !singlePlayer;
                gamepadX.runRumbleEffect(singlePlayerRumble);
                hasChangedX[13] = true;
                hasChangedX[3] = true;
            } else {
                hasChangedX[13] = false;
                hasChangedX[3] = false;
            }
        }
    }
}
