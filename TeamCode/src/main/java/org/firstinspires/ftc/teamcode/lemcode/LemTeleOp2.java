package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.archive.Odo2;

/**
 * This class is the primary teleOp for the 2022-2023 power-play season.
 * @author      Lemon
 */

@TeleOp(name = "LemTeleOp", group = "TeleOp")

public class LemTeleOp2 extends OpMode
{
    //Variables
    boolean singlePlayer = false;
    boolean grabbed = false;
    boolean beacon = false;
    double posMPCR = 1;
    double speedCoefficient = 0.6;
    int liftLevel = 0;
    double liftHeight = 0;
    boolean tempMPCR = false;
    int num = 0;
    int manual = 0;
    boolean hasZeroed = false;
    double grabberConstant = 0.54;
    double whenGrabbed;
    double lastTime;
    double time;
    double rLast;
    double lLast;
    double bad = 0;
    boolean cvd = false;
    double jeff = 0;
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
    Odo2 odometryTracker = new Odo2();

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
        liftZero();
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
    /**
     * This function tells the software to display information on the driver hub. It displays little information in two player mode to not confuse the drivers, but will display much more information in one player mode.
     */
    public void telemetry()
    {
        telemetry.addData("grabbed", grabbed);
        telemetry.addData("finger 1", "(" + gamepad1.touchpad_finger_1_x + ", " + gamepad1.touchpad_finger_1_y + ")");
        telemetry.addData("finger 2", "(" + gamepad1.touchpad_finger_2_x + ", " + gamepad1.touchpad_finger_2_y + ")");
        odometryTracker.setEncoderPos(encoderLeft.getCurrentPosition(), -encoderRight.getCurrentPosition(), encoderBack.getCurrentPosition());
        telemetry.addData("X position", odometryTracker.getX() / 2.54);
        telemetry.addData("Y position", odometryTracker.getY() / 2.54);
        telemetry.addData("heading", odometryTracker.getHDeg());

        if(singlePlayer) {
            telemetry.addData("Single player", singlePlayer);
            telemetry.addData("lift Level", liftLevel);
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
    /**
     * Resets all the drive encoders, which are actually used for odometer encoders.
     */
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
    /**
     * Takes an input and checks that it is less than 0.1. If it's below, it returns zero, otherwise it returns the original value.
     * This is intended to stop motors from being called with less power than makes them move.
     * @param input     what is the parameter for
     * @return          returns either 0 or the original value depending on whether its less then 0.1
     */
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
    /**
     * This function reads the distance sensors and spits out a direction to turn the robot in, to center the grabber on the junction
     * @return a double value to be added to the rx value in movement calculations
     */
    public double poleCenter()
    {
        double leftDist = leftDistance.getDistance(DistanceUnit.CM);
        double rightDist = rightDistance.getDistance(DistanceUnit.CM);
        if (grabbed && Math.abs((leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2) >= 800 + num) {
            if(leftDist < rightDist && leftDist < 17)
                return 0.05;
            if(rightDist < leftDist && rightDist < 17)
                return -0.05;
        } else
            return 0;
        return 0;
    }
    /**
     * I don't completely remember what purpose this serves, but it will take an input value and divide it by 200, then make it sure it stays between -1 and 1
     * @param input     is the value fed into the function
     * @return          returns the value divided by 200 and clipped to -1 to 1.
     */
    public double autoClip(double input)
    {
        return 0.5 * Math.max(Math.min(input/200,1),-1);
    }
    /**
     * Sets the threat to sleep for x milliseconds
     * @param input    is the amount of milliseconds to sleep for
     */
    public void sleep (double input)
    {
        try
        {
            Thread.sleep((long) input);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    /**
     * All the drive math needed to move the chassis around. It reads inputs off of the given game pad and sets the drive motors accordingly.
     * @param gamepadX     a choice between gamepad1 and gamepad2. Used for switching between single player and two player.
     * @param hasChangedX  each gamepad has an array of booleans to track when buttons have been pressed. This should correspond to the set game pad.
     */
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
    /**
     * This function controls the MultiPurpose Cone Righter. It automatically moves to position, unless told to move to the cone righting level.
     * @param gamepadX     is the gamepad used to activate the MPCR.
     * @param hasChangedX  is the boolean array to use the gamepad.
     */
    public void MPCRControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        //MPCR
        if((leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2 >= 1300 + num)
            posMPCR = 0.92;
        else if (leftLift.getCurrentPosition() + num <= 5)
            posMPCR = 0.55;
        else
            posMPCR = 0.45;

        if(tempMPCR)
            posMPCR = 0.2;
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

    /**
     * This function controls the grabber servos. It will close if open, and open if closed.
     * @param gamepadX     is the gamepad used to activate the grabber.
     * @param hasChangedX  is the boolean array to use the gamepad.
     */
    public void grabControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        double grabberAffector = (gamepadX.left_stick_x / 10);
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

        if(gamepadX.start)
        {
            if(!hasChangedX[13])
            {
                beacon = !beacon;
                hasChangedX[13] = true;
            }
        }
        else
        {
            hasChangedX[13] = false;
        }

        if(grabbed)
        {
            whenGrabbed = System.currentTimeMillis();
            if(beacon) {
                leftGrab.setPosition(1 - 0.5 - grabberAffector);
                rightGrab.setPosition(0.5 - grabberAffector);
            } else
            {
                leftGrab.setPosition(1 - 0.54 - grabberAffector);
                rightGrab.setPosition(0.54 - grabberAffector);
            }
        }
        else
        {
            leftGrab.setPosition(0.6);
            rightGrab.setPosition(0.3);
        }
        if(color.getDistance(DistanceUnit.INCH) < 0.5 && liftLevel == 0 && !grabbed)
        {
            gamepadX.runRumbleEffect(theresAConeRumble);
        }
    }
    /**
     * This function controls the lift. Pressing a, b, x, and y will move the lift to different levels. Pressing up and down on the d-pad will manually move the lift up and down.
     * @param gamepadX     is the gamepad used to activate the lift.
     * @param hasChangedX  is the boolean array to use the gamepad.
     */
    public void liftControl(Gamepad gamepadX, boolean[] hasChangedX)
    {
        //lift mechanism
        //initialize color sensor as distance sensor, detect when cones are at perfect distance? maybe make controller rumble
        //once a cone is grabbed, move lift to holding height and move MPCR into pole position
        //a is ground, b is low, y is medium, x is high
        //the button press includes moving to pole, depositing on pole, backing away from pole to starting position.
        //the lift then goes back to zero (bottom (grabbing position)) and the MPCR goes to standby position
        if(gamepadX.dpad_up)
            manual += 10;
        if(gamepadX.dpad_down)
            manual -= 10;

        if(gamepadX.a)
        {
            if(!hasChangedX[0])
            {
                if(grabbed)
                {
                    liftLevel = 1;
                    manual = 0;
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
                    manual = 0;
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
                    manual = 0;
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
                    manual = 0;
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
        {
            if (grabbed && System.currentTimeMillis() >= whenGrabbed + 500) {
                liftHeight = 84;
                manual = 0;
            } else {
                liftHeight = 30;
                manual = 0;
            }
        }

        lastTime = time;
        time = System.currentTimeMillis();

        if(liftLevel == 0 && (leftLift.getCurrentPosition() - lLast) / ((time - lastTime) / 1000) == 0)
        {
            bad = 0.0;
        }
        else
            bad = 1.0;

        lLast = leftLift.getCurrentPosition();

        if(gamepadX.left_trigger > 0)
            jeff = 250;
        if(gamepadX.left_trigger == 0)
            jeff = 0;
        if(liftLevel == 0)
            jeff = 0;

        leftLift.setTargetPosition((int) liftHeight + num + manual - (int) jeff);
        rightLift.setTargetPosition((int) liftHeight + num + manual - (int) jeff);

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
    /**
     * This function checks to see if the player mode needs to be changed. It will switch between single and two player, and rumble when a change has been made.
     * @param gamepadX     is the gamepad used to activate the MPCR.
     * @param hasChangedX  is the boolean array to use the gamepad.
     */
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
    public void liftZero ()
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
    }
}
