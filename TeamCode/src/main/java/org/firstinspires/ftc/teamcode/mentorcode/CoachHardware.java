package org.firstinspires.ftc.teamcode.mentorcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.VoltageSensor;

// import org.firstinspires.ftc.teamcode.mentorcode.XyhVector;
import org.firstinspires.ftc.teamcode.mentorcode.LimitSwitch;
import java.util.Locale;

/**
 * This file works in conjunction with the External Hardware Class called: OdometryPractise, which is based
 * on ConceptExternalHardwareClass.java. This file was based on RobotHardware.java sample from FTC samples.
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 */

public class CoachHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private LimitSwitch liftlimitlow = null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    private DcMotor rightback = null;

    private DcMotor encoderLeft = null;
    private DcMotor encoderRight = null;
    private DcMotor encoderAux = null;

    public enum MotorName {
        LEFTFRONT, LEFTBACK, RIGHTFRONT, RIGHTBACK
//        LEFTFRONT (0) {
//            @Override
//            public boolean isLeftFront() {
//                return true;
//            }
//        },
//        LEFTBACK (1) {
//            @Override
//            public boolean isLeftBack() {
//                return true;
//            }
//        },
//        RIGHTFRONT (2) {
//            @Override
//            public boolean isRightFront() {
//                return true;
//            }
//        }, RIGHTBACK (3) {
//            @Override
//            public boolean isRightBack() {
//                return true;
//            }
//        }
    }
    private MotorName motorname = null;

    private DcMotor armMotor = null;
    private Servo leftHand = null;
    private Servo rightHand = null;

    private Boolean hasOdometryWheels = false;

    public ElapsedTime opmodeRunTime = null;
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    // Define IMU public so they can be available by calling OpMode
    private BNO055IMU.Parameters imuParameters = null;

    // The IMU sensor object
    public BNO055IMU imu = null;

    // State used for updating telemetry
    private Orientation angles = null;

    public int loopCount = 0;

    public OdometryWheels odometrywheels = null;
//    private final static double L = 20.12;      // distance between encoder 1 and encoder 2 in cm
//    private final static double B = 11.5;       // distance between the midpoint of encoder 1 and encoder 2 and 3 in cm
//    private final static double R = 3.0;        // wheel radius in cm
//    private final static double N = 8192;       // encoder ticks per revolution REV encoder
//    private final static double cm_per_tick = 2.0 * Math.PI * R / N;
//
//    public int currentRightPosition = 0;
//    public int currentLeftPosition = 0;
//    public int currentAuxPosition = 0;
//
//    public int oldRightPosition = 0;
//    public int oldLeftPosition = 0;
//    public int oldAuxPosition = 0;
//
//    public XyhVector START_POS = null;
//    public XyhVector pos;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public CoachHardware(LinearOpMode opmode, Boolean odometryWheels) {
        myOpMode = opmode;
        hasOdometryWheels = odometryWheels;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Define and Initialize Motors
        leftfront = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightfront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftback = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightback = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");

//        // Define and Initialize Motors (note: need to use reference to actual OpMode).
//        armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // Reverses the two left motors
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake at power 0
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to run without encoders
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry Wheels assigned to motors on their ports
        //encoderLeft = rightfront;
        //encoderAux = rightback;
        //encoderRight = leftback;
        if (hasOdometryWheels) {
            // xLeft, xRight, center
            // vendor wheel says 35mm, actuals is between 33.8 and 34.2
            // because it's not stopping at 40cm, changing wheeldiameter to adjust
            // tried wheeldiameter = 3.4, 3.5, 3.55, closest is 3.5 on left side encoder
            // initial est trackwidth 22.8
            // measured trackwidth 34.5, right is one mm closer than left
            // wheel distance apart is 288mm, forward offset = ((288/2)+24+2)/10=17.0
            odometrywheels = new OdometryWheels(rightback, leftback, rightfront,
                    34.5, 17.8, 3.50, 8192);
        }

        // Set all motors to zero power
        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);

        resetEncoders();

        //SKJ remove 2022-12-14 liftlimitlow = new LimitSwitch(myOpMode, LimitSwitch.SwitchType.NC, "bottomLift");

        // Define and initialize ALL installed servos.
//        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
//        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
//        leftHand.setPosition(MID_SERVO);
//        rightHand.setPosition(MID_SERVO);

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        /* we keep track of how long it's been since the OpMode was started, just
         * to have some interesting data to show */
        opmodeRunTime = new ElapsedTime();

        // We show the log in oldest-to-newest order, as that's better for poetry
        myOpMode.telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        myOpMode.telemetry.log().setCapacity(10);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

        //START_POS = new XyhVector(213,102,Math.toRadians(-174));
        //pos = new XyhVector(213,102,Math.toRadians(-174));
    }

    public void onetimeFirstRun() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    public void runLoop() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        /** Update loop info and play nice with the rest of the {@link Thread}s in the system */
        loopCount++;

        setRobotTelemetry();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn  Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
//    public void driveRobot(double Drive, double Turn) {
//        // Combine drive and turn for blended motion.
//        double left = Drive + Turn;
//        double right = Drive - Turn;
//
//        // Scale the values so neither exceed +/- 1.0
//        double max = Math.max(Math.abs(left), Math.abs(right));
//        if (max > 1.0) {
//            left /= max;
//            right /= max;
//        }
//
//        // Use existing function to drive both wheels.
//        setDrivePower(left, right);
//    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }

    public void setRobotTelemetry() {
        /**
         //         * As an illustration, the first line on our telemetry display will display the battery voltage.
         //         * The idea here is that it's expensive to compute the voltage (at least for purposes of illustration)
         //         * so you don't want to do it unless the data is <em>actually</em> going to make it to the
         //         * driver station (recall that telemetry transmission is throttled to reduce bandwidth use.
         //         * Note that getBatteryVoltage() below returns 'Infinity' if there's no voltage sensor attached.
         //         *
         //         * @see Telemetry#getMsTransmissionInterval()
         //         */
        myOpMode.telemetry.addLine("robot | ")
                .addData("volts", " %.1f ", getRobotBatteryVoltage())
                .addData("loop count", " %d ", loopCount)
                .addData("ms/loop", " %.3f ms", opmodeRunTime.milliseconds() / loopCount);

        myOpMode.telemetry.addLine("IMU | ")
                .addData("status", imu.getSystemStatus().toShortString());

        myOpMode.telemetry.addLine("IMU Pose | ")
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

        myOpMode.telemetry.addLine("Chassis Power | ")
                .addData("leftfront", "%.2f", leftfront.getPower())
                .addData("rightfront", "%.2f", rightfront.getPower())
                .addData("leftback", "%.2f", leftback.getPower())
                .addData("rightback", "%.2f", rightback.getPower());
        //SKJ remove 2022-12-14 myOpMode.telemetry.addLine(liftlimitlow.toString());

        myOpMode.telemetry.addLine("Odometry current pos | ")
                .addData("leftX",  "%d", odometrywheels.left_encoder_pos)
                .addData("rightX",  "%d", odometrywheels.right_encoder_pos)
                .addData("Y",  "%d", odometrywheels.center_encoder_pos);

        myOpMode.telemetry.addLine("Odometry prev pos | ")
                .addData("leftX",  "%d", odometrywheels.prev_left_encoder_pos)
                .addData("rightX",  "%d", odometrywheels.prev_right_encoder_pos)
                .addData("Y",  "%d", odometrywheels.prev_center_encoder_pos);

        myOpMode.telemetry.addLine("Odometry current cm | ")
                .addData("cm_per_tick",  "%.5f", odometrywheels.cm_per_tick)
                .addData("leftX",  "%.2f", odometrywheels.leftX_cm)
                .addData("rightX",  "%.2f", odometrywheels.rightX_cm)
                .addData("Y",  "%.2f", odometrywheels.y_cm);

        /* SKJ troubleshoot
        myOpMode.telemetry.addLine("Odometry pos | ")
                .addData("x(cm)", "%.2f", odometrywheels.pos.x_cm)
                .addData("y(cm)", "%.2f", odometrywheels.pos.y_cm);

        myOpMode.telemetry.addLine("Odometry enc | ")
                .addData("x", "%.2f", odometrywheels.pos.getX())
                .addData("y", "%.2f", odometrywheels.pos.getY())
                .addData("h", "%.2f", Math.toDegrees(odometrywheels.pos.h));

         */
        /* do the telemetry update inside main opmode, instead of here inside robot hardware class */
        // myOpMode.telemetry.update();

    }
    //----------------------------------------------------------------------------------------------
    // Formatting IMU Angles
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // Computes the current battery voltage
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : myOpMode.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private Double getRobotBatteryVoltage() {
        return getBatteryVoltage();
    }

    public void resetEncoders() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!hasOdometryWheels) { // odometry wheel encoders should not be stopped
            leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    // Sets the power on single motor (use for testing)
    public void setSingleMotorPower(int motorport, double power) {
        switch(motorport) {
            case 0: // LEFTFRONT:
                leftfront.setPower(power);
                break;
            case 1: //LEFTBACK:
                leftback.setPower(power);
                break;
            case 2: //RIGHTFRONT:
                rightfront.setPower(power);
                break;
            case 3: //RIGHTBACK:
                rightback.setPower(power);
                break;
        }
    }
    // Sets the power on the motors directly based on a left and right value
    public void setSimplePower(double left, double right) {
        leftfront.setPower(left);
        rightfront.setPower(right);
        leftback.setPower(left);
        rightback.setPower(right);
    }
}
