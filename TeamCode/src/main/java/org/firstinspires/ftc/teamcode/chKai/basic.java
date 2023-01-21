package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "basic")
public class basic extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private Servo rightGrab;
    private Servo leftGrab;
    private boolean hasTouched;
    private  double xChange;
    private double yChange;

    private ColorSensor color;


    private RevBlinkinLedDriver led;


    private boolean stickActive = false;

    private boolean hasChanged2;
    private double servoSet;
    private double lastSpeed;
    private NeedForSpeed s = new NeedForSpeed();

    @Override
    public void runOpMode() {
        
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        color = hardwareMap.get(ColorSensor.class,"color");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, " rightGrab");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        ElapsedTime time = new ElapsedTime();
        Led led = new Led();
        CDS cds = new CDS();
        Lift lift = new Lift();
        Grabber grabber = new Grabber();
        grabber.initializeHardware(hardwareMap );
        led.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
        lift.initializeHardware(hardwareMap);
        s.initializeHardware(hardwareMap);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double x = 0;
                double y = 0;
                if (gamepad1.touchpad_finger_1 && !hasTouched){
                    xChange = gamepad1.touchpad_finger_1_x;
                    yChange = gamepad1.touchpad_finger_1_y;
                }
                hasTouched = gamepad1.touchpad_finger_1;
                if (gamepad1.touchpad_finger_1 && hasTouched){
                    x = gamepad1.touchpad_finger_1_x - xChange;
                    y = gamepad1.touchpad_finger_1_y - yChange;
                }
                if (!gamepad1.touchpad_finger_1){
                    x = 0;
                    y = 0;
                }
                y = -y;
                leftFront.setPower((y - x - gamepad1.right_stick_x));
                leftBack.setPower((y + x - gamepad1.right_stick_x));
                rightFront.setPower((y + x + gamepad1.right_stick_x));
                rightBack.setPower((y - x + gamepad1.right_stick_x));

                telemetry.addLine(cds.colorTelemetry());
                telemetry.addData("distance",cds.getDistance());
                telemetry.addData("spedometer",spedometer());
                telemetry.addData("speed",s.getSpeedY(1));
                telemetry.update();
            }


        }

    }
    public double spedometer(){

        double t = Math.max(s.getSpeedY(1),lastSpeed);
        lastSpeed = t;
        return t;
    }
}