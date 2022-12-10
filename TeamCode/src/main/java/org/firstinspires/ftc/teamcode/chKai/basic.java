package org.firstinspires.ftc.teamcode.chKai;

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

    private ColorSensor color;


    private RevBlinkinLedDriver led;


    private boolean stickActive = false;

    private boolean hasChanged2;
    private double servoSet;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
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

        Led led = new Led();
        CDS cds = new CDS();
        Lift lift = new Lift();
        led.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
        boolean grabbed= false;
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                boolean ledType = true;
                leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
                leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
                rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
                rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/2);

                if (gamepad1.right_bumper) {
                    grabbed = true;
                }

                if (gamepad1.left_bumper){
                    grabbed = false;

                }

                if (grabbed){
                    if(cds.getDistance()<1){
                        leftGrab.setPosition(0.46);
                        rightGrab.setPosition(0.54);
                        led.setLed("green");
                    }else {
                        led.setLed("violet");
                        leftGrab.setPosition(0.6);
                        rightGrab.setPosition(0.3);
                        lift.liftSet(6);
                    }
                } else if (!grabbed) {
                    led.setLed("violet");
                    leftGrab.setPosition(0.6);
                    rightGrab.setPosition(0.3);
                    lift.liftSet(0);
                }
                if (gamepad1.a){
                    lift.liftSet(6);
                }
                if (gamepad1.b){
                    lift.liftSet(7);
                }
                if (gamepad1.x){
                    lift.liftSet(8);
                }
                if (gamepad1.y){
                    lift.liftSet(9);
                }


                telemetry.addLine(cds.colorTelemetry());
                telemetry.addData("grabbed", grabbed);
                //telemetry.addLine(p.distanceTelemetry());
                telemetry.update();
            }


        }

    }
}