package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@Disabled
@TeleOp(name = "LemTeleOpDeprecated", group = "TeleOps")

//this will be the universal teleOp file. No functions need to be specific to one side (red v blue or 1 v 2)
public class LemTeleOp extends OpMode
{
    //variables
    double speedCoefficient = 0.5;
    boolean grabbed = false;
    boolean grabbedHasChanged  = false;
    double liftPosition = 0;
    double liftLevel = 0;

    //motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    private DcMotor leftLift;
    private DcMotor rightLift;
    //servos
    private Servo leftGrab;
    private Servo rightGrab;
    //sensors

    //IMU

    //camera

    //rumble
    Gamepad.RumbleEffect customRumbleEffect0;
    public void init()
    {
        //motors
        //initializeMotor(leftBack,"backLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //initializeMotor(rightBack,"backRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //initializeMotor(leftFront,"frontLeft");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //initializeMotor(rightFront,"frontRight");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //initializeMotor(leftLift,"liftLeft");
        leftLift = hardwareMap.get(DcMotorEx.class, "liftLeft");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //initializeMotor(rightLift,"liftRight");
        rightLift = hardwareMap.get(DcMotorEx.class, "liftRight");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        leftGrab = hardwareMap.get(Servo.class,"grabberLeft");
        rightGrab = hardwareMap.get(Servo.class, "grabberRight");
        //sensors

        //IMU

        //camera

        //rumbles
        customRumbleEffect0 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 125)
                .build();
    }
    public void loop()
    {
        gamepad1();
        gamepad2();
        telemetry();
    }
    public void gamepad1()
    {
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

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower = (leftBackPower * speedCoefficient * -1);
        rightBackPower = (rightBackPower * speedCoefficient * 1);
        leftFrontPower = (leftFrontPower  * speedCoefficient * -1);
        rightFrontPower = (rightFrontPower * speedCoefficient * 1);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

        telemetry.addData("leftBack",leftBackPower);
        telemetry.addData("rightBack",rightBackPower);
        telemetry.addData("leftFront",leftFrontPower);
        telemetry.addData("rightBack",rightBackPower);

        //lift
        /*
        grab
        cs2
        cs3
        cs4
        cs5
        gj / cs1
        lj
        mj
        hj
         */

        leftLift.setTargetPosition(-250);
        leftLift.setPower(0.8);
        rightLift.setTargetPosition(-250);
        rightLift.setPower(0.8);


        //grab
        if (gamepad1.right_bumper)
        {
            if (!grabbedHasChanged)
            {
                grabbed = !grabbed;
                grabbedHasChanged = true;
            }
        }
        else
        {
            grabbedHasChanged = false;
        }
        if (grabbed)
        {
            leftGrab.setPosition(1);
            rightGrab.setPosition(0);
        } else if (!grabbed)
        {
            leftGrab.setPosition(0.5);
            rightGrab.setPosition(0.5);
        }
        //speed coefficient

    }
    public void gamepad2()
    {
        
    }
    public void telemetry()
    {
        telemetry.addData("Color",1);
        telemetry.addData("Speed",100 * speedCoefficient + "%");
        telemetry.addData("lift height", "Left: " + leftLift.getCurrentPosition() + " and Right: " + rightLift.getCurrentPosition());
        telemetry.addData("grabbed",grabbed);
        telemetry.addData("servo1",leftGrab.getPosition());
        telemetry.addData("servo2",rightGrab.getPosition());
        telemetry.update();
    }
    //functions
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
    public void initializeMotor(DcMotor motor, String name)
    {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
