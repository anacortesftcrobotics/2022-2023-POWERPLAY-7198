package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "KaiImprovedTeleOp (Blocks to Java)")
public class KaiImprovedTeleOp extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DistanceSensor distance;

    private Servo scythe;
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
        rightBack = hardwareMap.get(DcMotor.class, "frontLeft");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        distance.getDistance(DistanceUnit.CM);
        scythe = hardwareMap.get(Servo.class, "scythe");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit =BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu=hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here

                telemetry.addData("distance",distance.getDistance(DistanceUnit.CM));

                if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                    stickActive = true;
                }else{
                    stickActive = false;
                    if(gamepad1.a && !hasChanged2)
                    {

                        if(servoSet == 1){
                            servoSet = 0;
                        }else if (servoSet == 0){
                            servoSet = 1;
                        }
                        hasChanged2 = true;
                    }else
                    {
                        hasChanged2 = false;
                    }
                    scythe.setPosition(servoSet);

                }
                if (stickActive){
                    leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
                    leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                    rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                    rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);

                }
                else
                {
                    setMotors(0);
                    if (gamepad1.dpad_up){
                        forward(-1);
                    }else if (gamepad1.dpad_down){
                        forward(1);
                    }else if (gamepad1.dpad_left){
                        side(1);
                    }else if (gamepad1.dpad_right){
                        side(-1);
                    }else if (gamepad1.right_bumper){
                        turn(1);
                    }else if (gamepad1.left_bumper){
                        turn(-1);
                    }else if(gamepad1.right_stick_button){
                        getCone();

                    }else if (gamepad1.x){
                        center();

                    }


                    telemetry.update();
                }
            }
        }
    }
    public void forward (double input)
    {

        double lastInput = input;
        boolean hasChanged = false;
        stopReset();

        leftBack.setTargetPosition((int)(input*850));
        rightBack.setTargetPosition((int)(input*850));
        leftFront.setTargetPosition((int)(input*850));
        rightBack.setTargetPosition((int)(input*850));

        runPosition();

        setMotors(0.5);
        input =0;
        while(opModeIsActive() && leftBack.isBusy() && !stickActive) {
            telemetry.addData("x",input);
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                stickActive = true;
            }else{
                stickActive = false;
            }

            if (gamepad1.dpad_up && !hasChanged){
                input += 1;
                hasChanged = true;
            }else if (gamepad1.dpad_down &&!hasChanged){
                input -=1;
                hasChanged = true;
            }else if (!gamepad1.dpad_down && !gamepad1.dpad_up){
                hasChanged = false;
            }
            if(input!=lastInput){
                leftBack.setTargetPosition((int)(input*850));
                rightBack.setTargetPosition((int)(input*850));
                leftFront.setTargetPosition((int)(input*850));
                rightFront.setTargetPosition((int)(input*850));
            }
            lastInput = input;
            telemetry.update();
        }
        runEncoder();
        setMotors(0);

    }
    public void side(double input)
    {

        double lastInput = input;
        boolean hasChanged = false;
        stopReset();

        leftBack.setTargetPosition((int)(-input*850));
        rightBack.setTargetPosition((int)(input*850));
        leftFront.setTargetPosition((int)(input*850));
        rightFront.setTargetPosition((int)(-input*850));

        runPosition();

        setMotors(0.5);
        input =0;
        while(opModeIsActive() && leftBack.isBusy() && !stickActive) {
            telemetry.addData("x",input);
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                stickActive = true;
            }else{
                stickActive = false;
            }

            if (gamepad1.dpad_left && !hasChanged){
                input += 1;
                hasChanged = true;
            }else if (gamepad1.dpad_right &&!hasChanged){
                input -=1;
                hasChanged = true;
            }else if (!gamepad1.dpad_left && !gamepad1.dpad_right){
                hasChanged = false;
            }
            if(input!=lastInput){
                leftBack.setTargetPosition((int)(-input*850));
                rightBack.setTargetPosition((int)(input*850));
                leftFront.setTargetPosition((int)(input*850));
                rightFront.setTargetPosition((int)(-input*850));
            }
            lastInput = input;
            telemetry.update();
        }
        runEncoder();
        setMotors(0);
    }
    public void turn(double input)
    {
        double lastInput = input;
        boolean hasChanged = false;
        stopReset();

        leftBack.setTargetPosition((int)(input*850));
        rightBack.setTargetPosition((int)(-input*850));
        leftFront.setTargetPosition((int)(input*850));
        rightFront.setTargetPosition((int)(-input*850));

        runPosition();

        setMotors(0.5);
        input =0;
        while(opModeIsActive() && leftBack.isBusy() && !stickActive) {
            telemetry.addData("x",input);
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                stickActive = true;
            }else{
                stickActive = false;
            }

            if (gamepad1.right_bumper && !hasChanged){
                input += 1;
                hasChanged = true;
            }else if (gamepad1.left_bumper &&!hasChanged){
                input -=1;
                hasChanged = true;
            }else if (!gamepad1.right_bumper && !gamepad1.left_bumper){
                hasChanged = false;
            }
            if(input!=lastInput){
                leftBack.setTargetPosition((int)(input*850));
                rightBack.setTargetPosition((int)(-input*850));
                leftFront.setTargetPosition((int)(input*850));
                rightFront.setTargetPosition((int)(-input*850));
            }
            lastInput = input;
            telemetry.update();
        }
        runEncoder();
        setMotors(0);
    }
    public void getCone()
    {
        while (distance.getDistance(DistanceUnit.CM)>3 && !stickActive){
            scythe.setPosition(0);

            setMotors(-0.3);
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                stickActive = true;
            }else{
                stickActive = false;
            }
        }
        setMotors(0);
        scythe.setPosition(1);

    }
    public void center(){
        double goal1 = Math.round(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle/90.0);
        double goal2 = 0;
        if (goal1 == 0){
            goal2 = 0;
        }else if(goal1 == 1){
            goal2 = 1;
        }else if(goal1 == 2 || goal1 == -2){
            goal2 = 2;
        }else if(goal1 == -1){
            goal2 = 3;
        }
        double heading1 =(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle/90.0);

        while(heading1 != goal2&&!stickActive&&opModeIsActive()){

            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                stickActive = true;
            }else{
                stickActive = false;
            }
            if (heading1 > goal2){
                leftBack.setPower(0.1);

                rightBack.setPower(-0.1);

                leftFront.setPower(0.1);

                rightFront.setPower(-0.5);

            }
            if (heading1 < goal2){
                leftBack.setPower(-0.1);

                rightBack.setPower(0.1);

                leftFront.setPower(-0.1);

                rightFront.setPower(0.1);
            }

        }
        setMotors(0);
    }
    public void stopReset(){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runPosition(){
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runEncoder(){
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotors(double input){
        leftBack.setPower(input);
        rightBack.setPower(input);
        leftFront.setPower(input);
        rightFront.setPower(input);
    }
}
