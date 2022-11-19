package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name = "KaiImprovedComp")
public class KaiImprovedComp extends LinearOpMode {

    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;

    private boolean stickActive = false;
    private boolean hasChanged2;
    private double servoSet;
    BNO055IMU imu;


    Orientation angles;
    Acceleration gravity;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here
                //left encoder is leftBack; right encoder is rightBack; rear encoder is rightFront.


                if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
                    stickActive = true;
                }else{
                    stickActive = false;



                }
                if (stickActive){
                    rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
                    leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                    rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                    leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);

                }
                else
                {
                    setMotors(1,0);
                    if (gamepad1.dpad_up){
                        forward(1);
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

        while(opModeIsActive() && (-rightBack.getCurrentPosition()+leftBack.getCurrentPosition())/2.0<(794888*input) && !stickActive) {
            telemetry.addData("x",input);

            setMotors(1,0.5);

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
            rightBack.getCurrentPosition();
            leftBack.getCurrentPosition();
            lastInput = input;
            telemetry.update();
        }

        setMotors(1,0);

    }
    public void side (double input)
    {

        double lastInput = input;
        boolean hasChanged = false;
        stopReset();

        while(opModeIsActive() && rightFront.getCurrentPosition()<(794888*input) && !stickActive) {
            telemetry.addData("x",input);

            setMotors(2,0.5);

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
            rightBack.getCurrentPosition();
            leftBack.getCurrentPosition();
            lastInput = input;
            telemetry.update();
        }
        runEncoder();
        setMotors(1,0);

    }
    public void turn(double input)
    {
        stopReset();
        double goal1=(Math.ceil(getAngle()/45))*45; //nearest 45 degre angle to the right of current postion
        double goal2=(Math.floor(getAngle()/45))*45;//nearest 45 degre angle to the left of current postion
        if(input > 0){
            while (getAngle()<goal1){
                setMotors(3,0.5);
            }
        }else if(input < 0){
            while (getAngle()>goal2){
                setMotors(3,-0.5);
            }
        }
        setMotors(1,0);
    }
    public float getAngle() //get the current angle in 360 degree form
    {
        float angle = 0;
        if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<=0){
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }else if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >0){
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 360;
        }
        return angle;
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
    public void setMotors(int type, double power){
        //type 1 is +forward/-back, 2 is strafe -left/+right, 3 is turn -left/+right
        if (type==1){
            leftBack.setPower(power);
            rightBack.setPower(power);
            leftFront.setPower(power);
            rightFront.setPower(power);
        }else if (type==2){
            leftBack.setPower(power);
            rightBack.setPower(-power);
            leftFront.setPower(-power);
            rightFront.setPower(power);
        }else if (type==3){
            leftBack.setPower(power);
            rightBack.setPower(-power);
            leftFront.setPower(power);
            rightFront.setPower(-power);
        }

    }
}
