/*package org.firstinspires.ftc.teamcode.chKai;

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


@TeleOp(name = "odoTest")
public class odoTest extends LinearOpMode {

    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;

    private int positionx = 0;
    private int positiony = 0;
    BNO055IMU imu;


    Orientation angles;
    Acceleration gravity;

    
    @Override
    public void runOpMode() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack.setMode(RUN_USING_ENCODERS);
        leftBack.setMode(RUN_USING_ENCODERS);
        rightFront.setMode(RUN_USING_ENCODERS);
        leftFront.setMode(RUN_USING_ENCODERS);
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
            while (opModeIsActive()) {
                //left encoder is leftBack; right encoder is rightBack; rear encoder is rightFront.
                rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
                leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
                telemetry.addData("X",getX());
                telemetry.addData("Y",getY());
                telemetry.addData("angle",getAngle());

                telemetry.update();
            }
        }
    }
    public double getAngle(){ //gets angle in radian form. 0* is 90* to the right of the starting orientation, and value adds when turning counterclockwise.
        //use for trig stuff
        float angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double angle2 = Math.toRadians(angle1)

        return angle2;
    }
    public int getX(){ //gets current position on the x-axis. Assumes that starting point is 0,0. Must be continually called.
        int x = positionx;
        //X +=(-sin(f)+cos(s))
        x += ((-leftBack.getCurrentPosition()+rightBack.getCurrentPosition())/2)*(-Math.sin(getAngle()));
        x += ((rightFront.getCurrentPosition())/2)*Math.cos(getAngle());
        resetEncoders();
        positionx = x;
        return x;
    }
    public int getY(){ //gets current position on the y-axis. Assumes that starting point is 0,0. Must be continually called.
        int y = positiony;
        //Y +=(cos(f)+sin(s))
        y += ((-leftBack.getCurrentPosition()+rightBack.getCurrentPosition())/2)*Math.cos(getAngle());
        y += ((rightFront.getCurrentPosition())/2)*Math.sin(getAngle());
        resetEncoders();
        positiony = y;
        return y;
    }
    public double getR(){
        float angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        float angle2 = 0;
        if (angle1<0){
            angle2=angle1+450;
        }else if (angle1>=0){
            angle2=angle1+90;
        }
        return angle2;
    }
    public int goalX(int a){
        //if a == 0, floor, if 1, round, if 2, ciel
        double goal = 0;
        if (a==0){
            goal = (Math.floor(getX()/84000))*84000;
            if (goal-getX()<3000 && goal-getX()<=0){
                goal -=1;
            }
        }else if (a==2){
            goal = (Math.ceil(getX()/84000))*84000;
            if (goal-getX()<3000&&goal-getX()>=0){
                goal +=1;
            }
        }else if (a==1){
            goal = (Math.round(getX()/84000))*84000;
        }
        return (int) goal;
    }
    public int goalY(int a){
        //if a == 0, floor, if 1, round, if 2, ciel
        double goal = 0;
        if (a==0){
            goal = (Math.floor(getY()/84000))*84000;
            if (goal-getY()<3000&&goal-getY()<=0){
                goal -=1;
            }
        }else if (a==2){
            goal = (Math.ceil(getY()/84000))*84000;
            if (goal-getY()<3000&&goal-getY()<=0){
                goal +=1;
            }
        }else if (a==1){
            goal = (Math.round(getY()/84000))*84000;
        }
        return (int) goal;
    }
    public int goalR(int a){
        //if a == 0, floor, if 1, round, if 2, ciel
        double goal=0;
        if (a==0){
            goal = (Math.floor(getR()/45))*45;
            if (goal-getR()<5&&goal-getR()<=0){
                goal -=1;
            }
        }else if (a==2){
            goal = (Math.ceil(getR()/45))*45;
            if (goal-getR()<5&&goal-getR()>=0){
                goal +=1;
            }
        }else if (a==1){
            goal = (Math.round(getR()/45))*45;
        }
        return (int)goal;
    }
    public void resetEncoders(){
        leftFront.setMode(STOP_AND _RESET_ENCODERS);
        leftBack.setMode(STOP_AND _RESET_ENCODERS);
        rightFront.setMode(STOP_AND _RESET_ENCODERS);
        rightBack.setMode(STOP_AND _RESET_ENCODERS);
    }
}
*/