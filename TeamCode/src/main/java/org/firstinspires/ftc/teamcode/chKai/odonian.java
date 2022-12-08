package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class odonian {
    private DcMotor leftBack, leftFront, rightBack, rightFront;


    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private int positionx = 0;
    private int positiony = 0;

    public void init(HardwareMap hardwareMap){
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

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public double getAngle(){
        //use for trig stuff
        //Y +=(cos(f)+sin(s))
        //X +=(-sin(f)+cos(s))

        float angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double angle2 = Math.toRadians(angle1);

        return angle2;
    }
    public int getX(){ //gets current position on the x-axis. Assumes that starting point is 0,0. Must be continually called.
        int x = positionx;
        double f = ((leftBack.getCurrentPosition()-rightBack.getCurrentPosition())/2);
        double s = ((rightFront.getCurrentPosition()));
        double diff = leftBack.getCurrentPosition()+rightBack.getCurrentPosition();
        //X +=(-sin(f)+cos(s))
        x -= f * Math.sin(getAngle());
        x += s * Math.cos(getAngle());
        resetEncoders();
        positionx = x;
        return x;
    }
    public int getY(){ //gets current position on the y-axis. Assumes that starting point is 0,0. Must be continually called.
        int y = positiony;
        //Y +=(cos(f)+sin(s))
        y += ((leftBack.getCurrentPosition()-rightBack.getCurrentPosition())/2)*Math.cos(getAngle());
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
    public String odoTelemetry(){
        return ("x- "+getX()+"\ny- "+getY()+"\nangle- "+getAngle());
    }
    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
