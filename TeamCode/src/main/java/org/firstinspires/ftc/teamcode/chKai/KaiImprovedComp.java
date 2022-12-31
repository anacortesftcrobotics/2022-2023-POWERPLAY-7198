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

public class KaiImprovedComp implements SubsystemManager{

    private DcMotor rightBack, leftBack, rightFront, leftFront, encoderLeft, encoderRight, encoderBack;
    private boolean hasChanged = false;
    private boolean Ying, Xing, Ring;
    private boolean YCing, XCing, RCing;
    private int xCounter, yCounter, rCounter;
    private double goalX, goalY, goalR;
    private int lastGoalX;
    private int lastGoalY;
    BNO055IMU imu;
    FakeOdometry odo = new FakeOdometry();
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void initializeHardware(HardwareMap hardwareMap) {
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
        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderBack = rightFront;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        
        imu.initialize(parameters);
    }
    public void check (Gamepad gamepad1)

                    if (dBounce(gamepad1.dpad_up)){
                        yCounter ++;
                        setGoalY();
                    }else if (dBounce(gamepad1.dpad_down)){
                        yCounter --;
                        setGoalY();
                    }else if (gamepad1.dpad_left){
                        side(1);
                    }else if (gamepad1.dpad_right){
                        side(-1);
                    }else if (gamepad1.right_bumper){
                        turn(1);
                    }else if (gamepad1.left_bumper){
                        turn(-1);
                    }
                    odo.getX();
                    odo.getY();
                    odo.getR();
                    drive();
                    if (odo.getY() = goalY && odo.getX() = goalX && odo.getR() = goalR && !stickActive()){
                        setMotors(1,0);
                    }

    }
    public void drive(){
        double pwr=0;
        if(!Xing && !Ring && !stickActive()){
            Ying = true;
            pwr = (goalY - odo.getY())/12
            if (pwr > 0.5){
                pwr = 0.5;
            }else if(pwr < -0.5){
                pwr = -0.5
            }
            setHeadless(1,pwr);
        }else{
            Ying = false;
        }
    }
    Math.abs(goalY - odo.getY())<1 && 

    public void centerX(){
        double near = (Math.round(odo.getX()/24))*24;
        double pwr = 0;
        if (Math.abs(odo.getX - near)>1){
            XCing = true;
            pwr = (near - odo.getX())/12
            if (pwr > 0.5){
                pwr = 0.5;
            }else if(pwr < -0.5){
                pwr = -0.5
            }
            setHeadless(1,pwr);
        }else{
            XCing = false;
        }
    }*/
    /**
    *Checks if either stick is being moved
    *if true, sets all goals to current position and all counters to 0, thereby disabling semi-autonomous movement
    *@param gamepad1 is the Gamepad being tested, presumably the chassis controller
    *//*
    public boolean stickActive(Gamepad gamepad1){
        boolean stickActive;
        if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.01){
            stickActive = true;
        }else{
            stickActive = false;
        }
        if (stickActive){
            goalX=odo.getX();
            goalY=odo.getY();
            goalR=odo.getR();
            yCounter = 0;
            xCounter = 0;
            rCounter = 0;
        }
        return stickActive;
    }*/
    /**Takes a boolean key from the gamepad and only lets it "count" once.
    *@param input is the boolean key being used
    *can only be used for one key at a time; if multiple keys are dBounced at the same time, only the first one will be counted
    *//*
    public boolean dBounce(boolean input){

        boolean x = false;
        if (input && !hasChanged){
            x = true;
        }else {
            x = false;
        }
        if(input){
            hasChanged = true;
        }else{
            hasChanged = false;
        }
        return x;
    }
    
    public void setGoalY(){
        double near = (Math.round(odo.getY()/24))*24;
        goalY = near + yCounter;
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

    }*/
    /**
    *movements in relation to field, not robot heading. 
    *@param type is the type of movement, 1 for y-axis and 2 for x-axis
    *type 1: +forward/-back. type2: +right/-left
    *only works if the opMode is initialized with the robot facing forwards
    *for turning, use setMotors
    *//*
    public void setHeadless(int type, double power){
        double a=0; //for FL and BR
        double b=0; //for FR and BL
        if (type == 1){
            a = (power*Math.cos(getAngle()-45));
            b = -(power*Math.sin(getAngle()-45));
        }else if (type == 2){
            a = (power*Math.sin(getAngle()-45));
            b = (power*Math.cos(getAngle()-45));
        }

        double FL =(a);
        double FR =(b);
        double BL =(b);
        double BR =(a);

        double maxPower=Math.max(Math.max(FL,FR),Math.max(BL,BR));
        if (maxPower > 1){
            FL=FL/maxPower;
            FR=FR/maxPower;
            BL=BL/maxPower;
            BR=BR/maxPower;
        }

        leftFront.setPower(FL);

        rightFront.setPower(FR);

        leftBack.setPower(BL);

        rightBack.setPower(BR);
    }
}
*/