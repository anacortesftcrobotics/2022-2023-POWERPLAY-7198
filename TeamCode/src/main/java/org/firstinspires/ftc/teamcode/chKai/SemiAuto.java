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

public class SemiAuto implements SubsystemManager{

    private DcMotor rightBack, leftBack, rightFront, leftFront, encoderLeft, encoderRight, encoderBack;
    private boolean hasChanged = false;
    private boolean ok = false;
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

    /**
    *initializes all the hardware needed
    *@param hardwareMap is the hardwareMap object you need to pass to it
    *//*
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
    /**
    *call this method in your opMode loop to use this class
    *@param gamepad1 is the gamepad you want to use for chassis control
     *//*
    public void check (Gamepad gamepad1){
        if (dBounce(gamepad1.dpad_up)){
            yCounter ++;
            setGoalY();
            ok = true;
        }else if (dBounce(gamepad1.dpad_down)){
            yCounter --;
            setGoalY();
            ok = true;
        }else if (dBounce(gamepad1.dpad_left)){
            xCounter --;
            setGoalX();
            ok = true;
        }else if (dBounce(gamepad1.dpad_right)){
            xCounter++;
            setGoalX();
            ok = true;
        }else if (gamepad1.right_bumper){
            rCounter ++;
            setGoalR();
            ok = true;
        }else if (gamepad1.left_bumper){
            rCounter --;
            setGoalR();
            ok = true;
        }
        odo.getX();
        odo.getY();
        odo.getR();
        Y();
        X();
        R();
        if (odo.getY() = goalY && odo.getX() = goalX && odo.getR() = goalR && !stickActive(gamepad1)){
            setMotors(1,0);
        }

    }
    /**
    *executes center X, then moves the robot along the y-axis to within one inch of goalY
    *only moves if the robot is in the process of strafing or turning or non-auto movement
     *//*
    public void Y(){
        double pwr=0;
        if(Math.abs(goalY - odo.getY())<1 && !Xing && !Ring && ok){
            Ying = true;
            centerX();
            if (!XCing){
                pwr = (goalY - odo.getY())/12;
                if (pwr > 0.5){
                    pwr = 0.5;
                }else if(pwr < -0.5){
                    pwr = -0.5;
                }
            }
            setHeadless(1,pwr);
        }else{
            Ying = false;
        }
    }
    /**
    *executes centerY, then moves the robot along the x-axis to within one inch of goalX
    *only moves if the robot is in the process of driving or turning or non-auto movement
     *//*
    public void X(){
        double pwr=0;
        if(Math.abs(goalX - odo.getX())<1 && !Ying && !Ring && ok){
            Xing = true;
            centerY();
            if (!YCing){
                pwr = (goalX - odo.getX())/12;
                if (pwr > 0.5){
                    pwr = 0.5;
                }else if(pwr < -0.5){
                    pwr = -0.5;
                }
            }
            setHeadless(2,pwr);
        }else{
            Xing = false;
        }
    }
    /**
    *turns the robot to within three degrees of goalR
    *only moves if the robot is in the process of strafing or driving or non-auto movement
     *//*
    public void R(){
        double pwr=0;
        if(Math.abs(goalY - odo.getY())<3 && !Xing && !Ying && ok){
            Ring = true;
            pwr = (goalY - odo.getY())/20;
            if (pwr > 0.5){
                pwr = 0.5;
            }else if(pwr < -0.5){
                pwr = -0.5;
            }
            
            setMotors(3,pwr);
        }else{
            Ring = false;
        }
    }

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
            setHeadless(2,pwr);
        }else{
            XCing = false;
        }
    }

    public void centerY(){
        double near = (Math.round(odo.getY()/24))*24;
        double pwr = 0;
        if (Math.abs(odo.getY() - near)>1){
            YCing = true;
            pwr = (near - odo.getY())/12;
            if (pwr > 0.5){
                pwr = 0.5;
            }else if(pwr < -0.5){
                pwr = -0.5;
            }
            setHeadless(1,pwr);
        }else{
            YCing = false;
        }
    }

    public void setGoalY(){
        double near = (Math.round(odo.getY()/24))*24;
        goalY = near + yCounter;
    }

    public void setGoalX(){
        double near = (Math.round(odo.getX()/24))*24;
        goalY = near + xCounter;
    }

    public void setGoalR(){
        double near = (Math.round(odo.getR()/45))*45;
        goalY = near + rCounter;
    }

    /**
    *Checks if either stick is being moved
    *if true, sets all goals to current position, all counters to 0, and ok to false, thereby disabling semi-autonomous movement
    *@param gamepad1 is the Gamepad being checked, presumably the chassis controller
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
            ok = false;
        }
        return stickActive;
    }
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
    /**
    *sets motors to drive, strafe, or turn. 
    *@param type is the type of movement, 1 for driving, 2 for strafing, and 3 for turning.
    *type 1: +forward/-back. types 2 and 3: +right/-left
    *@param power is the power you want it set at, from -1 to 1.
    *//*
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
    /**
    *movements in relation to field, not robot heading. 
    *@param type is the type of movement, 1 for y-axis and 2 for x-axis
    *type 1: +forward/-back. type2: +right/-left
    *@param power is the power you want it set at, from -1 to 1.
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
