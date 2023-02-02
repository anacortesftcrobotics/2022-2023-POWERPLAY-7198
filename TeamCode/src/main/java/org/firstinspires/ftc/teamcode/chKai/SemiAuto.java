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
import org.firstinspires.ftc.teamcode.powerplay.*;

public class SemiAuto implements SubsystemManager{
    //hardware stuff
    //imu
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    //motors/encoders
    private DcMotor rightBack, leftBack, rightFront, leftFront, encoderLeft, encoderRight, encoderBack;//encoderLeft is leftBack, encoderRight is rightBack, and encoderBack is rightFront. see initializeHardware.

    //instance variables
    private boolean hasChanged = false; //see dBounce
    private boolean ok = false; //ok to move semi-autonomously. see check, Y, X, R, and stickActive.
    private boolean Ying, Xing, Ring; //Ying = moving on the Y-axis, etc. see Y, X, and R.
    private boolean YCing, XCing, RCing; //YCing = centering on a tile in the y-axis, etc. see Y, X, centerX, and centerY. not using RCing yet.
    private int xCounter, yCounter, rCounter; //these count how many tiles (or eighth-turns) you want to move in each axis. see check, setGoalY, setGoalX, and setGoalR.
    private double goalX, goalY, goalR; //these hold the current goal position (where the robot is trying to go) for each axis. see X, Y, R, setGoalY, setGoalX, and setGoalR.
    private boolean x = false;
    Gyro gyro = new Gyro();
    Controller controller = new Controller();
    //odometry
    FakeOdometry odo = new FakeOdometry();
    /**
    *initializes all the hardware needed
    *@param hardwareMap is the hardwareMap object you need to pass to it
    */
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
        odo.initializeHardware(hardwareMap);
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
    public double getR(){
        return gyro.getHeading();
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
        odo.getOdoHeading();
        odo.updatePosition();
        Y();
        X();
        R();
        if (odo.getY() == goalY && odo.getX() == goalX && getR() == goalR && !stickActive(gamepad1)){
            setMotors(1,0);
        }

    }*/
    public void test(Gamepad gamepad1){
        if (controller.button(7, gamepad1.dpad_up)){
            yCounter ++;
            setGoalY();
            ok = true;
        }else if (controller.button(4, gamepad1.dpad_down)){
            yCounter --;
            setGoalY();
            ok = true;
        }
        if (controller.button(5, gamepad1.dpad_right)){
            xCounter ++;
            setGoalX();
            ok = true;
        }else if (controller.button(6, gamepad1.dpad_left)){
            xCounter --;
            setGoalX();
            ok = true;
        } else if (controller.button(8, gamepad1.left_stick_button)) {
            centerX();
            centerY();
            ok = true;
        }
        odo.getX();
        odo.getY();
        Y();
        X();
        R();
        if (odo.getY() == goalY && odo.getX() == goalX && getR() == goalR && !stickActive(gamepad1)){
            setMotors(1,0);
        }


    }
    public String telem(Gamepad gamepad1){
        double near = (Math.round(odo.getX()/24))*24;
        return "counter- "+yCounter+" goal- "+goalY+
                "\nstick active- "+stickActive(gamepad1)+"" +
                "\nok- "+ok+"" +
                "\nXing- "+Xing+"" +
                "\nRing- "+Ring+"" +
                "\nx near"+near+
                "\nx pos"+odo.getX()+
                "\nXcing- "+XCing;
    }

    /**
    *executes centerX, then moves the robot along the y-axis to within one inch of goalY
    *only moves if the robot is not in the process of x movement, r movement or non-auto movement
     */
    public void Y(){
        double pwr=0;
        if(Math.abs(goalY - odo.getY()) > 1 && !Xing && !Ring && ok){
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
            yCounter = 0;
        }
    }
    /**
    *executes centerY, then moves the robot along the x-axis to within one inch of goalX
    *only moves if the robot is not in the process of y movement, r movement or non-auto movement
     */
    public void X(){
        double pwr=0;
        if(Math.abs(goalX - odo.getX()) > 1 && !Ying && !Ring && ok){
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
    *only moves if the robot is not in the process of y movement, x movement or non-auto movement
     */
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
    /**
    *centers the robot on the nearest path in the x-axis, preparing it to move along the y-axis
    *path = an imaginary line going down the center of a row of tiles. we want the robot to move along the path.
     */
    public void centerX(){
        double near = (Math.round(odo.getX()/24))*24;
        double pwr = 0;
        if (Math.abs(odo.getX() - near)>1){
            XCing = true;
            /*pwr = (near - odo.getX())/12;
            if (pwr > 0.5){
                pwr = 0.5;
            }else if(pwr < -0.5){
                pwr = -0.5;
            }*/
            if(odo.getX()>near) {
                pwr = 0.5;
            }else{
                pwr = -0.5;
            }
            setHeadless(2,pwr);
        }else{
            XCing = false;
        }
    }
    /**
    *centers the robot on the nearest path in the y-axis, preparing it to move along the x-axis
    *path = an imaginary line going down the center of a row of tiles. we want the robot to move along the path.
     */
    public void centerY(){
        double near = (Math.round(odo.getY()/24))*24;
        double pwr = 0;
        if (Math.abs(odo.getY() - near)>1){
            YCing = true;
            /*pwr = (near - odo.getY())/12;
            if (pwr > 0.5){
                pwr = 0.5;
            }else if(pwr < -0.5){
                pwr = -0.5;
            }*/
            if(odo.getY()>near) {
                pwr = 0.5;
            }else{
                pwr = -0.5;
            }
            pwr = 5.0;
            setHeadless(1,pwr);
        }else{
            YCing = false;
        }
    }
    /**
    *determines the y-axis goal and sets goalY based on the nearest path and yCounter
     */
    public void setGoalY(){
        double near = (Math.round(odo.getY()/24))*24;
        goalY = near + yCounter*24;
    }
    /**
    *determines the x-axis goal and sets goalX based on the nearest path and xCounter
     */
    public void setGoalX(){
        double near = (Math.round(odo.getX()/24))*24;
        goalY = near + xCounter;
    }
    /**
    *determines the rotational axis goal and sets goalR based on the nearest 45* angle and rCounter
     */
    public void setGoalR(){
        double near = (Math.round(getR()/45))*45;
        goalY = near + rCounter;
    }

    /**
    *Checks if either stick is being moved
    *if true, sets all goals to current position, all counters to 0, and ok to false, thereby disabling semi-autonomous movement
    *@param gamepad1 is the Gamepad being checked, presumably the chassis controller
    */
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
            goalR=getR();
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
     *did not work, used controller.button instead.
    */
    public boolean dBounce(boolean input){
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
    */
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
    *@param Power is the power you want it set at, from -1 to 1.
    *only works if the opMode is initialized with the robot facing forwards
    *for turning, use setMotors
    */
    public void setHeadless(int type, double Power){
        double power = -Power; //because it was going backwards
        double a=0; //for FL and BR
        double b=0; //for FR and BL
        if (type == 1){
            a = (power*Math.cos(getR()-45));
            b = -(power*Math.sin(getR()-45));
        }else if (type == 2){
            a = (power*Math.sin(getR()-45));
            b = (power*Math.cos(getR()-45));
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
