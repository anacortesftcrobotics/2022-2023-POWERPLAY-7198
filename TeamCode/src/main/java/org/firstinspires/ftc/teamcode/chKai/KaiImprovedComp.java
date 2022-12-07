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

    private boolean hasChanged = false;
    private double servoSet;
    private int positionx = 0;
    private int positiony = 0;
    private int lastGoalX;
    private int lastGoalY;
    BNO055IMU imu;


    Orientation angles;
    Acceleration gravity;


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
            while (opModeIsActive()) {
                //left encoder is leftBack; right encoder is rightBack; rear encoder is rightFront.



                if (stickActive()){
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
                    getX();
                    getY();

                    telemetry.update();
                }
            }
        }
    }
    public void forward (double input)
    {
        double side =0;
        if (input > 0){

            while (opModeIsActive() && !stickActive() && getY()<goalY(2)){
                setHeadless(1,0.5);
                if (dBounce(gamepad1.dpad_up)){
                    input += 1;
                }
                if (dBounce(gamepad1.dpad_down)){
                    input -= 1;
                }
                if (dBounce(gamepad1.dpad_right)){
                    side += 1;
                }
                if (dBounce(gamepad1.dpad_left)){
                    side -= 1;
                }
            }
            if (input != 0){
                forward(input);
            }
            if (side != 0){
                side(side);
            }
        }else if (input < 0){
            while (opModeIsActive()&&!stickActive()&&getY()>goalY(0)){
                setHeadless(1,0.5);
                if (dBounce(gamepad1.dpad_up)){
                    input += 1;
                }
                if (dBounce(gamepad1.dpad_down)){
                    input -= 1;
                }
                if (dBounce(gamepad1.dpad_right)){
                    side += 1;
                }
                if (dBounce(gamepad1.dpad_left)){
                    side -= 1;
                }
            }
        }
    }
    public void side (double input)
    {

        double lastInput = input;
        boolean hasChanged = false;

        while(opModeIsActive() && rightFront.getCurrentPosition()<(794888*input) && !stickActive()) {
            telemetry.addData("x",input);

            setMotors(2,0.5);



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
    public void turn(double input)
    {

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
    public boolean stickActive(){
        boolean stickActive;
        if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0.1){
            stickActive = true;
        }else{
            stickActive = false;
        }
        return stickActive;
    }
    public boolean dBounce(boolean input){

        boolean x = false;
        if (input && !hasChanged){
            x = true;
            hasChanged = true;
        }else {
            hasChanged = false;
        }
        return x;
    }
    public double getAngle(){ //gets angle in radian form. 0* is 90* to the right of the starting orientation, and value adds when turning counterclockwise.
        //use for trig stuff
        float angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        float angle2 = 0;
        if (angle1<0){
            angle2=angle1+450;
        }else if (angle1>=0){
            angle2=angle1+90;
        }
        double radians=Math.toRadians(angle2);
        return radians;
    }
    public int getX(){ //gets current position on the x-axis. Assumes that starting point is 0,0. Must be continually called.
        int x = positionx;
        x += ((leftBack.getCurrentPosition()-rightBack.getCurrentPosition())/2)*Math.cos(getAngle());
        x += ((rightFront.getCurrentPosition())/2)*Math.sin(getAngle());
        positionx = x;
        return x;
    }
    public int getY(){ //gets current position on the y-axis. Assumes that starting point is 0,0. Must be continually called.
        int y = positiony;
        y += ((leftBack.getCurrentPosition()-rightBack.getCurrentPosition())/2)*Math.sin(getAngle());
        y += ((rightFront.getCurrentPosition())/2)*Math.cos(getAngle());
        positiony = y;
        return y;
    }
    public int getR(){ //gets angle in 360* form. value adds when turning counterclockwise.
        double angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double angle2 = 0;
        if (angle1<0){
            angle2=angle1+360.0;
        }else if (angle1>=0){
            angle2=angle1;
        }
        return  (int) angle2;
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
    public void setHeadless(int type, double power){
        //movements in relation to field, not robot heading.
        //type 1 is +forward/-back, 2 is +right/-left.
        //for turning, use setMotors.
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
