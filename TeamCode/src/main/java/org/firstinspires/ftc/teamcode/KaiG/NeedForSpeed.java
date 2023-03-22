package org.firstinspires.ftc.teamcode.KaiG;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor;

/** */
public class NeedForSpeed implements SubsystemManager{
    ElapsedTime tx = new ElapsedTime();
    ElapsedTime ty = new ElapsedTime();
    ElapsedTime tr = new ElapsedTime();
    ElapsedTime px = new ElapsedTime();
    ElapsedTime py = new ElapsedTime();
    ElapsedTime pr = new ElapsedTime();
    private DcMotor rightFront, leftFront, leftBack, rightBack;
    private DcMotor encoderLeft, encoderRight, encoderBack;
    private int lastX;
    private int lastY;
    private int lastR;
    private int lastMoveX;
    private int lastMoveY;
    private int lastMoveR;
    private boolean hasChanged;
    private double speedCoefficient = 1;

    public void initializeHardware(HardwareMap hardwareMap){
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderBack = rightFront;
        zero();
    }
    /**
    *sets all encoders to zero and motors to run without encoder
    */
    public void zero(){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
    *gets the speed forward/back in encoder ticks per millisecond
    *once I test it I can set max to the right value and it will give values from -1 to 1 */
    public double getSpeedY(double x){//x would be the speed coeficcient
        double max = 175.0;//would be the max speed we wanted, so the method would return values from 0 to 1
        double moved = (encoderLeft.getCurrentPosition() - encoderRight.getCurrentPosition()) - lastY;//amount moved since method last called
        double y = (moved/ty.milliseconds())/(max*x);
        ty.reset();
        lastY = encoderLeft.getCurrentPosition() - encoderRight.getCurrentPosition();
        return -y;
    }
    public double getSpeedX(double x){//x would be the speed coeficcient
        double max = 80.0;//would be the max speed we wanted, so the method would return values from 0 to 1
        double moved = (encoderBack.getCurrentPosition()) - lastX;//amount moved since method last called
        double y = (moved/tx.milliseconds())/(max*x);//higher x means smaller return value which would make the robot go faster
        tx.reset();
        lastX = encoderBack.getCurrentPosition();
        return y;
    }
    public double getSpeedR(double x){//x would be the speed coeficcient
        double max = 100.0;//would be the max speed we wanted, so the method would return values from 0 to 1
        double moved = (encoderLeft.getCurrentPosition()+encoderRight.getCurrentPosition()) - lastX;//amount moved since method last called
        double y = (moved/tr.milliseconds())/(max*x);//higher x means smaller return value which would make the robot go faster
        tr.reset();
        lastX = encoderLeft.getCurrentPosition()+encoderRight.getCurrentPosition();
        return y;
    }
    public double moveY (double input, double coefficient){ //input would be the speed to set it at. coefficient would be the speed coefficient. both should be between 1 and 0.
        if (py.milliseconds()>20){
            lastMoveY += (input - getSpeedY(coefficient))/2;
            py.reset();
        }
        return lastMoveY;
    }
    public double moveX (double input, double coefficient){ //input would be the speed to set it at. coefficient would be the speed coefficient. both should be between 1 and 0.
        if (px.milliseconds()>20){
            lastMoveX += (input - getSpeedX(coefficient))/2;
            px.reset();
        }
        return lastMoveX;
    }
    public double moveR (double input, double coefficient){ //input would be the speed to set it at. coefficient would be the speed coefficient. both should be between 1 and 0.
        if (pr.milliseconds()>20){
            lastMoveR += (input - getSpeedR(coefficient))/2;
            pr.reset();
        }
        return lastMoveR;
    }
    public String check (Gamepad gamepad1){

        if (gamepad1.left_trigger > 0.5 && !hasChanged){
            speedCoefficient -= 0.25;
        }else if (gamepad1.right_trigger > 0.5 && !hasChanged){
            speedCoefficient += 0.25;
        }
        if (gamepad1.left_trigger>0.5 || gamepad1.right_trigger>0.5){
            hasChanged = true;
        }else{
            hasChanged = false;
        }
        if (speedCoefficient < 0.25){
            speedCoefficient = 0.25;
        }
        if (speedCoefficient > 1.0){
            speedCoefficient = 1.0;
        }
        double y = moveY(gamepad1.left_stick_y, speedCoefficient);
        double x = moveX(gamepad1.left_stick_x, speedCoefficient);
        double r = moveR(gamepad1.right_stick_x, speedCoefficient);
        
        double FL = (y - x - r);

        double FR = (y + x + r);

        double BL = (y + x - r);

        double BR = (y - x + r);

        double maxPower = Math.max(Math.max(FL,FR),Math.max(BL,BR));
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
        return  "y= "+y+"x= "+x+"r= "+r;
    }
    public double getCoefficient(){
        return speedCoefficient;
    }
    
    
}
