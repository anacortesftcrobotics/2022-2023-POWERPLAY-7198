/*package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class NeedForSpeed implements SubsystemManager{
    ElapsedTime tx = new ElapsedTime();
    ElapsedTime ty = new ElapsedTime();
    ElapsedTime tr = new ElapsedTime();
    ElapsedTime px = new ElapsedTime();
    ElapsedTime py = new ElapsedTime();
    ElapsedTime pr = new ElapsedTime();
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor encoderLeft, encoderRight, encoderBack;
    private int lastX;
    private int lastY;
    private int lastR;
    private int lastMoveX;
    private int lastMoveY;
    private int lastMoveR;
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
    public double getSpeedY(double x){//x would be the speed coeficcient
        double max = 1.0;//would be the max speed we wanted, so the method would return values from 0 to 1
        double moved = (encoderLeft.getCurrentPosition() - encoderRight.getCurrentPosition()) - lastY;//amount moved since method last called
        double y = (moved/ty.milliseconds())/(max*x);
        ty.reset();
        lastY = encoderLeft.getCurrentPosition() - encoderRight.getCurrentPosition();
        return y;
    }
    public double getSpeedX(double x){//x would be the speed coeficcient
        double max = 1.0;//would be the max speed we wanted, so the method would return values from 0 to 1
        double moved = (encoderBack.getCurrentPosition()) - lastX;//amount moved since method last called
        double y = (moved/tx.milliseconds())/(max*x);//higher x means smaller return value which would make the robot go faster
        tx.reset();
        lastX = encoderBack.getCurrentPosition();
        return y;
    }
    public double getSpeedR(double x){//x would be the speed coeficcient
        double max = 1.0;//would be the max speed we wanted, so the method would return values from 0 to 1
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
    public void check (Gamepad gamepad1, double speedCoefficient){
        double y = s.moveY(gamepad1.left_stick_y, speedCoefficient);
        double x = s.moveX(gamePad1.left_stick_x, speedCoefficient);
        double r = s.moveR(gamePad1.right_stick_x, speedCoefficient);
        
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
    }
    
    
}
*/