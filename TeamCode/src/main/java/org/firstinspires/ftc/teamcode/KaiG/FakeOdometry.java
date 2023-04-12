package org.firstinspires.ftc.teamcode.KaiG;
import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class  FakeOdometry {
    DcMotor leftFront, rightFront, leftBack, rightBack, encoderLeft, encoderRight, encoderBack;
    Gyro gyro = new Gyro();
    public void initializeHardware(HardwareMap hardwareMap){
        //leftBack
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftFront
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightBack
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightFront
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderBack = rightFront;

        //reset, etc.

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.initializeHardware(hardwareMap);

    }

    public double getY(){
        return -(encoderLeft.getCurrentPosition() - encoderRight.getCurrentPosition())/3750;
    }
    public double getX(){ return -(encoderBack.getCurrentPosition())/1875; }
    public double getR(){
        gyro.updateHeading();
        return gyro.getHeading();
    }
}