package org.firstinspires.ftc.teamcode.KaiG;

import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.robotcore.hardware.*;

public class Basic implements SubsystemManager{
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    public void initializeHardware(HardwareMap hardwareMap){
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void go(Gamepad gamepad1) {
                leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
    }

    public void goPlus(double FL, double FR, double BL, double BR) {
        leftFront.setPower(FL);
        leftBack.setPower(BL);
        rightFront.setPower(FR);
        rightBack.setPower(BR);
    }
}
