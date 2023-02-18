package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode{
    DcMotor turret1;
    DcMotor turret2;

    Servo turnTable;
    Basic basic = new Basic();
    public void runOpMode() {
        turret1 = hardwareMap.get(DcMotor.class, "turret1");
        turret2 = hardwareMap.get(DcMotor.class, "turret2");
        turnTable = hardwareMap.get(Servo.class, "turnTable");
        basic.initializeHardware(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                basic.go(gamepad1);
                if (gamepad1.dpad_up){turret1.setPower(0.25);}
                else if (gamepad1.dpad_down){turret1.setPower(-0.25);}
                else if (gamepad1.dpad_right){turret2.setPower(0.25);}
                else if (gamepad1.dpad_left){turret2.setPower(-0.25);}
                else if (gamepad1.right_bumper){turnTable.setPosition(0.25);}
                else if (gamepad1.left_bumper){turnTable.setPosition(-0.25);}
                }
            }
        }
    }




