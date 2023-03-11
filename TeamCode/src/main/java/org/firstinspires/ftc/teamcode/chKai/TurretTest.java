package org.firstinspires.ftc.teamcode.chKai;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode{
    DcMotor turret1;
    DcMotor turret2;

    DcMotorSimple turnTable;
    Basic basic = new Basic();

    BNO055IMU imu, imu2;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        turret1 = hardwareMap.get(DcMotor.class, "turret1");
        turret2 = hardwareMap.get(DcMotor.class, "turret2");
        turnTable = hardwareMap.get(DcMotorSimple.class, "turnTable");
        turret1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        basic.initializeHardware(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
        imu2.initialize(parameters);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                basic.go(gamepad1);
                if (gamepad1.dpad_up){turret1.setPower(1.0);}
                else if (gamepad1.dpad_down){turret1.setPower(-0.3);}
                else if (gamepad1.dpad_right) {
                    turret2.setPower(1);
                    /*turret2.setTargetPosition(10);
                    turret2.setPower(0.5);
                    turret2.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                }
                else if (gamepad1.dpad_left){
                    turret2.setPower(-0.3);
                    /*turret2.setTargetPosition(2);
                    turret2.setPower(0.3);
                    turret2.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                }
                else if (gamepad1.right_bumper){turnTable.setPower(0.25);}
                else if (gamepad1.left_bumper){turnTable.setPower(-0.25);}
                else {
                    turret1.setPower(0);
                    turret2.setPower(0);
                    turnTable.setPower(0);
                }
                telemetry.addData("turret2 position", turret2.getCurrentPosition());
                telemetry.addData("worm position", turret1.getCurrentPosition());
                telemetry.addData("imu", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("imu2", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("angle", angle1());
                telemetry.update();
            }
        }
    }
    //turret2 is 70 ticks for 90 degrees.
    public double angle1(){
        return Math.toRadians((turret1.getCurrentPosition()/2.222222222222)+5);
    }
}




