package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.powerplay.Gyro;

@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode{
    DcMotor turret1;
    DcMotor turret2;

    DcMotorSimple turnTable;
    Basic basic = new Basic();
    Gyro gyro = new Gyro();
    BNO055IMU imu, imu2;
    Orientation angles;
    Acceleration gravity;
    int lastT1, lastT2;
    double  lastTable;
    ElapsedTime tTable = new ElapsedTime();
    ElapsedTime tT1 = new ElapsedTime();
    ElapsedTime tT2 = new ElapsedTime();
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public void runOpMode() {
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
        gyro.initializeHardware(hardwareMap);

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
                getTableVelo();
                getT1Velo();
                getT2Velo();
                telemetry.addData("turret2 position", turret2.getCurrentPosition());
                telemetry.addData("worm position", turret1.getCurrentPosition());
                telemetry.addData("imu", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("imu2", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
        }
    }
    public double getTableVelo(){
        double x = (getTablePos() - lastTable)/tTable.milliseconds();
        lastTable = getTablePos();
        tTable.reset();
        return x;
    }
    public double getT1Velo(){
        double x = (turret1.getCurrentPosition()-lastT1)/tT1.milliseconds();
        lastT1 = turret1.getCurrentPosition();
        tT1.reset();
        return x;
    }
    public double getT2Velo(){
        double x = (turret2.getCurrentPosition()-lastT2)/tT2.milliseconds();
        lastT2 = turret2.getCurrentPosition();
        tT2.reset();
        return x;
    }
    public double getTablePos(){
        return gyro.getHeading() -gyro.getHeading2();
    }
}




