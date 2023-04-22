package org.firstinspires.ftc.teamcode.KaiG;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.pidclasses.PIDFArmController;


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
    PIDFArmController control = new PIDFArmController(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0);
    ElapsedTime systemTime = new ElapsedTime();
    double targetPos = 0;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        control.launch(turret1.getCurrentPosition(), systemTime.milliseconds());
        control.setOutputClamping(-1, 1);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(gamepad1.dpad_up){targetPos = 15;}
                if(gamepad1.dpad_down){targetPos =0;}

                control.updateArmClamped(Math.toRadians(targetPos), getRads(), 0, systemTime.milliseconds());
                basic.go(gamepad1);
                telemetry.addData("turret2 position", turret2.getCurrentPosition());
                telemetry.addData("worm position", turret1.getCurrentPosition());
                telemetry.addData("imu", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("imu2", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("rad", getRads());
                telemetry.update();
            }
        }
    }
    public double getRads(){
        return Math.toRadians((turret1.getCurrentPosition()+20)/2.222222);
    }
}




