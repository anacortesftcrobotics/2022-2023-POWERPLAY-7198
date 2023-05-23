package org.firstinspires.ftc.teamcode.KaiG;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.pidclasses.PIDFArmController;
import org.firstinspires.ftc.teamcode.powerplay.Chassis;
import org.firstinspires.ftc.teamcode.powerplay.Controller;


@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode{
    DcMotor turret1;
    DcMotor turret2;
    DcMotor wrist;
    Servo hand;
    TouchSensor zero1, zero2;
    boolean lim1, lim2;
    double htarg;
    double j1Coefficient = 0.4;
    double j2Coefficient = 0.4;
    double wristCoefficient = 0.6;


    DcMotorSimple turnTable;

    Chassis chassis = new Chassis();

    BNO055IMU imu, imu2;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    PIDFArmController control = new PIDFArmController(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0);
    ElapsedTime systemTime = new ElapsedTime();
    Controller controller1 = new Controller();
    Controller controller2 = new Controller();
    double targetPos = 0;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist = hardwareMap.get(DcMotor.class, "wrist");
        turret1 = hardwareMap.get(DcMotor.class, "turret1");
        turret2 = hardwareMap.get(DcMotor.class, "turret2");
        hand = hardwareMap.get(Servo.class, "grabber");
        chassis.initializeHardware(hardwareMap);

        zero1 = hardwareMap.get(TouchSensor.class, "zero1");
        zero2 = hardwareMap.get(TouchSensor.class, "zero2");

        turnTable = hardwareMap.get(DcMotorSimple.class, "turnTable");
        turret1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller1.setGamepad(gamepad1);
        controller2.setGamepad(gamepad2);

        turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //basic.initializeHardware(hardwareMap);

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        imu.initialize(parameters);
//        imu2.initialize(parameters);
//        control.launch(turret1.getCurrentPosition(), systemTime.milliseconds());
//        control.setOutputClamping(-1, 1);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                chassis.xyrMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

                if (gamepad2.right_bumper){turnTable.setPower(-0.5);}
                else if (gamepad2.left_bumper){turnTable.setPower(0.5);}
                else turnTable.setPower(0);

                if (gamepad2.a){
                    hand.setPosition(0);
                }
                if (gamepad2.b){
                    hand.setPosition(0.5);
                }

//                if(!zero1.isPressed() || gamepad2.right_stick_y > 0)                }
                turret1.setPower(-j1Coefficient * gamepad2.right_stick_y);

//                turret1.setPower(-j1Coefficient * gamepad2.right_stick_y);
                turret2.setPower(-j2Coefficient * gamepad2.left_stick_y);
//                wrist.setPower(-wristCoefficient * gamepad2.left_stick_x);

                if (gamepad2.dpad_up){wrist.setPower(0.5);}
                else if (gamepad2.dpad_down){wrist.setPower(-0.5);}
                else wrist.setPower(0);

                //zero1.isPressed();

//                telemetry.addData("zero1", zero1.isPressed());
//                telemetry.addData("zero2", zero2.isPressed());
                telemetry.update();

            }
        }
    }
    public double getRads(){
        return Math.toRadians((turret1.getCurrentPosition()+20)/2.222222);
    }
}




