package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")

public class TeleOp extends OpMode
{
    //Hardware stuff
    private DcMotor leftBack, leftFront, rightBack, rightFront, leftLift, rightLift;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;

    //Servos
    private Servo leftGrab, rightGrab, leftMPCR, rightMPCR;

    //Distance Sensors
    private DistanceSensor leftDistance, rightDistance;

    //Color Sensors
    private ColorRangeSensor color;

    //Touch Sensors
    private TouchSensor zero;

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    Robot powerplay = new Robot();

    public void init() {
        powerplay.initializeHardware(hardwareMap);
        powerplay.robotDefaultState();
    }

    public void loop() {
        powerplay.chassis.xyrMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
