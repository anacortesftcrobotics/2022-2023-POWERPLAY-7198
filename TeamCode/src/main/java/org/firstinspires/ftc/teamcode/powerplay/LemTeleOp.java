package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "TeleOp", group = "TeleOp")

public class LemTeleOp extends OpMode
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

    //Robot
    Robot powerplay = new Robot();

    //Controllers
    Controller controller1 = new Controller(gamepad1);
    Controller controller2 = new Controller(gamepad2);

    public void init()
    {
        powerplay.initializeHardware(hardwareMap);
        powerplay.robotDefaultState();
    }
    public void loop()
    {
        powerplay.driveLoop(controller1, controller2);
        powerplay.dataLoop();
        powerplay.robotTelemetry(telemetry);
    }
}