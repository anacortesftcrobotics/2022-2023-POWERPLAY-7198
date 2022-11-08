package org.firstinspires.ftc.teamcode.lemcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "LemAutoOp", group = "Autos")

public class LemAutoOp extends LinearOpMode
{
    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront, leftLift, rightLift;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;

    //Servos
    private Servo leftGrab, rightGrab, leftMPCR, rightMPCR;

    //Distance Sensors
    private DistanceSensor leftDistance, rightDistance;

    //Color Sensors
    private DistanceSensor color;

    //Touch Sensors
    private TouchSensor zero;

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Library
    private LemLibrary robob;
    //Put the function library here
    /*

     */

    public void runOpMode() throws InterruptedException
    {
        robob = new LemLibrary(hardwareMap, this, telemetry);
        robob.init();
        waitForStart();
        waitForStart();
        robob.move(26);
        robob.move(-26);
        //robob.strafe(10);
        //robob.turn(45);
        robob.grab();
        robob.grab();

        //allStop here
    }
}
