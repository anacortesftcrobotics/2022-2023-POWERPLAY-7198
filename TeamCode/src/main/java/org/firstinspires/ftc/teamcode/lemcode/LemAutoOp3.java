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
import org.firstinspires.ftc.teamcode.kaicode.Odo1Offset;

@Disabled
@Autonomous(name = "LemAutoOpRP1", group = "Red Position 1")

public class LemAutoOp3 extends LinearOpMode
{
    //Variables
    int colorCheck = 1;
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

    //Odometry
    Odo1Offset odometryTracker = new Odo1Offset(-813829,796907,-786263,3.5, 8192);

    //Library
    private LemLibrary robob;
    //Put the function library here
    /*
    init
    move
    strafe
    turn
    MPCR

    liftZero
    lift
    grab
    check
    poleCenter (buggy)

     0 - 1        - retracted
     1 - 0.45     - rest
     2- 0.14      - cone right

     0 - grabbing height
     1 - stack 1
     2 - stack 2
     3 - stack 3
     4 - stack 4
     5 - stack 5
     6 - ground
     7 - low
     8 - medium
     9 - high
     */
    //Put notes here
    /*
    Always remember to move the MPCR after a delay after moving the lift.
    Hopefully this will become part of the deposit function.
    telemetry.addLine("");
    telemetry.update();
     */
    public void runOpMode() throws InterruptedException
    {
        robob = new LemLibrary(hardwareMap, this, telemetry);
        robob.init();
        waitForStart();

        robob.grab();
        robob.grab();
        robob.grab();
        sleep(500);
        robob.lift(6);
        robob.move(2);
        robob.turn(85);
        robob.moveBad(-20);
        robob.grab();
        sleep(500);
        robob.lift(0);
        robob.moveBad(18);
        robob.turn(-90);

        //regular park stuff from starting position
        robob.move(16);
        colorCheck = robob.check();
        sleep(250);
        robob.move(22);
        robob.move(-16);
        //robob.turn(-10);
        if(colorCheck == 1)
            robob.strafe(-22);
        if(colorCheck == 3)
            robob.strafe(22);
        robob.move(2);

        //allStop here
    }
}
