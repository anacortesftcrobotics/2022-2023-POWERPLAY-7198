package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This class is an intermediary between the teleOp and autoOp classes and all the sub-assembly classes on the 2022-2023 powerplay robot.
 * @author      Lemon
 */

public class Robot {
    /*
    acts as intermediary between all subsystem class and opmodes via the interface
    put them all here, then call robob elsewhere
    should contain all possible functions (i.e move, MPCRToPosition), and some parameters for variability (i.e drive mode, controller mode, )
     */
    public static HardwareMap hardwareMap;

    public Robot (HardwareMap hwMap)
    {
        hardwareMap = hwMap;
    }

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
    //private ColorSensor color;

    //Touch Sensors
    private TouchSensor zero;

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Subassembly Objects
    Chassis chassis = new Chassis(hardwareMap);
    Lift lift = new Lift(hardwareMap);
    Grabber grabber = new Grabber(hardwareMap);
    MPCR mpcr = new MPCR(hardwareMap);

    CDS cds = new CDS(hardwareMap);
    Pablo pablo = new Pablo(hardwareMap);

    Gyro gyro = new Gyro(hardwareMap);
    Odometry odometry = new Odometry(hardwareMap);

    public void initializeHardware ()
    {
        chassis.initializeHardware();
        lift.initializeHardware();
        grabber.initializeHardware();
        mpcr.initializeHardware();

        cds.initializeHardware();
        pablo.initializeHardware();

        gyro.initializeHardware();
        odometry.initializeHardware();
    }

    public void robotDefaultState()
    {
        /*
        zero out lift
        put mpcr into place
        open grabber
        tell motors to stop
         */
    }

    public void controlLoop()
    {

    }

    public void RobotTelemetry()
    {

    }

    //Multi Assembly functions
    public void deposit()
    {

    }
}
