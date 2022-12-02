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

    public Robot ()
    {

    }

    //Subassembly Objects
    Chassis chassis = new Chassis();
    Lift lift = new Lift();
    Grabber grabber = new Grabber();
    MPCR mpcr = new MPCR();

    CDS cds = new CDS();
    Pablo pablo = new Pablo();

    Gyro gyro = new Gyro();
    Odometry odometry = new Odometry();

    public void initializeHardware (HardwareMap hardwareMap)
    {
        chassis.initializeHardware(hardwareMap);
        lift.initializeHardware(hardwareMap);
        grabber.initializeHardware(hardwareMap);
        mpcr.initializeHardware(hardwareMap);

        cds.initializeHardware(hardwareMap);
        pablo.initializeHardware(hardwareMap);

        gyro.initializeHardware(hardwareMap);
        odometry.initializeHardware(hardwareMap);
    }

    public void robotDefaultState()
    {
        chassis.brake();
        grabber.grab(false);
        mpcr.preSetMPCR(1);
        lift.liftZero();
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
