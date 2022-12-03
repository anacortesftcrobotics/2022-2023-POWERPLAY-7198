package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is an intermediary between the teleOp and autoOp classes and all the sub-assembly classes on the 2022-2023 powerplay robot.
 * @author      Lemon
 */

public class Robot {
    public Robot () {}
    //Variables
    boolean coneRight = false;
    boolean grabbed = false;
    boolean singlePlayer = false;
    int stackIterator = 5;

    //Subassembly Objects
    Chassis chassis = new Chassis();
    Lift lift = new Lift();
    Grabber grabber = new Grabber();
    MPCR mpcr = new MPCR();

    CDS cds = new CDS();
    Pablo pablo = new Pablo();

    Gyro gyro = new Gyro();
    Odometry odometry = new Odometry();

    ElapsedTime time = new ElapsedTime();

    Controller controller1 = new Controller();
    Controller controller2 = new Controller();

    /**
     * Initializes all hardware in all subsystem classes.
     */
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

    /**
     * Sets the robot to the default, starting position.
     */
    public void robotDefaultState()
    {
        chassis.brake();
        lift.liftZero();
        grabber.grab(false);
        mpcr.preSetMPCR(0);
    }

    /**
     * The whole teleOp control scheme. Does driving, mpcr, lift, grabbing, etc.
     * @param gamepad1 is the first controller used.
     * @param gamepad2 is the second controller used.
     */
    public void driveLoop(Gamepad gamepad1, Gamepad gamepad2)
    {
        playerCheck(gamepad1);
        if(singlePlayer)
            gamepad2 = gamepad1;

        driveControl(gamepad1);
        MPCRControl(gamepad1);
        liftControl(gamepad2);
        grabberControl(gamepad2);

        if(time.time() > 996)
        {
            gamepad2.runLedEffect(controller1.ledB);
            time.reset();
        }

        if(cds.getDistance() < 0.5 && !grabbed && lift.lastPlace == 0)
        {
            gamepad2.runRumbleEffect(controller1.rumbleA);
        }

        lift.liftSafetyCheck();
    }

    public void driveControl (Gamepad gamepad1)
    {
        chassis.xyrMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(controller1.button(15,(gamepad1.left_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient - 0.2)));
        if(controller1.button(16,(gamepad1.right_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient + 0.2)));

    }

    public void MPCRControl (Gamepad gamepad1)
    {
        if(controller1.button(10,gamepad1.left_bumper))
            coneRight = !coneRight;
        if(coneRight)
        {
            if (lift.lastPlace <= 7)
                lift.liftSet(7);
            mpcr.preSetMPCR(3);
        }
        else
            mpcr.preSetMPCR(0);
    }

    public void liftControl (Gamepad gamepad2)
    {
        if(!coneRight) {
            lift.setManual((int) (-gamepad2.right_stick_y * 100));
            int level;
            if(grabbed)
            {
                if(lift.lastPlace == 0)
                    level = 1;
                level = lift.lastPlace;
                if(controller2.button(0, gamepad2.a))
                {
                    if(stackIterator > 1)
                        stackIterator--;
                    else if(stackIterator == 1)
                        stackIterator = 5;
                    level = stackIterator;
                }
                if(controller2.button(1, gamepad2.b))
                    level = 7;
                if(controller2.button(2, gamepad2.x))
                    level = 8;
                if(controller2.button(3, gamepad2.y))
                    level = 9;
            }
            else
            {
                level = lift.lastPlace;
                if(controller2.button(0, gamepad2.a))
                    level = 0;
                if(controller2.button(1, gamepad2.b))
                    level = 0;
                if(controller2.button(2, gamepad2.x))
                    level = 0;
                if(controller2.button(3, gamepad2.y))
                    level = 0;
            }
            lift.liftSet(level);
        }
    }

    public void grabberControl(Gamepad gamepad2)
    {
        if(controller2.button(11, gamepad2.right_bumper))
            grabbed = !grabbed;
        if(grabbed)
        {
            grabber.setShift(gamepad2.left_stick_x);
            grabber.grab(true);
        } else
            grabber.grab(false);
    }

    public void playerCheck(Gamepad gamepad1)
    {
        if(controller1.button(8, gamepad1.left_stick_button))
            if(controller1.button(13, gamepad1.start))
                singlePlayer = !singlePlayer;
    }

    /**
     * Updates the imu and odo heading data.
     */
    public void dataLoop()
    {
        gyro.updateHeading();
        odometry.updatePosition();
    }

    /**
     * Adds useful information to display on driver hub during teleOp.
     * @param telemetryIn is a thing that needs to be passed in, in order to work.
     */
    public void robotTelemetry(Telemetry telemetryIn)
    {
        Heading temp = odometry.convertToHeading();
        telemetryIn.addData( "Speed Coefficient: ", chassis.speedCoefficient);
        telemetryIn.addData( "X: ", temp.x);
        telemetryIn.addData( "Y: ", temp.y);
        telemetryIn.addData( "ODO Heading: ", temp.h);
        telemetryIn.addData( "IMU Heading: ", gyro.getHeading());
        telemetryIn.addData( "last level: ", lift.lastPlace);
        telemetryIn.addData( "right Lift: ", lift.rightLift.getCurrentPosition());
        telemetryIn.addData( "left Lift: ", lift.leftLift.getCurrentPosition());
        telemetryIn.addData( "time ", time.time());

        telemetryIn.update();
    }
}
