package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.kaicode.*;
import org.firstinspires.ftc.teamcode.odometry.OdoController;

/**
 * This class is an intermediary between the teleOp and autoOp classes and all the sub-assembly classes on the 2022-2023 powerplay robot.
 * @author      Lemon
 */

public class Robot {
    /**
     * Empty constructor
     */
    public Robot () {}

    //Variables
//    boolean once = false;
//    boolean coneRight = false;
//    boolean grabbed = false;
//    boolean beaconed = false;
//    boolean singlePlayer = false;
//    int iLast;

    double[] depositState = {0.0,0.0,0.0,0.0,0.0,0.0};
    int depositDoing = 1;

    //Subassembly Objects
    Chassis chassis = new Chassis();
//    Lift lift = new Lift();
//    Grabber grabber = new Grabber();
//    MPCR mpcr = new MPCR();

//    CDS cds = new CDS();
//    Pablo pablo = new Pablo();

    Gyro gyro = new Gyro();
    OdoController odometry = new OdoController();

//    Odo3 odo1 = new Odo3();
//    Odo2 odo3 = new Odo2();

//    Led led = new Led();

    ElapsedTime time = new ElapsedTime();

    Controller controller1 = new Controller();
    Controller controller2 = new Controller();

    //stuff
    Gamepad.RumbleEffect rumbleA;
    Gamepad.RumbleEffect rumbleB;
    Gamepad.RumbleEffect rumbleC;

    Gamepad.LedEffect ledA;
    Gamepad.LedEffect ledB;
    Gamepad.LedEffect ledC;

    /**
     * Initializes all hardware in all subsystem classes.
     */
    public void initializeHardware (HardwareMap hardwareMap)
    {
        chassis.initializeHardware(hardwareMap);
//        lift.initializeHardware(hardwareMap);
//        grabber.initializeHardware(hardwareMap);
//        mpcr.initializeHardware(hardwareMap);
//
//        cds.initializeHardware(hardwareMap);
//        pablo.initializeHardware(hardwareMap);

        gyro.initializeHardware(hardwareMap);
//        odometry.initializeHardware(hardwareMap);
//
//        led.initializeHardware(hardwareMap);

        //stuff
        rumbleA = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)
                .build();

        rumbleB = new Gamepad.RumbleEffect.Builder()
                .addStep(0.2,1.0,250)
                .build();

        rumbleC = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0,0.0,200)
                .addStep(0.75,0.25,200)
                .addStep(0.5,0.5,200)
                .addStep(0.25,0.75,200)
                .addStep(0.0,1.0,200)
                .build();

        ledA = new Gamepad.LedEffect.Builder()
                .addStep(0.0,0.0,0.0,1000)
                .build();

        ledB = new Gamepad.LedEffect.Builder()
                .addStep(1.0,0.0,0.0,1000)
                .addStep(1.0,0.5,0.0,1000)
                .addStep(1.0,1.0,0.0,1000)
                .addStep(0.5,1.0,0.0,1000)
                .addStep(0.0,1.0,0.0,1000)
                .addStep(0.0,1.0,0.5,1000)
                .addStep(0.0,1.0,1.0,1000)
                .addStep(0.0,0.5,1.0,1000)
                .addStep(0.0,0.0,1.0,1000)
                .addStep(0.5,0.0,1.0,1000)
                .addStep(1.0,0.0,1.0,1000)
                .addStep(1.0,0.0,0.5,1000)
                .build();

        ledC = new Gamepad.LedEffect.Builder()
                .addStep(0.0,1.0,0.0,250)
                .build();
    }

    /**
     * Sets the robot to the default, starting position.
     */
    public void robotDefaultState()
    {
//        led.setLed("violet");
//        cds.onOffLED(false);
        chassis.brake();
//        lift.liftZero();
//        grabber.grab(false, false);
//        mpcr.preSetMPCR(0);
//        lift.liftSet(0.5);
    }

    /**
     * The whole teleOp control scheme. Does driving, mpcr, lift, grabbing, etc.
     * @param gamepad1 is the first controller used.
     * @param gamepad2 is the second controller used.
     */
    public void driveLoop(Gamepad gamepad1, Gamepad gamepad2)
    {
        driveControl(gamepad1);
//        MPCRControl(gamepad1);
//        liftControl(gamepad2);
//        grabberControl(gamepad2);

//        if(cds.getDistance() < 0.5 && !grabbed && lift.doubleLastPlace <= 6)
//        {
//            gamepad1.runRumbleEffect(rumbleB);
//            gamepad2.runRumbleEffect(rumbleB);
//            gamepad1.runLedEffect(ledC);
//            gamepad2.runLedEffect(ledC);
//            led.setLed("green");
//        }
//        else
//            led.setLed("violet");
//
//        lift.liftSafetyCheck();
    }

    /**
     * This is the method that runs the driving of the chassis.
     * @param gamepad1 is the controller used to drive.
     */
    public void driveControl (Gamepad gamepad1)
    {
        chassis.xyrMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(controller1.button(15,(gamepad1.left_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient - 0.25)));
        if(controller1.button(16,(gamepad1.right_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient + 0.25)));
    }

//    /**
//     * This method controls the multipurpose cone righter.
//     * @param gamepad1 is the gamepad being used to control the MPCR.
//     */
//    public void MPCRControl (Gamepad gamepad1)
//    {
//        if(controller1.button(10,gamepad1.left_bumper))
//            coneRight = !coneRight;
//        if(coneRight)
//        {
//            if (lift.doubleLastPlace <= 14)
//                lift.liftSet(14.0);
//            if(lift.leftLift.getCurrentPosition()  > 1000)
//                mpcr.preSetMPCR(3);
//        }
//        else
//            mpcr.preSetMPCR(0);
//    }

//    /**
//     * This is the method that controls the lift.
//     * @param gamepad2 is th gamepad used to control the lift.
//     */
//    public void liftControl (Gamepad gamepad2)
//    {
//        if(!coneRight) {
//            lift.setManual((int) (-gamepad2.left_stick_y * 100));
//            double level;
//            if(grabbed)
//            {
//                level = lift.doubleLastPlace;
//                if(controller2.button(4, gamepad2.dpad_down))
//                    level = 0.5; //1
//                if(controller2.button(5, gamepad2.dpad_left))
//                    level = 4.25; //4
//                if(controller2.button(6, gamepad2.dpad_right))
//                    level = 3; //3
//                if(controller2.button(7, gamepad2.dpad_up))
//                    level = 5.5; //5
//
//                if(controller2.button(0, gamepad2.a))
//                    level = 1.75; //2
//                if(controller2.button(1, gamepad2.b))
//                    level = 14; //7
//                if(controller2.button(2, gamepad2.x))
//                    level = 24; //8
//                if(controller2.button(3, gamepad2.y))
//                    level = 34; //9
//            }
//            else
//            {
//                level = lift.doubleLastPlace;
//                if(controller2.button(4, gamepad2.dpad_down))
//                    level = 0; //1
//                if(controller2.button(5, gamepad2.dpad_left))
//                    level = 4.25; //4
//                if(controller2.button(6, gamepad2.dpad_right))
//                    level = 3; //3
//                if(controller2.button(7, gamepad2.dpad_up))
//                    level = 5.5; //5
//
//                if(controller2.button(0, gamepad2.a))
//                    level = 1.75; //0
//                if(controller2.button(1, gamepad2.b))
//                    level = 0; //0
//                if(controller2.button(2, gamepad2.x))
//                    level = 0; //0
//                if(controller2.button(3, gamepad2.y))
//                    level = 0; //0
//            }
//            lift.liftSet(level);
//        }
//    }

//    /**
//     * This is the method that controls the grabber. It allows for opening and closing, as well as shifting between regular and beacon grab.
//     * @param gamepad2 is the gamepad used to control the grabber.
//     */
//    public void grabberControl(Gamepad gamepad2)
//    {
//        if(controller2.button(11, gamepad2.right_bumper))
//        {
//            grabbed = !grabbed;
//            beaconed = false;
//        }
//
//        if(controller2.button(10, gamepad2.left_bumper))
//        {
//            grabbed = !grabbed;
//            beaconed = true;
//        }
//
//        if(grabbed)
//        {
//            grabber.setShift(gamepad2.left_stick_x);
//            grabber.grab(true, beaconed);
//        } else
//            grabber.grab(false, beaconed);
//    }

//    /**
//     * This is a method designed to be used in an auto loop. It will reset the odometry, but only once per step.
//     * @param i is the i value or the current step that the auto is on.
//     */
//    public void resetOdoButOnlyLikeOnce(int i)
//    {
//        if(i != iLast)
//        {
//            odometry.setPosition(0,0,0);
//            odometry.resetDriveEncoder();
//        }
//        iLast = i;
//    }

    /**
     * Updates the imu and odo heading data.
     */
    public void dataLoop()
    {
        gyro.updateHeading();
//        odometry.updatePosition();
    }

    /**
     * Adds useful information to display on driver hub during teleOp.
     * @param telemetryIn is a thing that needs to be passed in, in order to work.
     */
    public void robotTelemetry(Telemetry telemetryIn)
    {
        int i = 0;
//        Heading temp = odometry.convertToHeading(odometry.getX(), odometry.getY(), gyro.getHeading());
//
//        odo1.setEncoderPos((int)odometry.getLeft(), (int)-odometry.getRight(), (int)odometry.getBack());
//        telemetryIn.addData("1x",  odo1.getX());
//        telemetryIn.addData("1y",  odo1.getY());
//        telemetryIn.addData("1h",  odo1.getHDeg());
//        odo3.setEncoderPos((int)odometry.getLeft(), (int)-odometry.getRight(), (int)odometry.getBack());
//        telemetryIn.addData("3x",  odo3.getX());
//        telemetryIn.addData("3y",  odo3.getY());
//        telemetryIn.addData("3h",  odo3.getHDeg());

//        telemetryIn.addData("range",  cds.getDistance());
//        telemetryIn.addData("color",  cds.getColor());
//        telemetryIn.addData("left",  pablo.getLeftDistance());
//        telemetryIn.addData("right",  pablo.getRightDistance());
        telemetryIn.addData("isLooped",  chassis.inLoop);
        telemetryIn.addData("x end",  chassis.xEndState);
        telemetryIn.addData("y end",  chassis.yEndState);
        telemetryIn.addData("h end",  chassis.hEndState);
//        telemetryIn.addData("x err",  chassis.xEndState - temp.x);
//        telemetryIn.addData("y err",  chassis.yEndState - temp.y);
//        telemetryIn.addData("h err",  chassis.hEndState - temp.h);
        telemetryIn.addData("LF",  chassis.leftFront.getPower());
        telemetryIn.addData("LB",  chassis.leftBack.getPower());
        telemetryIn.addData("RF",  chassis.rightFront.getPower());
        telemetryIn.addData("RB",  chassis.rightBack.getPower());
        telemetryIn.addData( "Speed Coefficient: ", chassis.speedCoefficient);
//        telemetryIn.addData( "X: ", temp.x);
//        telemetryIn.addData( "Y: ", temp.y);
//        telemetryIn.addData( "ODO Heading: ", temp.h);
        telemetryIn.addData( "IMU Heading: ", gyro.getHeading());
//        telemetryIn.addData( "last level: ", lift.doubleLastPlace);
//        telemetryIn.addData( "right Lift pos: ", lift.rightLift.getCurrentPosition());
//        telemetryIn.addData( "left Lift pos: ", lift.leftLift.getCurrentPosition());
//        telemetryIn.addData( "change ", lift.liftChange);
//        telemetryIn.addData( "manual ", lift.liftManual);
        telemetryIn.addData( "time ", time.time());

        telemetryIn.update();
    }

    public void sleep(int input)
    {
        try {
            Thread.sleep(input);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
