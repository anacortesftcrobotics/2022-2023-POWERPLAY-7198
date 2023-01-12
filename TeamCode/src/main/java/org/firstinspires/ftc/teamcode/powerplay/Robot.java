package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chKai.Grabliftled;

/**
 * This class is an intermediary between the teleOp and autoOp classes and all the sub-assembly classes on the 2022-2023 powerplay robot.
 * @author      Lemon
 */

public class Robot {
    public Robot () {}
    //Variables
    boolean once = false;
    boolean coneRight = false;
    boolean grabbed = false;
    boolean beaconed = false;
    boolean singlePlayer = false;
    boolean hasGrabbedUp = false;
    int iLast;
    double[] depositState = {0.0,0.0,0.0,0.0,0.0,0.0};
    int depositDoing = 0;

    //Subassembly Objects
    Chassis chassis = new Chassis();
    Lift lift = new Lift();
    Grabber grabber = new Grabber();
    MPCR mpcr = new MPCR();

    CDS cds = new CDS();
    Pablo pablo = new Pablo();

    Gyro gyro = new Gyro();
    Odometry odometry = new Odometry();

    Led led = new Led();

    ElapsedTime time = new ElapsedTime();
    Grabliftled grabliftled = new Grabliftled();

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
        lift.initializeHardware(hardwareMap);
        grabber.initializeHardware(hardwareMap);
        mpcr.initializeHardware(hardwareMap);
        grabliftled.initializeHardware(hardwareMap);

        cds.initializeHardware(hardwareMap);
        pablo.initializeHardware(hardwareMap);

        gyro.initializeHardware(hardwareMap);
        odometry.initializeHardware(hardwareMap);

        led.initializeHardware(hardwareMap);

        //stuff
        rumbleA = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)
                .build();

        rumbleB = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0,1.0,500)
                .build();

        rumbleC = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0,0.0,200)
                .addStep(0.75,0.25,200)
                .addStep(0.5,0.5,200)
                .addStep(0.25,0.75,200)
                .addStep(0.0,1.0,200)
                .build();

        ledA = new Gamepad.LedEffect.Builder()
                .addStep(0.5,0.5,0.5,1000)
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
                .addStep(1.0,1.0,1.0,500)
                .addStep(0.0,0.0,0.0,500)
                .build();
    }

    /**
     * Sets the robot to the default, starting position.
     */
    public void robotDefaultState()
    {
        led.setLed("red");
        cds.onOffLED(false);
        chassis.brake();
        lift.liftZero();
        grabber.grab(false, false);
        mpcr.preSetMPCR(0);
    }

    /**
     * The whole teleOp control scheme. Does driving, mpcr, lift, grabbing, etc.
     * @param gamepad1 is the first controller used.
     * @param gamepad2 is the second controller used.
     */
    public void driveLoop(Gamepad gamepad1, Gamepad gamepad2)
    {
        driveControl(gamepad1);
        MPCRControl(gamepad1);
        liftControl(gamepad1);
        grabberControl(gamepad1);

        if(time.time() > 12)
        {
            gamepad1.runLedEffect(ledB);
            gamepad2.runLedEffect(ledB);
            time.reset();
        }

        if(cds.getDistance() < 0.5 && !grabbed && lift.doubleLastPlace <= 2)
        {
            gamepad1.runRumbleEffect(rumbleB);
            gamepad2.runRumbleEffect(rumbleB);
            led.setLed("green");
        }
        else
            led.setLed("violet");

        lift.liftSafetyCheck();
    }

    public void driveControl (Gamepad gamepad1)
    {
        chassis.xyrMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(controller1.button(15,(gamepad1.left_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient - 0.25)));
        if(controller1.button(16,(gamepad1.right_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient + 0.25)));
    }

    public void MPCRControl (Gamepad gamepad1)
    {
        if(controller1.button(10,gamepad1.left_bumper))
            coneRight = !coneRight;
        if(coneRight)
        {
            if (lift.doubleLastPlace <= 14)
                lift.liftSet(14.0);
            if(lift.leftLift.getCurrentPosition()  > 1000)
                mpcr.preSetMPCR(3);
        }
        else
            mpcr.preSetMPCR(0);
    }

    public void liftControl (Gamepad gamepad2)
    {
        if(!coneRight) {
            lift.setManual((int) (-gamepad2.left_stick_y * 100));
            double level= lift.doubleLastPlace;
            if (grabliftled.isGrabbedUp()&&hasGrabbedUp){
                level = 2.0;
            } else if (grabliftled.isGrabbedUp()&&!hasGrabbedUp) {
                level = lift.doubleLastPlace;
                if(controller2.button(4, gamepad2.dpad_down))
                    level = 0.5; //1
                if(controller2.button(5, gamepad2.dpad_left))
                    level = 4.25; //4
                if(controller2.button(6, gamepad2.dpad_right))
                    level = 3; //3
                if(controller2.button(7, gamepad2.dpad_up))
                    level = 5.5; //5

                if(controller2.button(0, gamepad2.a))
                    level = 1.75; //2
                if(controller2.button(1, gamepad2.b))
                    level = 14; //7
                if(controller2.button(2, gamepad2.x))
                    level = 24; //8
                if(controller2.button(3, gamepad2.y))
                    level = 34; //9
            }else if (!grabliftled.isGrabbedUp()) {
                level = lift.doubleLastPlace;
                level = 0;
            }
            hasGrabbedUp = grabliftled.isGrabbedUp();

            lift.liftSet(level);
        }
    }

    public void grabberControl(Gamepad gamepad2)
    {
        grabliftled.autoGrab(controller2.button(11, gamepad2.right_bumper));

    }

    public void resetOdoButOnlyLikeOnce(int i)
    {
        if(i != iLast)
        {
            odometry.setPosition(0,0,0);
            odometry.resetDriveEncoder();
        }
        iLast = i;
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
        int i = 0;
        Heading temp = odometry.convertToHeading(odometry.getX(), odometry.getY(), gyro.getHeading());

        telemetryIn.addData("isLooped",  chassis.inLoop);
        telemetryIn.addData("x end",  chassis.xEndState);
        telemetryIn.addData("y end",  chassis.yEndState);
        telemetryIn.addData("h end",  chassis.hEndState);
        telemetryIn.addData("x err",  chassis.xEndState - temp.x);
        telemetryIn.addData("y err",  chassis.yEndState - temp.y);
        telemetryIn.addData("h err",  chassis.hEndState - temp.h);
        telemetryIn.addData("LF",  chassis.leftFront.getPower());
        telemetryIn.addData("LB",  chassis.leftBack.getPower());
        telemetryIn.addData("RF",  chassis.rightFront.getPower());
        telemetryIn.addData("RB",  chassis.rightBack.getPower());
        telemetryIn.addData( "Speed Coefficient: ", chassis.speedCoefficient);
        telemetryIn.addData( "X: ", temp.x);
        telemetryIn.addData( "Y: ", temp.y);
        telemetryIn.addData( "ODO Heading: ", temp.h);
        telemetryIn.addData( "IMU Heading: ", gyro.getHeading());
        telemetryIn.addData( "last level: ", lift.doubleLastPlace);
        telemetryIn.addData( "right Lift pos: ", lift.rightLift.getCurrentPosition());
        telemetryIn.addData( "left Lift pos: ", lift.leftLift.getCurrentPosition());
        telemetryIn.addData( "change ", lift.liftChange);
        telemetryIn.addData( "manual ", lift.liftManual);
        telemetryIn.addData( "time ", time.time());
        telemetryIn.addData("grabbed ", grabliftled.grabbed());

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
    public boolean deposit(String liftLevel)
    {
        /*
        0 init x
        1 init y
        2 init h
        3 init lift height
        4 end lift height
        5 start time
         */
        boolean doneYet = false;

        switch(depositDoing) {
            case 1:
                depositState[0] = odometry.getX();
                depositState[1] = odometry.getX();
                depositState[2] = gyro.getHeading();
                depositState[3] = lift.doubleLastPlace;
                if(liftLevel.equals("low"))
                    depositState[4] = 15;
                else if(liftLevel.equals("med"))
                    depositState[4] = 25;
                else if(liftLevel.equals("high"))
                    depositState[4] = 35;
                else
                    depositState[4] = 15;
                depositState[5] = time.time();
                depositDoing++;
                break;
            case 2:
                lift.liftSet(depositState[4]);
                depositDoing++;
                break;
            case 3:
                // start scanning left and right until the distance sensors hit something
                    //move left until 0.5 sec or pablo spots something
                    //move right 1 sec or until pablo spots something
                    //19.4 right
                    //24.6 left
                    //etc
                depositDoing++;
                break;
            case 4:
                //Center on the pole
                    //get distance sensors to be equal / around the pole

                /*
                double leftDist = leftDistance.getDistance(DistanceUnit.CM);
                double rightDist = rightDistance.getDistance(DistanceUnit.CM);
                if (leftDist < rightDist && leftDist < 18)
                    return 0.05;
                if (rightDist < leftDist && rightDist < 18)
                    return -0.05;
                return 0;
                 */

                depositDoing++;
                break;
            case 5:
                //move up to pole
                    //get color to check what its seeing, if its red or blue, go to cone distance, if its not, go to pole distance
                if(true){
                    depositState[5] = time.time();
                    depositDoing++;
                }
                depositDoing++;
                break;
            case 6:
                lift.liftSet(lift.doubleLastPlace - 1);
                grabber.grab(false, false);
                if(time.time() - depositState[5] > 1.5)
                    depositDoing++;
                break;
            case 7:
                //back off
                depositDoing++;
                break;
            case 8:
                //reset
                //lower lift to initial position

                //return to starting position
                depositDoing++;
                break;
            case 9:
                doneYet = true;
                depositDoing++;
                break;
        }
        if(doneYet)
        {
            chassis.brake();
            return true;
        }
        return false;
    }
}
