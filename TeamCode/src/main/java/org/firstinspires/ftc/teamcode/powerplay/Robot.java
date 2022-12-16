package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.kaicode.*;

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
    int iLast;

    double[] depositState = {0.0,0.0,0.0,0.0,0.0,0.0};
    int depositDoing = 1;

    //Subassembly Objects
    Chassis chassis = new Chassis();
    Lift lift = new Lift();
    Grabber grabber = new Grabber();
    MPCR mpcr = new MPCR();

    CDS cds = new CDS();
    Pablo pablo = new Pablo();

    Gyro gyro = new Gyro();
    Odometry odometry = new Odometry();

    Odo3 odo1 = new Odo3();
    Odo2 odo3 = new Odo2();

    Led led = new Led();

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
        lift.initializeHardware(hardwareMap);
        grabber.initializeHardware(hardwareMap);
        mpcr.initializeHardware(hardwareMap);

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
        led.setLed("violet");
        cds.onOffLED(false);
        chassis.brake();
        lift.liftZero();
        grabber.grab(false, false);
        mpcr.preSetMPCR(0);
        lift.liftSet(0.5);
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
        liftControl(gamepad2);
        grabberControl(gamepad2);

        if(cds.getDistance() < 0.5 && !grabbed && lift.doubleLastPlace <= 6)
        {
            gamepad1.runRumbleEffect(rumbleB);
            gamepad2.runRumbleEffect(rumbleB);
            gamepad1.runLedEffect(ledC);
            gamepad2.runLedEffect(ledC);
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
            double level;
            if(grabbed)
            {
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
            }
            else
            {
                level = lift.doubleLastPlace;
                if(controller2.button(4, gamepad2.dpad_down))
                    level = 0; //1
                if(controller2.button(5, gamepad2.dpad_left))
                    level = 4.25; //4
                if(controller2.button(6, gamepad2.dpad_right))
                    level = 3; //3
                if(controller2.button(7, gamepad2.dpad_up))
                    level = 5.5; //5

                if(controller2.button(0, gamepad2.a))
                    level = 1.75; //0
                if(controller2.button(1, gamepad2.b))
                    level = 0; //0
                if(controller2.button(2, gamepad2.x))
                    level = 0; //0
                if(controller2.button(3, gamepad2.y))
                    level = 0; //0
            }
            lift.liftSet(level);
        }
    }

    public void grabberControl(Gamepad gamepad2)
    {
        if(controller2.button(11, gamepad2.right_bumper))
        {
            grabbed = !grabbed;
            beaconed = false;
        }

        if(controller2.button(10, gamepad2.left_bumper))
        {
            grabbed = !grabbed;
            beaconed = true;
        }

        if(grabbed)
        {
            grabber.setShift(gamepad2.left_stick_x);
            grabber.grab(true, beaconed);
        } else
            grabber.grab(false, beaconed);
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

        odo1.setEncoderPos((int)odometry.getLeft(), (int)-odometry.getRight(), (int)odometry.getBack());
        telemetryIn.addData("1x",  odo1.getX());
        telemetryIn.addData("1y",  odo1.getY());
        telemetryIn.addData("1h",  odo1.getHDeg());
        odo3.setEncoderPos((int)odometry.getLeft(), (int)-odometry.getRight(), (int)odometry.getBack());
        telemetryIn.addData("3x",  odo3.getX());
        telemetryIn.addData("3y",  odo3.getY());
        telemetryIn.addData("3h",  odo3.getHDeg());

        telemetryIn.addData("range",  cds.getDistance());
        telemetryIn.addData("color",  cds.getColor());
        telemetryIn.addData("left",  pablo.getLeftDistance());
        telemetryIn.addData("right",  pablo.getRightDistance());
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
    public boolean deposit(double liftLevel, Heading input, boolean cone)
    {
        boolean doneYet = false;
        switch(depositDoing) {
            case 1:
                //saves the initial position
                led.setLed("hot pink");
                depositState[0] = odometry.getX();
                depositState[1] = odometry.getX();
                depositState[2] = gyro.getHeading();
                depositState[3] = lift.doubleLastPlace;
                depositState[4] = liftLevel;
                depositState[5] = time.time();
                depositDoing++;
                break;
            case 2:
                //sets the lift to the right height
                led.setLed("dark red");
                lift.liftSet(liftLevel);
                depositState[5] = time.time();
                depositDoing++;
                break;
            case 3:
                //scans for the pole
                led.setLed("red");
                chassis.move(0,0,(15 * Math.cbrt(Math.sin(2 *  Math.PI * (time.time() - depositState[5])) * Math.pow((time.time() - depositState[5]),2)) / 57.2958),input);
                if(pablo.getLeftDistance() < 25 || pablo.getRightDistance() < 25)
                    depositDoing++;
                break;
            case 4:
                //center on pole
                led.setLed("red orange");
                double turn = 0;
                if(pablo.getLeftDistance() < pablo.getRightDistance())
                    turn = -2.5;
                if(pablo.getLeftDistance() > pablo.getRightDistance())
                    turn = 2.5;
                if(chassis.move(0,0,turn,input))
                    if(Math.round(pablo.getLeftDistance()) == Math.round(pablo.getRightDistance()))
                        depositDoing++;
                break;
            case 5:
                //move up to the pole
                led.setLed("orange");
                double dist = 0;
                boolean done = false;
                if(cone)
                    if(cds.getDistance() > 0.39)
                        if(chassis.move((((1.73559 * cds.getDistance()) - 0.405826) - 0.27),0,0,input))
                            done = true;
                else
                    if(cds.getDistance() > 1.55)
                        if(chassis.move((((1.73559 * cds.getDistance()) - 0.405826) - 2.28),0,0,input))
                            done = true;

                if(done){
                    depositState[5] = time.time();
                    depositDoing++;
                }
                break;
            case 6:
                led.setLed("gold");
                lift.liftSet(lift.doubleLastPlace - 1);
                grabber.grab(false, false);
                if(time.time() - depositState[5] > 1.5)
                    depositDoing++;
                break;
            case 7:
                led.setLed("yellow");
                if(chassis.move(-2,0,0, input))
                    depositDoing++;
                break;
            case 8:
                led.setLed("lawn green");
                lift.liftSet(depositState[3]);
                if(chassis.move(depositState[0],depositState[1],depositState[2],input))
                    depositDoing++;
                break;
            case 9:
                led.setLed("green");
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
