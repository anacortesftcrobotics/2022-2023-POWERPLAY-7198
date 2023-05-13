package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.kaicode.OdoController;

/**
 * This class is an intermediary between the teleOp and autoOp classes and all the sub-assembly classes on the 2022-2023 powerplay robot.
 * @author      Lemon
 * @author      kaiwallis
 */

public class Robot {
    /**
     * Empty constructor
     */
    public Robot () {}

    //Subassembly Objects
    Chassis chassis = new Chassis();
    Gyro gyro = new Gyro();
    OdoController odometry = new OdoController();

    ElapsedTime time = new ElapsedTime();

    Controller controller1 = new Controller();
    Controller controller2 = new Controller();

    Logger logger = Logger.getInstance();


    /**
     * Initializes all hardware in all subsystem classes.
     */
    public void initializeHardware (HardwareMap hardwareMap) {
        chassis.initializeHardware(hardwareMap);
        gyro.initializeHardware(hardwareMap);
        odometry.initializeHardware(hardwareMap);
    }

    /**
     * Sets the robot to the default, starting position.
     */
    public void robotDefaultState() {
        chassis.brake();
    }

    /**
     * The whole teleOp control scheme. Does driving, mpcr, lift, grabbing, etc.
     * @param gamepad1 is the first controller used.
     * @param gamepad2 is the second controller used.
     */
    public void driveLoop(Gamepad gamepad1, Gamepad gamepad2) {
        driveControl(gamepad1);
    }

    /**
     * This is the method that runs the driving of the chassis.
     * @param gamepad1 is the controller used to drive.
     */
    public void driveControl (Gamepad gamepad1) {
        chassis.xyrMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(controller1.button(15,(gamepad1.left_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient - 0.25)));
        if(controller1.button(16,(gamepad1.right_trigger > 0.5)))
            chassis.setSpeedCoefficient(Math.max(-1, Math.min(1, chassis.speedCoefficient + 0.25)));
    }

    /**
     * Updates the imu and odo heading data.
     */
    public void dataLoop() {
        gyro.updateHeading();
        odometry.update();
    }

    /**
     * Adds useful information to display on driver hub during teleOp.
     * @param telemetryIn is a thing that needs to be passed in, in order to work.
     */
    public void robotTelemetry(Telemetry telemetryIn) {
        int i = 0;

//        telemetryIn.addData("LF",  chassis.leftFront.getPower());
//        telemetryIn.addData("LB",  chassis.leftBack.getPower());
//        telemetryIn.addData("RF",  chassis.rightFront.getPower());
//        telemetryIn.addData("RB",  chassis.rightBack.getPower());
        telemetryIn.addData( "Speed Coefficient: ", chassis.speedCoefficient);
        telemetryIn.addData( "IMU Heading: ", gyro.getHeading());
        telemetryIn.addData( "time ", time.time());

        telemetryIn.addData("x: ", "%.4f", odometry.getX());
        telemetryIn.addData("y: ", "%.4f", odometry.getY());
        telemetryIn.addData("h:  ", "%.4f", odometry.getHeading());

//        telemetryIn.addData("l1: ", Logger.getLine1());
//        telemetryIn.addData("l2: ", Logger.getLine2());
//        telemetryIn.addData("l3: ", Logger.getLine3());
    }

    public void sleep(int input) {
        try {
            Thread.sleep(input);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
