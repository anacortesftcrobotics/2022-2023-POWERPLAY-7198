package org.firstinspires.ftc.teamcode.powerplay;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Thread.sleep;

/**
 * This class runs the drive motors of the 2022 - 2023 powerplay robot.
 * @author Lemon
 */

public class Chassis
{
    public Chassis ()
    {

    }

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    boolean inLoop = false;
    double speedCoefficient = 0.5;
    // 0 : X Position
    // 1 : Y Position
    // 2 : Heading
    // 3 : Distance Traveling
    double[] saveState = {0.0,0.0,0.0,0.0};

    /**
     * This method registers the hardware used by this class.
     * In this case, it's the four drive motors used by the robot.
     */
    public void initializeHardware(HardwareMap hardwareMap)
    {
        //leftBack
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftFront
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightBack
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightFront
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * This function takes in x, y, and xr values, and tells the motors to move accordingly.
     * @param x is the left-right movement.
     * @param y is the forwards-backwards movement.
     * @param rx is the rotation.
     */
    public void xyrMovement(double x, double y, double rx)
    {
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        x = deadZone(x);
        y = deadZone(y);
        rx = deadZone(rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower = Math.cbrt(leftBackPower);
        rightBackPower = Math.cbrt(rightBackPower);
        leftFrontPower = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBackPower = (leftBackPower * speedCoefficient * 0.9);
        rightBackPower = (rightBackPower * speedCoefficient * 0.9);
        leftFrontPower = (leftFrontPower  * speedCoefficient * 0.9);
        rightFrontPower = (rightFrontPower * speedCoefficient * 0.9);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
    }

    /**
     * This function tells the robot to move forwards or backwards a certain amount of inches.
     * It is specially designed to be called in a certain type of control loop.
     * @param dist is the amount of inches to move.
     * @param heading is an object that tells the robot where it is, so it can change the motor power values to course correct.
     * @return a boolean that says if the robot has reached its destination yet.
     */
    public boolean move(double dist, Heading heading)
    {
        boolean doneYet = false;

        if(!inLoop)
        {
            saveState(heading.x, heading.y, heading.h);
            inLoop = true;
        }

        double xEndState = dist * Math.cos(Math.toRadians(saveState[2] + 90));
        double yEndState = dist * Math.sin(Math.toRadians(saveState[2] + 90));
        double hEndState = saveState[2];

        double xError = xEndState - heading.x;
        double yError = yEndState - heading.y;
        double hError = hEndState - heading.h;

        double trackError = Math.abs(dist) - Math.sqrt(Math.pow((heading.x), 2) + Math.pow((heading.y), 2));
        double translatedError = Math.max(-1, Math.min(1, Math.cbrt(trackError) * (1 / 3)));
        double correction = hError / 50;

        if(Math.abs(xError) < 0.5 && Math.abs(yError) < 0.5 && Math.abs(hError) < 5)
        {
            doneYet = true;
        }

        if(!doneYet)
        {
            leftBack.setPower(translatedError - correction);
            rightBack.setPower(translatedError + correction);
            leftFront.setPower(translatedError - correction);
            rightFront.setPower(translatedError + correction);
        }

        if(doneYet)
        {
            inLoop = false;
            brake();
        }
        return doneYet;
    }

    /**
     * This function tells the robot to move left or right a certain amount of inches.
     * It is specially designed to be called in a certain type of control loop.
     * @param dist is the amount of inches to move.
     * @param heading is an object that tells the robot where it is, so it can change the motor power values to course correct.
     * @return a boolean that says if the robot has reached its destination yet.
     */
    public boolean strafe(double dist, Heading heading)
    {
        boolean doneYet = false;

        if(!inLoop)
        {
            saveState(heading.x, heading.y, heading.h);
            inLoop = true;
        }

        double xEndState = dist * Math.cos(Math.toRadians(saveState[2]));
        double yEndState = dist * Math.sin(Math.toRadians(saveState[2]));
        double hEndState = saveState[2];

        double xError = xEndState - heading.x;
        double yError = yEndState - heading.y;
        double hError = hEndState - heading.h;

        double trackError = Math.abs(dist) - Math.sqrt(Math.pow((heading.x), 2) + Math.pow((heading.y), 2));
        double translatedError = Math.max(-1, Math.min(1, Math.cbrt(trackError) * (1 / 3)));
        double correction = hError / 50;

        if(Math.abs(xError) < 0.5 && Math.abs(yError) < 0.5 && Math.abs(hError) < 5)
        {
            doneYet = true;
        }

        if(!doneYet)
        {
            leftBack.setPower(-translatedError - correction);
            rightBack.setPower(translatedError + correction);
            leftFront.setPower(translatedError - correction);
            rightFront.setPower(-translatedError + correction);
        }

        if(doneYet)
        {
            inLoop = false;
            brake();
        }
        return doneYet;
    }

    /**
     * This function tells the robot to turn a certain amount of degrees.
     * It is specially designed to be called in a certain type of control loop.
     * @param degree is the amount of degrees to turn.
     * @param heading is an object that tells the robot where it is, so it can change the motor power values to course correct.
     * @return a boolean that says if the robot has reached its destination yet.
     */
    public boolean turn(double degree, Heading heading)
    {
        boolean doneYet = false;

        if(!inLoop)
        {
            saveState(heading.x, heading.y, heading.h);
            inLoop = true;
        }

        double xEndState = saveState[0];
        double yEndState = saveState[1];
        double hEndState = saveState[2] + degree;

        double xError = xEndState - heading.x;
        double yError = yEndState - heading.y;
        double hError = hEndState - heading.h;

        double trackError = hError;
        double translatedError = Math.max(-1, Math.min(1, Math.cbrt(trackError) * (1 / 3)));

        if(Math.abs(xError) < 0.5 && Math.abs(yError) < 0.5 && Math.abs(hError) < 5)
        {
            doneYet = true;
        }

        if(!doneYet)
        {
            leftBack.setPower(translatedError);
            rightBack.setPower(translatedError);
            leftFront.setPower(-translatedError);
            rightFront.setPower(-translatedError);
        }

        if(doneYet)
        {
            inLoop = false;
            brake();
        }
        return doneYet;
    }

    /**
     * This function takes in positional data and saves it to a local storage array for easy access.
     * @param x
     * @param y
     * @param h
     */
    public void saveState(double x, double y, double h)
    {
        saveState[0] = x;
        saveState[1] = y;
        saveState[2] = h;
    }

    /**
     * This function tells the robot to stop moving.
     */
    public void brake()
    {
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /**
     * This should tell a certain motor to move at a power level, for an amount of time.
     * DO NOT use this function in competition code.
     * @param motor is the motor to move.
     * @param power is the power level to move the motor at.
     * @param length is how long the motor should run for.
     * @throws InterruptedException
     */
    public void motorTest(DcMotor motor, double power, int length) throws InterruptedException
    {
        motor.setPower(power);
        sleep(length);
    }

    /**
     * This method changes the speed coefficient, which is used in XYR movement.
     * @param input is the new value that speedCoefficient will be set to.
     */
    public void setSpeedCoefficient(double input)
    {
        double temp = Math.max(input, 0.25);
        temp = Math.min(temp, 1);
        temp *= 100;
        temp = Math.round(temp);
        temp /= 100;
        speedCoefficient = temp;
    }

    /**
     * This method takes an input, and determines if it's lower than 0.1  and returns 0 if it is. Otherwise, it should just return the value it was given.
     * @param input is the value to be checked.
     * @return the processed value.
     */
    public double deadZone(double input)
    {
        if (Math.abs(input) < 0.1)
        {
            return  0;
        }
        else
        {
            return input;
        }
    }

    /**
     * This method takes a number and checks if its positive or negative.
     * @param input the number that is being checked.
     * @return -1 or 1. Or 0 I guess. But don't give it 0.
     */
    public double parity (double input)
    {
        return input / Math.abs(input);
    }
}
