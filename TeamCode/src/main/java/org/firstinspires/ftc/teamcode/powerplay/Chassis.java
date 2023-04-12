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
    /**
     * Empty constructor.
     */
    public Chassis ()
    {

    }

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public boolean inLoop = false;
    public double speedCoefficient = 0.5;
    // 0 : X Position
    // 1 : Y Position
    // 2 : Heading
    // 3 : Distance Traveling
    public double[] saveState = {0.0,0.0,0.0,0.0};

    public double xEndState = 0.0;
    public double yEndState = 0.0;
    public double hEndState = 0.0;

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
     * @param cx is the amount of inches to move on the x-axis
     * @param cy is the amount of inches to move on the y-axis
     * @param crx is the amount of degrees to turn.
     * @param heading is an object that tells the robot where it is, so it can change the motor power values to course correct.
     * @return a boolean that says if the robot has reached its destination yet.
     */
    public boolean move(double cx, double cy, double crx, Heading heading)
    {
        boolean doneYet = false;

        if(!inLoop)
        {
            //saveState(heading.x, heading.y, heading.h);
            xEndState = saveState[0] + cx;
            yEndState = saveState[1] + cy;
            hEndState = saveState[2] + crx;
            inLoop = true;
        }

        double xError = (xEndState - saveState[0]) - (heading.x - saveState[0]);
        double yError = (yEndState - saveState[1]) - (heading.y - saveState[1]);
        double hError = (hEndState - saveState[2]) - (heading.h - saveState[2]);

        if(Math.abs(xError) < 0.5 && Math.abs(yError) < 0.5 && Math.abs(hError) < 5)
        {
            doneYet = true;
        }

        if(!doneYet)
        {
            //xyrMovement(restrict(yError), restrict(xError), restrict(-hError / 10));
            double yMin = restrictSet(((yError - 1) / 8), -0.4, -0.3);
            double yMax = restrictSet(((yError - 1) / 8), 0.3, 0.4);
            double xMin = restrictSet(((xError - 1) / 22), -0.4, -0.15);
            double xMax = restrictSet(((xError - 1) / 22), 0.15, 0.4);
            xyrMovement(restrictSet(yError,yMin, yMax), restrictSet(xError,xMin, xMax), restrict(-hError / 10));
        }

        if(doneYet)
        {
            saveState = new double[] {0.0, 0.0, 0.0};
            xEndState = 0.0;
            yEndState = 0.0;
            hEndState = 0.0;
            inLoop = false;
            brake();
            return true;
        }
        return false;
    }

    /**
     * This function takes in positional data and saves it to a local storage array for easy access.
     * @param x is the x position given to the function.
     * @param y is the y position given to the function.
     * @param h is the heading, in degrees given to the function.
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
     * Restricts a value to between -1 and 1.
     * @param input is the value to be restricted.
     * @return the value after being restricted.
     */
    public double restrict(double input)
    {
        return Math.max(-1, Math.min(1, input));
    }

    /**
     * This function will restrict a value between a given minimum and maximum.
     * @param input is the input value that is being processed.
     * @param min is the allowed minimum.
     * @param max is the allowed maximum.
     * @return the input value after being restricted.
     */
    public double restrictSet(double input, double min, double max)
    {
        return Math.max(min, Math.min(max, input));
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
