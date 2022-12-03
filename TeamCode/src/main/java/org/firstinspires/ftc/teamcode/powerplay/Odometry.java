package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class runs the odometry system on the 2022-2023 powerplay robot.
 * @author      Lemon
 */

public class Odometry implements SubsystemManager{

    public Odometry ()
    {

    }

    double[] storage = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double CMPT = (((3.5 / 2.54) * Math.PI) / 8192),distA = 7.1855, distB = 6.75, distC = 7.125, x = 0, y = 0, h = 0;

    private DcMotor leftBack, leftFront, rightBack, rightFront;
    private DcMotor encoderRight, encoderLeft, encoderBack;

    /**
     * This method registers the hardware used by this class.
     */
    public void initializeHardware (HardwareMap hardwareMap)
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
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderBack = rightFront;
        resetDriveEncoder();
    }

    /**
     * This method updates the position of the robot when given the encoder values of the odometry wheels.
     * @param left is the left encoder wheel position.
     * @param right is the right encoder wheel position.
     * @param back is the back encoder wheel position.
     */
    public void updatePosition(double left, double right, double back)
    {
        double deltaH;
        double deltaX;
        double deltaY;
        for(int i = 0; i <=2; i++) {
            storage[i] = storage[i + 2];
        }
        storage[3] = left * CMPT;
        storage[4] = right * CMPT;
        storage[5] = back * CMPT;
        for(int i = 0; i <=2; i++) {
            storage[i + 6] = storage[i + 3] - storage[i];
        }
        deltaH = (storage[6] / distA) - (storage[7] / distB);
        deltaX = ((storage[6] + storage[7]) / 2) * Math.cos(h) - (storage[8] - distC * deltaH) * Math.sin(h);
        deltaY = ((storage[6] + storage[7]) / 2) * Math.sin(h) + (storage[8] - distC * deltaH) * Math.cos(h);

        x += deltaX;
        y += deltaY;
        h += deltaH;
    }

    /**
     * This is an alternate method for updating the odometry values.
     */
    public void updatePosition()
    {
        double deltaH;
        double deltaX;
        double deltaY;

        for(int i = 0; i <= 2; i++) {
            storage[i] = storage[i + 2];
        }

        storage[3] = encoderLeft.getCurrentPosition() * CMPT;
        storage[4] = -encoderRight.getCurrentPosition() * CMPT;
        storage[5] = encoderBack.getCurrentPosition() * CMPT;

        for(int i = 0; i <= 2; i++) {
            storage[i + 6] = storage[i + 3] - storage[i];
        }

        deltaH = (storage[6] / distA) - (storage[7] / distB);
        deltaX = (((storage[6] + storage[7]) / 2) + deltaH) * Math.cos(h) - (storage[8] - distC * deltaH) * Math.sin(h);
        deltaY = (((storage[6] + storage[7]) / 2) + deltaH) * Math.sin(h) + (storage[8] - distC * deltaH) * Math.cos(h);

        x += deltaX;
        y += deltaY;
        h += deltaH;
    }

    public Heading getOdoHeading()
    {
        Heading temp = new Heading();
        temp.setHeading(getX(), getY(), getH());
        return temp;
    }

    public Heading convertToHeading(double x, double y, double h)
    {
        Heading temp = new Heading();
        temp.setHeading(x, y, h);
        return temp;
    }

    /**
     * Method that returns the x position from the odometry object.
     * @return the current x position.
     */
    public double getX()
    {
        return x;
    }

    /**
     * Method that returns the y position from the odometry object.
     * @return the current y position.
     */
    public double getY()
    {
        return y;
    }

    /**
     * Method that returns the heading from the odometry object.
     * @return the current heading in degrees
     */
    public double getH()
    {
        return Math.toDegrees(h);
    }

    /**
     * Method that sets the position and heading of the robot to the new values given.
     * @param newH is the new heading of the robot. Should be given in degrees.
     * @param newX is the new x position of the robot.
     * @param newY is the new y position of the robot.
     */
    public void setPosition(double newX, double newY, double newH)
    {
        for(int i = 0; i <=  8; i++) {
            storage[i] = 0.0;
        }
        x = newX;
        y = newY;
        h = Math.toRadians(newH);
    }

    /**
     * Returns the position of the left encoder wheel.
     * @return the position of the left encoder wheel in encoder ticks.
     */
    public double getLeft ()
    {
        return encoderLeft.getCurrentPosition();
    }

    /**
     * Returns the position of the right encoder wheel.
     * @return the position of the right encoder wheel in encoder ticks.
     */
    public double getRight ()
    {
        return encoderRight.getCurrentPosition();
    }

    /**
     * Returns the position of the back encoder wheel.
     * @return the position of the back encoder wheel in encoder ticks.
     */
    public double getBack ()
    {
        return encoderBack.getCurrentPosition();
    }

    /**
     * Resets the motor encoders.
     */
    public void resetDriveEncoder()
    {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
