package org.firstinspires.ftc.teamcode.powerplay;

/**
 * This class is for passing and using the position and heading data of the robot more efficiently.
 * @author Lemon
 */

public class Heading
{
    double x = 0;
    double y = 0;
    double h = 0;

    /**
     * Empty constructor
     */
    public Heading()
    {

    }

    /**
     * Method to set the value of a heading object.
     * @param newX is the new X value that is being set.
     * @param newY is the new Y value that is being set.
     * @param newH is the new heading value that is being set.
     */
    public void setHeading(double newX, double newY, double newH)
    {
        x = newX;
        y = newY;
        h = newH;
    }

    /**
     * This method individually sets the X value of the heading object.
     * @param input
     */
    public void setX (double input)
    {
        x = input;
    }

    /**
     * This method individually sets the Y value of the heading object.
     * @param input
     */
    public void setY (double input)
    {
        y = input;
    }

    /**
     * This method individually sets the heading value of the heading object.
     * @param input
     */
    public void setH (double input)
    {
        h = input;
    }
    public double getH () {return h;}
}
