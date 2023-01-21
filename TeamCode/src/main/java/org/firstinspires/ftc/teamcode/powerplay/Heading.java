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

    public Heading()
    {

    }

    public void setHeading(double newX, double newY, double newH)
    {
        x = newX;
        y = newY;
        h = newH;
    }

    public void setX (double input)
    {
        x = input;
    }

    public void setY (double input)
    {
        y = input;
    }

    public void setH (double input)
    {
        h = input;
    }
    public double getH () {return h;}
}
