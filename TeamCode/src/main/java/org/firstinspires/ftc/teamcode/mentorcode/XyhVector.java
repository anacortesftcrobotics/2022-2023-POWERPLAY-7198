package org.firstinspires.ftc.teamcode.mentorcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * XyhVector class used keeps track of one reading Odometry x, y, and heading (h)
 * @author Coach Jenkins
 */

public class XyhVector {
    /**
     * private properties for each limit switch
     */
    private double x, y;
    public double h, x_cm, y_cm, cm_per_tick;

    /**
     * Constructor for XyhVector
     *
     * @param x             the x-coordinate of odometry
     * @param y             the y-coordinate of odometry
     * @param h             the heading of odometry
     * @param cm_per_tick  ticks to convert to cm
     */
    public XyhVector(double x, int y, double h, double cm_per_tick) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.cm_per_tick = cm_per_tick;
    }

    public double hcos() {
        return Math.cos(h);
    }

    public double hsin() {
        return Math.sin(h);
    }

    // Put code here that you want to execute when the value has changed.
    public void setX(double val) {
        this.x = val;
        this.x_cm = val * cm_per_tick;
    }

    public double getX() {
        return this.x;
    }

    public void setY(double val) {
        this.y = val;
        this.y_cm = val * cm_per_tick;
    }

    public double getY() {
        return this.y;
    }
}
