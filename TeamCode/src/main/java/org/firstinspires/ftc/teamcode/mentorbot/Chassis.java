package org.firstinspires.ftc.teamcode.mentorbot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * An interface for robot translocators
 *
 * @author Mentor Liam
 */
public abstract class Chassis extends Subsystem {

    DistanceUnit distanceUnit;

    AngleUnit angleUnit;

    /**
     *  Sets the power of the motors on the chassis based on dimensioned normal vectors.
     *  Assumes that the positive x-axis of the robot is forward pointing.
     *
     * @param x the x vector size
     * @param r the rotation vector size
     */
    public abstract void setPowerXR(double x, double r);

    /**
     *  Sets the distance the motors should travel automatically based on dimensioned vectors
     *  The size of vectors x and y are in this Chassis' current {@link DistanceUnit}, and r is in this Chassis'
     *  current {@link AngleUnit}
     *
     * @param x the x vector size
     * @param r the rotation vector size
     */
    public abstract void moveDistanceXR(double x, double r);

    public AngleUnit getAngleUnit() {
        return angleUnit;
    }
    public void setAngleUnit(AngleUnit unit) {
        this.angleUnit = unit;
    }

    public DistanceUnit getDistanceUnit() {
        return  distanceUnit;
    }
    public void setDistanceUnit(DistanceUnit unit) {
        this.distanceUnit = unit;
    }
}