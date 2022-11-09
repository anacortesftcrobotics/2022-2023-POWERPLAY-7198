package org.firstinspires.ftc.teamcode.mentorbot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * An interface for robot translocators
 *
 * @author Mentor Liam
 */
public interface Chassis extends Subsystem {

    /**
     *  Sets the power of the motors on the chassis based on dimensioned normal vectors.
     *  Assumes that the positive x-axis of the robot is forward pointing.
     *
     * @param x - the x vector size
     * @param r - the rotation vector size
     */
    void setPowerXR(double x, double r);

    /**
     *  Sets the distance the motors should travel automatically based on dimensioned vectors
     *  The size of vectors x and y are in this Chassis' current {@link DistanceUnit}, and r is in this Chassis'
     *  current {@link AngleUnit}
     *
     * @param x - the x vector size
     * @param r - the rotation vector size
     */
    void moveDistanceXR(double x, double r);

    AngleUnit getAngleUnit();
    void setAngleUnit(AngleUnit unit);

    DistanceUnit getDistanceUnit();
    void setDistanceUnit(DistanceUnit unit);
}