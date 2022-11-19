package org.firstinspires.ftc.teamcode.mentorbot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * An Interface for robot translocators that are capable of moving in both planar axes.
 *
 * @author Mentor Liam
 */
public interface OmnidirectionalChassis extends Chassis {

    /**
     * Sets the power of the motors on the chassis based on dimensioned normal vectors.
     * Assumes that the positive x-axis of the robot is forward pointing.
     *
     * @param x the x vector size
     * @param y the y vector size
     * @param r the rotation vector size
     */
    void setPowerXYR(double x, double y, double r);

    /**
     * Sets the distance the motors should travel automatically based on dimensioned vectors
     * The size of vectors x and y are in this Chassis' current {@link DistanceUnit}, and r is in this Chassis'
     * current {@link AngleUnit}
     *
     * @param x the x vector size
     * @param y the y vector size
     * @param r the rotation vector size
     */
    void moveDistanceXYR(double x, double y, double r);
}