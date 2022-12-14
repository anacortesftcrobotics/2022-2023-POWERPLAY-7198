package org.firstinspires.ftc.teamcode.mentorbot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * An interface for various types of extension subsystems.
 *
 * @author Mentor Liam
 */
public abstract class PrismaticJoint extends Subsystem {

    /**
     * Registers a position for future reference via String
     * @param name the String used to reference the position
     * @param extensionDistance the distance of the position
     * @param unit the DistanceUnit of the extensionDistance
     */
    public abstract void registerPosition(String name, double extensionDistance, DistanceUnit unit);

    public abstract boolean removePosition(String name);

    public abstract void moveToRegisteredPosition(String name);

    public abstract void moveToPosition(double extensionDistance, DistanceUnit unit);
}
