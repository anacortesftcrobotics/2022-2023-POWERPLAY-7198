package org.firstinspires.ftc.teamcode.mentorbot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * An interface for various types of extension subsystems.
 *
 * @author Mentor Liam
 */
public interface Extendable extends Subsystem {

    /**
     * Registers a position for future reference via String
     * @param name the String used to reference the position
     * @param extensionDistance the distance of the position
     * @param unit the DistanceUnit of the extensionDistance
     */
    void registerPosition(String name, double extensionDistance, DistanceUnit unit);

    boolean removePosition(String name);

    void moveToRegisteredPosition(String name);

    void moveToPosition(double extensionDistance, DistanceUnit unit);
}
