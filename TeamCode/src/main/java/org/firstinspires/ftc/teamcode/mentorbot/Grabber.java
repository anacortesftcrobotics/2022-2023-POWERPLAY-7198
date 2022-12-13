package org.firstinspires.ftc.teamcode.mentorbot;

/**
 * A grabber is a subsystem designed to hold onto items during the game.
 *
 * @author Mentor Liam
 */
public abstract class Grabber extends Subsystem {

    String name = "GRAB";
    public abstract void grab();

    public abstract void release();
}