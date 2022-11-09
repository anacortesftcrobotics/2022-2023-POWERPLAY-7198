package org.firstinspires.ftc.teamcode.mentorbot;

/**
 * A grabber is a subsystem designed to hold onto items during the game.
 *
 * @author Mentor Liam
 */
public interface Grabber extends Subsystem {

    void grab();

    void release();
}