package org.firstinspires.ftc.teamcode.mentorbot;

/**
 * An interface for various types of error-based process control functions
 *
 * @author Mentor Liam
 */
public interface ErrorController {

    /**
     * Resets any tracked data like total delta or error. Calling this function should return the ErrorController
     * to the same state as one freshly constructed.
     */
    void reset();

    /**
     * Processes the error to determine a response value.
     *
     * @param error the difference between the expected and actual value
     * @param delta the amount of time since the last error reading
     * @return the response value given by the control function
     */
    double processFrame(double error, double delta);
}
