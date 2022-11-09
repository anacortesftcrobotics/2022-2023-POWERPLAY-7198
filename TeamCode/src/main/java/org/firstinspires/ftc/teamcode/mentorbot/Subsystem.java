package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A Subsystem represents a collection of hardware components that work together to perform a task on the robot.
 *
 * @author Mentor Liam
 */
public interface Subsystem {

    /**
     * Registers the hardware devices used by the subsystem.
     * The HardwareMap must belong to the currently running opMode
     *
     * @param hardwareMap - the HardwareMap of the connected robot
     * @return true if the subsystem successfully found every HardwareDevice needed
     */
    boolean registerHardware(HardwareMap hardwareMap);

    /**
     * Checks the status of all devices required by the Subsystem and provides them as a string.
     * The format will be as follows:
     * <p>
     * <code>device: status[\n...]</code>
     *
     * @return the String of concatenated device statuses for this subsystem
     */
    String getHardwareStatus();
}
