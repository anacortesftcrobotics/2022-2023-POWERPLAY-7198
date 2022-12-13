package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A Subsystem represents a collection of hardware components that work together to perform a task on the robot.
 *
 * @author Mentor Liam
 */
public abstract class Subsystem {

    String name;

    /**
     * Registers the hardware devices used by the subsystem.
     * The HardwareMap must belong to the currently running opMode
     *
     * @param hardwareMap the HardwareMap of the connected robot
     */
    public abstract void registerHardware(HardwareMap hardwareMap);

    /**
     * Checks the status of all devices required by the Subsystem and provides them as a string.
     * The format will be as follows:
     * <p>
     * <code>device: status[\n...]</code>
     *
     * @return the String of concatenated device statuses for this subsystem
     */
    public abstract String getHardwareStatus();

    /**
     * Updates the Subsystem operation based on user input
     *
     * @param gamepad1 should match TeleOp's gamepad1
     * @param gamepad2 should match TeleOp's gamepad2
     * @return the current status of the Subsytem
     */
    public abstract String update(Gamepad gamepad1, Gamepad gamepad2);

    public abstract void stop();
}
