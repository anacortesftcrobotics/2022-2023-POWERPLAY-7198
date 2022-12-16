package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PowerplayGrabber extends Grabber {

    boolean grabStatus;

    Servo leftPaddle, rightPaddle;

    /**
     *
     */
    @Override
    public void grab() {
        // Idempotency
        if (grabStatus) {
            return;
        }

        leftPaddle.setPosition(0.6 + 0.02);
        rightPaddle.setPosition(1 - 0.6 + 0.02);
    }

    /**
     *
     */
    @Override
    public void release() {
        // Idempotency
        if (!grabStatus) {
            return;
        }

        leftPaddle.setPosition(0);
        rightPaddle.setPosition(1);
    }

    /**
     * Registers the hardware devices used by the subsystem.
     * The HardwareMap must belong to the currently running opMode
     *
     * @param hardwareMap the HardwareMap of the connected robot
     */
    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        leftPaddle = hardwareMap.get(Servo.class, "leftGrab");
        rightPaddle = hardwareMap.get(Servo.class, "rightGrab");
    }

    /**
     * Checks the status of all devices required by the Subsystem and provides them as a string.
     * The format will be as follows:
     * <p>
     * <code>device: status[\n...]</code>
     *
     * @return the String of concatenated device statuses for this subsystem
     */
    @Override
    public String getHardwareStatus() {
        return "LeftPaddle: " + leftPaddle.getConnectionInfo() +
                "RightPaddle: " + rightPaddle.getConnectionInfo();
    }

    @Override
    public String update(Gamepad gamepad1, Gamepad gamepad2) {
        return null;
    }

    /**
     *
     */
    @Override
    public void stop() {

    }
}
