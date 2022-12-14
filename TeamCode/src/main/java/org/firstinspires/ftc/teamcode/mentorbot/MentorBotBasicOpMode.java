package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class MentorBotBasicOpMode extends OpMode {

    ArrayList<Subsystem> robot;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        robot = new ArrayList<Subsystem>();
        robot.add(new PowerplayMecanum());
        robot.add(new PowerplayGrabber());
        robot.add(new PoseTracker(17.3853, 17.0234, 16.7964));

        for (Subsystem subsystem : robot) {
            subsystem.registerHardware(hardwareMap);
        }
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        for (Subsystem subsystem : robot) {
            telemetry.addData(subsystem.name, subsystem.update(gamepad1, gamepad2));
        }
    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    @Override
    public void stop() {
        for (Subsystem subsystem : robot) {
            subsystem.stop();
        }
    }
}
