package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "ArtistOp", group = "0TeleOp")

public class ArtistOp extends OpMode
{
    Robot powerplay = new Robot();

    public void init()
    {
        powerplay.initializeHardware(hardwareMap);
    }
    public void start()
    {
        powerplay.robotDefaultState();
    }
    public void loop()
    {
        powerplay.dataLoop();
        powerplay.mpcr.preSetMPCR(0);
        powerplay.chassis.brake();
        powerplay.grabber.grab(false, false);
        powerplay.lift.liftSet(10);
        powerplay.grabber.setShift(gamepad1.touchpad_finger_1_x);
        powerplay.lift.setShift((int) gamepad1.touchpad_finger_1_y * 180);
        powerplay.robotTelemetry(telemetry);
    }
}
