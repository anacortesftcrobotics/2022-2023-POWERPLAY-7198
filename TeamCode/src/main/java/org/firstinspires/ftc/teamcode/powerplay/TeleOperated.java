package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "TeleOp", group = "TeleOp")

public class TeleOperated extends OpMode
{
    boolean once = false;
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
        powerplay.driveLoop(gamepad1, gamepad2);
        powerplay.robotTelemetry(telemetry);
    }
}