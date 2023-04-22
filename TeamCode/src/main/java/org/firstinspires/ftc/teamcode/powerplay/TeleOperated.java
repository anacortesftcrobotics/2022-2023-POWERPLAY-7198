package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is the TeleOp for the 2022-2023 powerplay season.
 * @author      Lemon
 */

@TeleOp (name = "OdoTesting", group = "TeleOp")

public class TeleOperated extends OpMode
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
        powerplay.driveLoop(gamepad1, gamepad2);
        powerplay.robotTelemetry(telemetry);
    }
}