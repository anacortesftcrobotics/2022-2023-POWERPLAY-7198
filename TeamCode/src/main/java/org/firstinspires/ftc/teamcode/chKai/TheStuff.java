package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "The Stuff")
public class TheStuff extends LinearOpMode {
    @Override
    public void runOpMode() {
        SemiAuto a = new SemiAuto();
        NeedForSpeed b = new NeedForSpeed();
        a.initializeHardware(hardwareMap);
        b.initializeHardware(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                a.test(gamepad1);
                telemetry.addLine(a.test(gamepad1));
                //b.check(gamepad1);
                /*telemetry.addData("coefficient", b.getCoefficient());
                telemetry.addData("speed y", b.getSpeedY(1));
                telemetry.addData("speed x", b.getSpeedX(1));
                telemetry.addData("speed r",b.getSpeedR(1));
                telemetry.addLine(b.check(gamepad1));*/
                telemetry.update();
            }
        }
    }
}