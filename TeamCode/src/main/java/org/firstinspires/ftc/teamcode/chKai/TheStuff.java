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
        Basic c = new Basic();
        a.initializeHardware(hardwareMap);
        b.initializeHardware(hardwareMap);
        c.initializeHardware(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (! a.stickActive(gamepad1)) {
                    a.test(gamepad1);
                }else {
                    c.go(gamepad1);
                }
                telemetry.addLine(a.telem(gamepad1));
                telemetry.update();
            }
        }
    }
}