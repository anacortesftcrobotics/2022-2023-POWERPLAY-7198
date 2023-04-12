package org.firstinspires.ftc.teamcode.KaiG;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "The Stuff")
public class TheStuff extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean wasActive = false;
        SemiAuto a = new SemiAuto();
        NeedForSpeed b = new NeedForSpeed();
        Basic c = new Basic();
        a.initializeHardware(hardwareMap);
        b.initializeHardware(hardwareMap);
        c.initializeHardware(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                a.test(gamepad1);
                if (wasActive && !a.stickActive(gamepad1)){
                    c.goPlus(0,0,0,0);
                } else if (a.stickActive(gamepad1)) {
                    c.go(gamepad1);
                }else if (!wasActive && !a.stickActive(gamepad1)){
                    c.goPlus(a.getFL(), a.getFR(), a.getBL(), a.getBR());
                }
                wasActive = a.stickActive(gamepad1);
                telemetry.addLine(a.telem(gamepad1));
                telemetry.update();
            }
        }
    }
}