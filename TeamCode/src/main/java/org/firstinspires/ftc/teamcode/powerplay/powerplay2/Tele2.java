package org.firstinspires.ftc.teamcode.powerplay.powerplay2;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.powerplay.*;

@TeleOp (name = "tele2")
public class Tele2 extends LinearOpMode {

    // TurnTable table = new TurnTable();
    Arm2 arm = new Arm2(gamepad2, hardwareMap);
    //Hand2 hand = new Hand2(gamepad2, hardwareMap);
    //Chassis chassis = new Chassis();
    Controller control1 = new Controller();
    Controller control2 = new Controller();

    double iElbow1, iElbow2 = 0;
    boolean open;
    double target1, target2 = 0;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();
        while (opModeIsActive()) {

            target1 += gamepad2.left_stick_x * .2;
            arm.update(0, target1);


            telemetry.addData("elbow 1 position rads", arm.angle1());
            telemetry.addData("elbow 2 position rads", arm.angle2());
            telemetry.addData("1 pos deg", arm.degrees1());
            telemetry.addData("2 pos deg", arm.degrees2());
            telemetry.addData("target2", target2);
            telemetry.addData("power2", arm.getPwr2());
            telemetry.addData("power1", arm.getPwr1());
            telemetry.update();
        }
    }

    public void initializeHardware(){
        control1.setGamepad(gamepad1);
        control2.setGamepad(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //table.initializeHardware(hardwareMap);
    }
}
