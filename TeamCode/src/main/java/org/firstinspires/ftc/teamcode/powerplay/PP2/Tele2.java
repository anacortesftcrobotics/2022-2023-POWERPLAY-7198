package org.firstinspires.ftc.teamcode.powerplay.PP2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mentorcode.LimitSwitch;
import org.firstinspires.ftc.teamcode.powerplay.*;
import org.firstinspires.ftc.teamcode.pidclasses.*;
@TeleOp (name = "tele2")
public class Tele2 extends LinearOpMode {
    TurnTable table = new TurnTable();
    Arm arm = new Arm();
    Hand hand = new Hand();
    Chassis chassis = new Chassis();
    Controller control1 = new Controller();
    Controller control2 = new Controller();

    double iElbow1, iElbow2 = 0;
    boolean open;
    double target2 = 0;

    @Override
    public void runOpMode() {
        initializeHardware();
        while (opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            //chassis
            //chassis.xyrMovement(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //turntable
            //        table.setPower(gamepad2.right_stick_x);

            //arm
            target2 += gamepad2.left_stick_x * .02;
            arm.update(0, target2);

            //arm
            /*if(gamepad2.dpad_up){iElbow1 += 0.1;}
            else if (gamepad2.dpad_down) {iElbow1 -= 0.1;}
            if(gamepad2.dpad_right){iElbow2 += 0.1;}
            else if (gamepad2.dpad_left) {iElbow2 -= 0.1;}
            arm.update(iElbow1, iElbow2);*/
            //hand
            //        if (control2.button(11)){open = !open;}
            //        hand.grab(open);

            telemetry.addData("elbow 1 position rads", arm.angle1());
            telemetry.addData("elbow 2 position rads", arm.angle2());
        }
    }

    public void initializeHardware(){
        control1.setGamepad(gamepad1);
        control2.setGamepad(gamepad2);
        table.initializeHardware(hardwareMap);
        arm.initializeHardware(hardwareMap);
        hand.initializeHardware(hardwareMap);
        chassis.initializeHardware(hardwareMap);
    }
}
