package org.firstinspires.ftc.teamcode.powerplay.powerplay2;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.powerplay.*;

public class Hand2 implements SubsystemManager{
    Servo grab;
    DcMotor wrist;
    Gamepad gamepad;
    public Hand2(Gamepad g, HardwareMap h){
        gamepad = g;
        initializeHardware(h);
    }
    public void initializeHardware(HardwareMap hardwareMap){
        grab = hardwareMap.get(Servo.class, "grabber");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void setWrist(int pos){
        wrist.setTargetPosition(pos);
        wrist.setPower(0.5);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void grab(boolean open){
        final double OPEN = 0;
        final double CLOSED = 0;
        final double BEACON = 0;
        if (open) {grab.setPosition(OPEN);}
        else {grab.setPosition(CLOSED);}
    }
}
