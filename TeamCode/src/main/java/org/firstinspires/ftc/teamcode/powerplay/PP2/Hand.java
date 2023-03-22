package org.firstinspires.ftc.teamcode.powerplay.PP2;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mentorcode.LimitSwitch;
import org.firstinspires.ftc.teamcode.powerplay.*;
import org.firstinspires.ftc.teamcode.pidclasses.*;

public class Hand implements SubsystemManager{
    Servo grab;
    DcMotor wrist;
    public void initializeHardware(HardwareMap hardwareMap){
        //grab = hardwareMap.get(Servo.class, "grabber");
        //wrist = hardwareMap.get(DcMotor.class, "wrist");
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
