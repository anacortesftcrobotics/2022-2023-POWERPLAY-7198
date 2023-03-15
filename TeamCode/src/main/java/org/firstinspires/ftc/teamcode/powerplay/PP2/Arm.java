package org.firstinspires.ftc.teamcode.powerplay.PP2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mentorcode.LimitSwitch;
import org.firstinspires.ftc.teamcode.powerplay.*;
import org.firstinspires.ftc.teamcode.pidclasses.*;

public class Arm implements SubsystemManager {
    DcMotor elbow1;
    DcMotor elbow2;
    LimitSwitch zero;
    PIDFArmController pid1 = new PIDFArmController(0,0,0,0,0,0);
    PIDFArmController pid2 = new PIDFArmController(0,0,0,0,0,0);
    ElapsedTime time = new ElapsedTime();
    @Override
    public void initializeHardware(HardwareMap hardwareMap) {
        elbow1 = hardwareMap.get(DcMotor.class, "turret1");
        elbow1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elbow2 = hardwareMap.get(DcMotor.class, "turret2");
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        time.reset();
        pid1.initiate(elbow1.getCurrentPosition(), time.milliseconds());
        pid2.initiate(elbow2.getCurrentPosition(), time.milliseconds());

        pid1.setOutputClamping(-1, 1);
        pid2.setOutputClamping(-1, 1);

        zero = hardwareMap.get(LimitSwitch.class, "");
    }
    public void update(double target1, double target2){
        elbow1.setPower(pid1.updateArmClamped(Math.toRadians(target1), angle1(), angle2(), time.milliseconds()));
        elbow1.setPower(pid2.updateArmClamped(Math.toRadians(target2), angle2(), 0, time.milliseconds()));
    }
    public double angle1(){
        return Math.toRadians((elbow1.getCurrentPosition()/2.222222222222)+5);
    }
    public double angle2(){
        return Math.toRadians((elbow2.getCurrentPosition()/2.222222222222)+5 - angle1());
    }
}
