package org.firstinspires.ftc.teamcode.powerplay.PP2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mentorcode.LimitSwitch;
import org.firstinspires.ftc.teamcode.powerplay.*;
import org.firstinspires.ftc.teamcode.pidclasses.*;

public class Arm implements SubsystemManager {
    private final double DIFFERENCE_BETWEEN_LIMIT_SWITCH_AND_ACTUAL_ZERO = 5;
    private  final double CHANGING_ENCODER_TICKS_INTO_DEGREES = 2+2/9;
    DcMotor elbow1, elbow2;
    LimitSwitch zero1, zero2;
    PIDFArmController pid1 = new PIDFArmController(1,0,0,0,0,0,0);
    PIDFArmController pid2 = new PIDFArmController(0,0,0,0,0,0,0);
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
        pid1.launch(elbow1.getCurrentPosition(), time.milliseconds());
        pid2.launch(elbow2.getCurrentPosition(), time.milliseconds());

        pid1.setOutputClamping(-1, 1);
        pid2.setOutputClamping(-1, 1);

        zero1 = hardwareMap.get(LimitSwitch.class, "");
        zero2 = hardwareMap.get(LimitSwitch.class, "");
    }
    public void update(double target1, double target2){
        double pwr1, pwr2 = 0;
        pwr1 = pid1.updateArmClamped(Math.toRadians(target1), angle1(), angle2(), time.milliseconds());
        pwr2 = pid2.updateArmClamped(Math.toRadians(target2), angle2(), 0, time.milliseconds());
        if (zero1.isPressed()){elbow1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); pwr1 = 0;}
        if (zero2.isPressed()){elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); pwr1 = 0;}
        elbow1.setPower(pwr1);
        elbow2.setPower(pwr2);
    }
    public double angle1(){
        return Math.toRadians((elbow1.getCurrentPosition()/CHANGING_ENCODER_TICKS_INTO_DEGREES) + DIFFERENCE_BETWEEN_LIMIT_SWITCH_AND_ACTUAL_ZERO);
    }
    public double angle2(){
        return Math.toRadians((elbow2.getCurrentPosition()/CHANGING_ENCODER_TICKS_INTO_DEGREES) + DIFFERENCE_BETWEEN_LIMIT_SWITCH_AND_ACTUAL_ZERO - angle1());
    }
}
