package org.firstinspires.ftc.teamcode.powerplay.powerplay2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.powerplay.Chassis;
import org.firstinspires.ftc.teamcode.powerplay.Gyro;
import org.firstinspires.ftc.teamcode.powerplay.Heading;

public class Chassis2 extends Chassis {
    Gyro imu;
    Gamepad gamepad;
    public Chassis2(Gamepad g, HardwareMap h){
        gamepad = g;
        initializeHardware(h);
    }

    public void initializeHardware(HardwareMap hardwareMap)
    {
        super.initializeHardware(hardwareMap);
        imu.initializeHardware(hardwareMap);
    }
    public void xyrMovement(double x, double y){
            double rx = -(imu.getHeading()/5);
            super.xyrMovement(x,y,rx);
    }
    public boolean toPos(double xGoal, double yGoal, Heading heading){
        return super.toPos(xGoal, yGoal, imu.getHeading(), heading);
    }
    public void update(){
        xyrMovement(gamepad.left_stick_x, gamepad.left_stick_y);
    }


}
