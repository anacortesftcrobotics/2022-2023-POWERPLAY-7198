package org.firstinspires.ftc.teamcode.powerplay.PP2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.powerplay.Chassis;
import org.firstinspires.ftc.teamcode.powerplay.Gyro;
import org.firstinspires.ftc.teamcode.powerplay.Heading;

public class Chassis2 extends Chassis {
    Gyro imu;

    public void initializeHardware(HardwareMap hardwareMap)
    {
        super.initializeHardware(hardwareMap);
        imu.initializeHardware(hardwareMap);
    }
    public void xyrMovement(double x, double y){
            double rx = imu.getHeading();
            super.xyrMovement(x,y,rx);
    }


}
