package org.firstinspires.ftc.teamcode.powerplay.powerplay2;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.powerplay.*;

public class TurnTable implements SubsystemManager {
    DcMotorSimple table; //init this way because it is run by a SparkMini controller
    Gyro imu = new Gyro();
    Pablo pablo = new Pablo();

    @Override
    public void initializeHardware(HardwareMap hardwareMap) {
        table = hardwareMap.get(DcMotorSimple.class, "turnTable"); //I'm not sure this is the right name
        imu.initializeHardware(hardwareMap);
        pablo.initializeHardware(hardwareMap);
    }
    public void setPower(double pwr){
        table.setPower(pwr);
    }
    public void rtp(int deg){

    }
    public void center(){

    }
}
