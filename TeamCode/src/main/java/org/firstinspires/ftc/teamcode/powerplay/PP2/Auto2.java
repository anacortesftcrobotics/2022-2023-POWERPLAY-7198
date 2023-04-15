package org.firstinspires.ftc.teamcode.powerplay.PP2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.powerplay.*;

public class Auto2 extends OpMode {
    Heading head = new Heading();
    Chassis2 chas = new Chassis2();
    Odo2 odo = new Odo2();
    Gyro2 imu = new Gyro2();
    CDS cds = new CDS();
    int i = 0;

    @Override
    public void init() {
        chas.initializeHardware(hardwareMap);
        odo.initializeHardware(hardwareMap);
        imu.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
    }


    @Override
    public void loop(){
        switch (i){
            case 0:
                //set arm to correct position
                i = (chas.toPos(0,24, 0, head)) ? 0:1;
                break;
            case 1:
                switch (cds.identify()){
                    case 0:
                        chas.xyrMovement(0,0.25);
                        break;
                    case 1:
                        i = (chas.toPos(-24,0, 0, head)) ? 1:2;
                        break;
                    case 2:

                }
        }
    }

    public void update(){
        head.setHeading(odo.getX(),odo.getY(), imu.getHeading());
    }
}
