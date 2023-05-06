package org.firstinspires.ftc.teamcode.powerplay.powerplay2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.powerplay.*;

public class Auto2 extends OpMode {
    Heading head = new Heading();
    Chassis2 chas = new Chassis2(gamepad1, hardwareMap);
    Odo2 odo = new Odo2();
    Gyro2 imu = new Gyro2();
    CDS cds = new CDS();
    int color = 0;
    int i = 0;

    @Override
    public void init() {
        odo.initializeHardware(hardwareMap);
        imu.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
        cds.onOffLED(true);
    }


    @Override
    public void loop(){
        update();
        switch (i){
            case 0: //move forward one tile
                //set arm to correct position
                i = (chas.toPos(0,24, head)) ? 0:1;
                break;
            case 1: //move forward slowly until the cds detects something, then save the color value in color
                if (cds.identify() == 0){chas.xyrMovement(0, 0.25);}
                else {
                    color = cds.identify();
                    i = 2;
                }
                break;
            case 2: //move forward one tiles to push signal cone out of the way
                i = (chas.toPos(0,48, head)) ? 2:3;
                break;
            case 3: //move back one tile
                i = (chas.toPos(0,24, head)) ? 3:4;
            case 4: //move right or left or not at all, depending on color
                switch (color){
                    case 1:
                        i = (chas.toPos(-24,24, head)) ? 4:5;
                    case 2:
                        i = 5;
                    case 3:
                        i = (chas.toPos(24,24, head)) ? 4:5;
                }
                break;
            case 5:

        }
    }

    public void update(){
        head.setHeading(odo.getX(),odo.getY(), imu.getHeading());
    }
}
