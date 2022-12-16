package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous (name = "AutoOpEX", group = "Autonomous")

public class AutoOperated3 extends OpMode {
    int i = 0;
    Heading temp = new Heading();
    Robot powerplay = new Robot();
    double liftHeight = 0;
    int pos;
    double storage;
    boolean once = false;
    public void init()
    {
        powerplay.initializeHardware(hardwareMap);
    }
    public void start()
    {
        powerplay.robotDefaultState();
        powerplay.chassis.setSpeedCoefficient(0.3);
        powerplay.cds.onOffLED(false);
        powerplay.time.reset();
    }
    public void loop()
    {
        powerplay.gyro.updateHeading();
        powerplay.odometry.updatePosition();
        powerplay.robotTelemetry(telemetry);
        temp = powerplay.odometry.convertToHeading(powerplay.odometry.getX(), powerplay.odometry.getY(), powerplay.gyro.getHeading());
        powerplay.lift.liftSet(liftHeight);
        switch(i) {
            case 0:
                powerplay.grabber.grab(true,false);
                if(powerplay.time.time() > 0.5)
                    i++;
                break;
            case 1:
                powerplay.resetOdoButOnlyLikeOnce(i);
                liftHeight = 15;
                if(powerplay.chassis.move(19.25,0,0,temp))
                    i++;
                break;
            case 2:
            case 5:
            case 7:
                if(!once)
                {
                    storage = powerplay.time.time();
                    once = !once;
                    powerplay.chassis.brake();
                }
                if(powerplay.time.time() - storage > 0.75 ){
                    once = false;
                    i++;
                }
                break;
            case 3:
                if(!once)
                {
                    storage = powerplay.time.time();
                    powerplay.cds.onOffLED(true);
                    once = !once;
                }
                int j = powerplay.cds.getSleeve();
                if(j == 0) {
                    powerplay.led.setLed("black");
                    pos = 2;
                }
                else if(j == 1){
                    powerplay.led.setLed("red");
                    pos = 1;
                }
                else if (j == 2){
                    powerplay.led.setLed("green");
                    pos = 2;
                }
                else if (j == 3){
                    powerplay.led.setLed("blue");
                    pos = 3;
                }
                else if (j == 4){
                    powerplay.led.setLed("white");
                    pos = 2;
                }
                if(powerplay.time.time() - storage > 1) {
                    powerplay.cds.onOffLED(false);
                    once = false;
                    i++;
                }
                break;
            case 4:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(22,0,0,temp))
                    i++;
                break;
            case 6:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(-19,2,0,temp))
                    i++;
                break;
            case 8:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(pos == 1)
                    if(powerplay.chassis.move(2, -23, 0, temp))
                        i++;
                if(pos == 2)
                    i++;
                if(pos == 3)
                    if(powerplay.chassis.move(2, 23, 0, temp))
                        i++;
                break;
            case 9:
                powerplay.resetOdoButOnlyLikeOnce(i);
                liftHeight = 0;
                if(powerplay.chassis.move(0,0,40,temp))
                    i++;
                break;
            case 10:
                i++;
                break;
            case 11:
                i++;
                break;
            case 12:
                i++;
                break;
            case 13:
                i++;
                break;
            case 14:
                i++;
                break;
            case 15:
                i++;
                break;
            case 16:
                i++;
                break;
            case 17:
                i++;
                break;
            default:
                powerplay.led.setLed("black");
                powerplay.resetOdoButOnlyLikeOnce(i);
                powerplay.chassis.brake();
                terminateOpModeNow();
                break;
        }
        powerplay.lift.liftSet(liftHeight);
    }
}