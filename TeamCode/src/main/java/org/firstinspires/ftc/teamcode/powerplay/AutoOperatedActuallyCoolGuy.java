package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This was another AutoOp given to logan to write.
 * This class is non-operational, and should not be run.
 * @author      Lemon and Logan
 */

@Disabled
@Autonomous (name = "AutoOpCoolGuy2", group = "Autonomous")

public class AutoOperatedActuallyCoolGuy extends OpMode {
    int i = 0;
    Heading temp = new Heading();
    Robot powerplay = new Robot();
    double liftHeight = 0;
    int pos;
    double storage;
    boolean once;
    public void init()
    {
        powerplay.initializeHardware(hardwareMap);
    }
    public void start()
    {
        powerplay.robotDefaultState();
        powerplay.chassis.setSpeedCoefficient(0.4);
        powerplay.cds.onOffLED(false);
        powerplay.time.reset();
    }
    public void loop()
    {
        powerplay.gyro.updateHeading();
        powerplay.odometry.updatePosition();
        powerplay.robotTelemetry(telemetry);
        temp = powerplay.odometry.convertToHeading(powerplay.odometry.getX(), powerplay.odometry.getY(), powerplay.gyro.getHeading());
        switch(i) {
            case 0:
                powerplay.led.setLed("black");
                powerplay.grabber.grab(true, false);
                if (powerplay.time.time() > 2)
                    i = 1;
                break;
            case 1:
                powerplay.led.setLed("black");
                liftHeight = 14;
                i = 2;
                break;
            case 2:
                once = false;
                powerplay.led.setLed("black");
                powerplay.resetOdoButOnlyLikeOnce(i);
                if (powerplay.chassis.move(20.5, 0, 0, temp))
                    i = 3;
                break;
            case 3:
                liftHeight = 35;
                if (!once) {
                    storage = powerplay.time.time();
                    powerplay.cds.onOffLED(true);
                    once = true;
                }
                int j = powerplay.cds.getSleeve();
                if (j == 0) {
                    powerplay.led.setLed("black");
                    pos = 2;
                } else if (j == 1) {
                    powerplay.led.setLed("red");
                    pos = 1;
                } else if (j == 2) {
                    powerplay.led.setLed("green");
                    pos = 2;
                } else if (j == 3) {
                    powerplay.led.setLed("blue");
                    pos = 3;
                } else if (j == 4) {
                    powerplay.led.setLed("white");
                    pos = 2;
                }
                if (powerplay.time.time() - storage > 5) {
                    once = false;
                    powerplay.cds.onOffLED(false);
                    i = 4;
                }
                break;
            case 4:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(12, 0, 0, temp))
                    i = 5;
                break;
            case 5:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(0, 0, 30, temp))
                    i = 6;
                break;
            case 6:
                if(!once)
                {
                    storage = powerplay.time.time();
                    once = true;
                    powerplay.chassis.brake();
                }
                if(powerplay.time.time() - storage > 0.5 ){
                    once = false;
                    i = 7;
                }
                break;
            case 7:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(11, 0, 0, temp))
                    i = 8;
                break;
            case 8:
                if(!once)
                {
                    storage = powerplay.time.time();
                    once = true;
                    powerplay.chassis.brake();
                }
                if(powerplay.time.time() - storage > 0.5 ){
                    once = false;
                    i = 9;
                }
                break;
            case 9:
                powerplay.grabber.grab(false,false);
                i = 10;
                break;
            case 10:
                if(!once)
                {
                    storage = powerplay.time.time();
                    once = true;
                    powerplay.chassis.brake();
                }
                if(powerplay.time.time() - storage > 0.5 ){
                    once = false;
                    i = 11;
                }
                break;
            case 11:
                liftHeight = 0;
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(-12, 0, 0, temp))
                    i = 12;
                break;
            case 12:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(0, 0, -30, temp))
                    i = 13;
                break;
            case 13:
                powerplay.resetOdoButOnlyLikeOnce(i);
                powerplay.led.setLed("white");
                if(powerplay.chassis.move(-19,-2,0,temp))
                    i = 14;
                break;
            case 14:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(pos == 1)
                    if(powerplay.chassis.move(3, -24, 0, temp))
                        i = 15;
                if(pos == 2)
                    i = 15;
                if(pos == 3)
                    if(powerplay.chassis.move(3, 24, 0, temp))
                        i = 15;
                break;
            case 15:
                i = 16;
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
    private enum State
    {
        grabbing,
        unGrabbing,
        waiting,
        setLiftUp,
        setLiftDown,
        move20_5


    }
}