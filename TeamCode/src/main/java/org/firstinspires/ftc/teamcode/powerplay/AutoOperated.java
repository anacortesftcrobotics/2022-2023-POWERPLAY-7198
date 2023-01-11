package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This was the first AutoOp made. It functioned, but became broken after changes were made elsewhere in the code.
 * This class is non-operational, and should not be run.
 * @author      Lemon
 */

@Disabled
@Autonomous (name = "AutoOp", group = "Autonomous")

public class AutoOperated extends OpMode {
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
        switch(i) {
            case 0:
                powerplay.led.setLed("black");
                powerplay.grabber.grab(true, false);
                if(powerplay.time.time() > 2)
                    i++;
                break;
            case 1:
                powerplay.led.setLed("black");
                liftHeight = 14;
                i++;
                break;
            case 2:
                once = false;
                powerplay.led.setLed("black");
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(20.5, 0, 0, temp))
                    i++;
                break;
            case 3:
                if(once)
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
                if(powerplay.time.time() - storage > 5) {
                    powerplay.cds.onOffLED(false);
                    i++;
                }
                break;
            case 4:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(20, 0, 0, temp))
                    i++;
                break;
            case 5:
            case 6:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(powerplay.chassis.move(-8.5, 1, 0, temp))
                    i++;
                break;
            case 7:
                powerplay.resetOdoButOnlyLikeOnce(i);
                if(pos == 1)
                    if(powerplay.chassis.move(4, -25, 0, temp))
                        i++;
                if(pos == 2)
                    i++;
                if(pos == 3)
                    if(powerplay.chassis.move(1, 25, 0, temp))
                        i++;
                break;
            case 8:
                liftHeight = 0;
                if((powerplay.lift.leftLift.getCurrentPosition() + powerplay.lift.liftChange + powerplay.lift.liftManual + 10) < 20)
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