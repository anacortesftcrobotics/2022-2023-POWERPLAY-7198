package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class should be used to run the CDS on the 2022-2023 powerplay robot.
 * CDS = Color Distance Sensor
 * @author      Lemon
 */

public class CDS implements SubsystemManager {

    public CDS ()
    {

    }

    private ColorRangeSensor color;

    /**
     * The method in all subsystem classes to register the hardware that this class uses.
     * In this case it's the color sensor for the robot.
     */
    public void initializeHardware(HardwareMap hardwareMap) {
        color = hardwareMap.get(ColorRangeSensor.class,"color");
    }

    /**
     * This method returns the distance in CM.
     * @return the distance in CM.
     */
    public double getDistance ()
    {
        return color.getDistance(DistanceUnit.INCH);
    }
    public double getRed(){
        return color.red();
    }
    public double getBlue(){
        return color.blue();
    }
    public double getGreen(){
        return color.green();
    }

    /**
     * returns the argb value of the color sensor.
     * @return the argb value.
     */
    public double getColor ()
    {
        return color.argb();
    }

    /**
     * A method to check which way the signal sleeve is facing.
     * @return an int value based on which side. 1 = red, 2 = green, 3 = blue, 0 = error.
     */
    public int getSleeve()
    {
        if(color.red() > color.green() && color.red() > color.blue())
        {
            //red
            return 1;
        }
        else if(color.green() > color.red() && color.green() > color.blue())
        {
            //green
            return 2;
        }
        else if(color.blue() > color.green() && color.blue() > color.red())
        {
            //blue
            return 3;
        }
        return 0;
    }

    /**
     * Turns the color sensor led on and off.
     * @param onOff boolean for on and off. True = on, false = off.
     */
    public void onOffLED (boolean onOff)
    {
        color.enableLed(onOff);
    }
    public String colorTelemetry(){
        double red = getRed();
        double green = getGreen();
        double blue = getGreen();
        double dist = color.getDistance(DistanceUnit.CM);
        return ("red - "+ red+"\ngreen - "+green+"\nblue - "+blue);
    }
    public int identify () { //x=s 1 if red, 2 if green, 3 if blue, 4 if white and 0 if nothing.
        int x = 0;
        if (getDistance() > 1) {
            x = 0;
        } else {
            if (getRed() > getBlue() && getRed() > getGreen()) {
                x = 1;
            } else if (getGreen() > getBlue() && getGreen() > getRed()) {
                x = 2;
            } else if (getBlue() > getRed() && getBlue() > getGreen()) {
                if (getRed() < 1000)//(cds.getBlue()- cds.getGreen()>(cds.getRed())+ cds.getGreen()+ cds.getBlue()/5)
                {
                    x = 3;
                } else {
                    x = 4;
                }
            }
        }
        return x;
    }
}
