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

    /**gets the red value from the CDS
    *@return int value of red
    *@author Kai G
     */
    public int getRed(){
        return color.red();
    }
     /**gets the blue value from the CDS
    *@return int value of blue
    *@author Kai G
     */
    public int getBlue(){
        return color.blue();
    }
     /**gets the green value from the CDS
    *@return int value of green
    *@author Kai G
     */
    public int getGreen(){
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
     * @return an int value based on which side. 1 = red, 2 = green, 3 = blue, 4 = white, 0 = nothing
     */
    public int getSleeve()
    {
        if(color.getDistance(DistanceUnit.INCH) > 1)
            return 0;
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
            if(color.red() < 1000)
                return 3;
            return 4;
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
    /**
    *add telemetry with the individual rgb color values, as well as the distance in cm. 
    *@return a string with each color value and distance, labeled
    *to use: telemetry.addLine([CDS object].colorTelemetry);
    *@author Kai G
    */
    public String colorTelemetry(){
        int red = getRed();
        int green = getGreen();
        int blue = getGreen();
        int dist = color.getDistance(DistanceUnit.CM);
        return ("red - "+ red+"\ngreen - "+green+"\nblue - "+blue+"\ndistance (cm)"+dist);
    }
    /**
    *Check which way the signal sleeve is facing
    *@return 0 if nothing, 1 if red, 2 if green, 3 if blue, and 4 if white.
    *sensor light should be on
    *@author Kai G
    */
    public int identify () { 
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
