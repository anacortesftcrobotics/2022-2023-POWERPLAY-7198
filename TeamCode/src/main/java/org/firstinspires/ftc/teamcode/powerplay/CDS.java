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
}
