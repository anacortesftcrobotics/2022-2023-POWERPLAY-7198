package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class runs the grabber on the 2022-2023 powerplay robot.
 * MPCR = Multi Purpose Cone Righter
 * @author      Lemon
 */

public class Grabber implements SubsystemManager{

    public static HardwareMap hardwareMap;

    public Grabber (HardwareMap hwMap)
    {
        hardwareMap = hwMap;
    }

    private Servo leftGrab, rightGrab;

    public double grabberShift = 0;

    /**
     * The method in all subsystem classes to register the hardware that this class uses.
     * In this case it's the grabber servos for the robot.
     */
    public void initializeHardware()
    {
        leftGrab = hardwareMap.get(Servo.class,"leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");
    }

    /**
     * This method will open and close the grabber. It will also shift the grabber based on a local variable that can be changed with the setShift method.
     * @param open is a boolean that dictates whether the grabber is open or not.
     */
    public void grab(boolean open)
    {
        if(open)
        {
            leftGrab.setPosition(0.45 - grabberShift);
            rightGrab.setPosition(0.55 - grabberShift);
        }
        else
        {
            leftGrab.setPosition(0.6);
            rightGrab.setPosition(0.3);
        }
    }

    /**
     * This method will change the variable that shifts the grabber. Should only ever accept values between -1 and 1.
     * @param shift the amount to change the variable by. Should be between -1 and 1.
     */
    public void setShift(double shift)
    {
        grabberShift = (shift / 10);
    }
}
