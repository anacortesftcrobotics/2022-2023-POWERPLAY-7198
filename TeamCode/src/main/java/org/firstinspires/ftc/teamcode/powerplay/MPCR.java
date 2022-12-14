package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class runs the MPCR on the 2022-2023 powerplay robot.
 * MPCR = Multi Purpose Cone Righter
 * @author      Lemon
 */

public class MPCR implements SubsystemManager{

    public MPCR ()
    {

    }

    private Servo leftMPCR, rightMPCR;

    /**
     * This method registers the hardware used in this class.
     */
    public void initializeHardware(HardwareMap hardwareMap)
    {
        leftMPCR = hardwareMap.get(Servo.class, "leftMPCR");
        rightMPCR = hardwareMap.get(Servo.class, "rightMPCR");
    }

    /**
     * This method sets the cone righter to preset positions.
     * 0   1       back
     * 1   0.55    out of the way
     * 2   0.45    in the way
     * 3   0.16    cone righting
     * @param position is
     */
    public void preSetMPCR(int position)
    {
        switch (position) {
            case 0:
                rightMPCR.setPosition(0.92);
                leftMPCR.setPosition(0.08);
                break;
            case 1:
                rightMPCR.setPosition(0.55);
                leftMPCR.setPosition(0.45);
                break;
            case 2:
                rightMPCR.setPosition(0.45);
                leftMPCR.setPosition(0.55);
                break;
            case 3:
                rightMPCR.setPosition(0.2);
                leftMPCR.setPosition(0.8);
                break;
            default:
                rightMPCR.setPosition(0.45);
                leftMPCR.setPosition(1 - 0.45);
        }
    }

    /**
     * This method sets the cone righter to manually input positions.
     * @param position is a double. Should be between 0 and 1. 1 is all the way back into the robot, 0 is all the way out front.
     */
    public void setMPCR(double position)
    {
        rightMPCR.setPosition(position);
        leftMPCR.setPosition(1 - position);
    }
}
