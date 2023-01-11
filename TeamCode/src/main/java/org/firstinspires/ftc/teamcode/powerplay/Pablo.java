package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/**
 * This class runs the distance sensors mounted on the 2022-2023 powerplay robot grabber.
 * @author      Lemon
 */
public class Pablo implements SubsystemManager{

    /**
     * Empty constructor
     */
    public Pablo() {

    }

    DistanceSensor leftDistance, rightDistance;

    /**
     * This method registers the distance sensors. Needs to be called in order to use the pablo class.
     */
    public void initializeHardware(HardwareMap hardwareMap) {
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
    }

    /**
     * This function returns the distance that the left distance sensor reads, in CM.
     * @return a double value of the distance, in CM.
     */
    public double getLeftDistance() {
        return leftDistance.getDistance(DistanceUnit.CM);
    }

    /**
     * This function returns the distance that the right distance sensor reads, in CM.
     * @return a double value of the distance, in CM.
     */
    public double getRightDistance() {
        return rightDistance.getDistance(DistanceUnit.CM);
    }

    /**
     * This function reads the distance sensors and spits out a direction to turn the robot in, to center the grabber on the junction
     * @return a double value to be added to the rx value in movement calculations
     */
    public double poleCenter() {
        double leftDist = leftDistance.getDistance(DistanceUnit.CM);
        double rightDist = rightDistance.getDistance(DistanceUnit.CM);
        if (leftDist < rightDist && leftDist < 18)
            return 0.05;
        if (rightDist < leftDist && rightDist < 18)
            return -0.05;
        return 0;
    }
}
