package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * This class runs the MPCR on the 2022-2023 powerplay robot.
 * MPCR = Multi Purpose Cone Righter
 * @author      Lemon
 */

public class Gyro implements SubsystemManager{

    /**
     * Empty constructor
     */
    public Gyro()
    {

    }

    //variables
    public double a = 0;
    public double deltaA = 0;
    public double h = 0;


    //hardware
    public BNO055IMU imu;

    Orientation angles;

    Acceleration gravity;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /**
     * The method in all subsystem classes to register the hardware that this class uses.
     * In this case it's the IMU for the robot.
     */
    public void initializeHardware(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
    }

    /**
     * This method updates the global heading variable.
     */
    public void updateHeading()
    {
        a = angles.firstAngle;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        deltaA = angles.firstAngle - a;

        if (deltaA < -180)
            deltaA += 360;
        else if (deltaA > 180)
            deltaA -= 360;

        h += deltaA;
    }

    /**
     * This method returns the global heading variable.
     * @return the angle in degrees.
     */
    public double getHeading()
    {
        return h;
    }
}
