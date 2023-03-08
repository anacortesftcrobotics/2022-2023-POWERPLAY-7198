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

    public Gyro()
    {

    }

    //variables
    double a = 0;
    double deltaA = 0;
    double h = 0;
    double a2 = 0;
    double deltaA2 = 0;
    double h2 = 0;

    //hardware
    BNO055IMU imu;
    BNO055IMU imu2;
    Orientation angles;
    Orientation angles2;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /**
     * The method in all subsystem classes to register the hardware that this class uses.
     * In this case it's the IMU for the robot.
     */
    public void initializeHardware(HardwareMap hardwareMap)
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
        imu2.initialize(parameters);
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
    public void updateHeading2()
    {
        a2 = angles2.firstAngle;
        angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        deltaA2 = angles2.firstAngle - a;

        if (deltaA2 < -180)
            deltaA2 += 360;
        else if (deltaA2 > 180)
            deltaA2 -= 360;

        h2 += deltaA2;
    }
    /**
     * This method returns the global heading variable.
     * @return the angle in degrees.
     */
    public double getHeading()
    {
        return h;
    }
    public double getHeading2()
    {
        return h2;
    }
}
