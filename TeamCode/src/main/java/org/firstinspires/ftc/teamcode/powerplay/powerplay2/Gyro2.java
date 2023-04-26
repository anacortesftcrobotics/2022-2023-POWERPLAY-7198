package org.firstinspires.ftc.teamcode.powerplay.powerplay2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.powerplay.Gyro;

public class Gyro2 extends Gyro {
    double a2 = 0;
    double deltaA2 = 0;
    double h2 = 0;
    BNO055IMU imu2;
    Orientation angles2;
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
    public double getHeading2()
    {
        return h2;
    }
}
