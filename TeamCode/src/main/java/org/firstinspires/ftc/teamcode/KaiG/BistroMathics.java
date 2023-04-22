package org.firstinspires.ftc.teamcode.KaiG;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class BistroMathics{
    private static double angle(BNO055IMU imu){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        angle += 45;
        if(angle < 0){angle += 360;}
        if(angle > 360){angle -= 360;}
        return angle;
    }
    public static double FRBLy(BNO055IMU imu){
        double a = angle(imu);
        double x = 0;
        if (a > 0 && a < 90){x = a/90;}
        else if(a > 90 && a < 180){x = (180 - a)/90;}
        else if(a > 180 && a < 270){x = (a - 180)/-90;}
        else if(a > 270 && a < 360){x = (360 - a)/-90;}
        return x;
    }
    public static double FLBRy(BNO055IMU imu){
        double a = angle(imu);
        double x = 0;
        if (a > 0 && a < 90){x = (90 - a)/90;}
        else if(a > 90 && a < 180){x = (a - 90)/-90;}
        else if(a > 180 && a < 270){x = (270 - a)/-90;}
        else if(a > 270 && a < 360){x = (a - 270)/90;}
        return x;
    }
    public static double FRBLx(BNO055IMU imu){
        return -FLBRy(imu);
    }
    public static double FLBRx(BNO055IMU imu){
        return FRBLy(imu);
    }
}