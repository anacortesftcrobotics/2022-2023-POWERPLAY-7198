package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.powerplay.*;

public class BistroMath{
    private double angle(BNO055IMU imu){
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