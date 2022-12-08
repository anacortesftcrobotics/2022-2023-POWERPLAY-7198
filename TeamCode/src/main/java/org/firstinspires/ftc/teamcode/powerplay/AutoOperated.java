package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "AutoOp", group = "Autonomous")

public class AutoOperated extends OpMode {
    boolean once = false;
    int i = 0;
    private DcMotor leftBack, leftFront, rightBack, rightFront, leftLift, rightLift;
    private DcMotor encoderRight, encoderLeft, encoderBack;
    private Servo leftGrab, rightGrab, leftMPCR, rightMPCR;
    private DistanceSensor leftDistance, rightDistance;
    private ColorRangeSensor color;
    private TouchSensor zero;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Robot powerplay = new Robot();
    public void init()
    {
        powerplay.initializeHardware(hardwareMap);
    }
    public void start()
    {
        powerplay.robotDefaultState();
    }
    public void loop()
    {
        powerplay.gyro.updateHeading();
        powerplay.odometry.updatePosition();
        switch(i) {
            case 0:
                if (powerplay.chassis.move(20, powerplay.odometry.convertToHeading(powerplay.odometry.getX(), powerplay.odometry.getY(), powerplay.gyro.getHeading())))
                    i++;
                break;
            case 1:
                if (powerplay.chassis.strafe(20, powerplay.odometry.convertToHeading(powerplay.odometry.getX(), powerplay.odometry.getY(), powerplay.gyro.getHeading())))
                    i++;
                break;
            case 2:
                if(powerplay.chassis.turn(90, powerplay.odometry.convertToHeading(powerplay.odometry.getX(), powerplay.odometry.getY(), powerplay.gyro.getHeading())))
                    i++;
                break;
            case 3:
                powerplay.chassis.brake();
                terminateOpModeNow();
                break;
            default:
                powerplay.chassis.brake();
                break;
        }
        powerplay.robotTelemetry(telemetry);
    }
}