package org.firstinspires.ftc.teamcode.kaicode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Feeds and stores data from the outdated Odo4 class, do not use :)
 * @author kaiwallis
 */
public class OldOdoController {
    public OldOdoController() {}

    Odo4 odo = new Odo4(39, 2.35, 14.8, 5, 8192); //created in cm
    OldPose2D pose = new OldPose2D();

    public DcMotor encoderLeft, encoderRight, encoderBack;
    public int left, right, back;

    private int lastLeft, lastRight, lastBack;

    /**
     * Initializes the three odometry wheels
     * In this case, it's the four drive motors used by the robot.
     */
    public void initializeHardware (HardwareMap hardwareMap)
    {
        encoderLeft = hardwareMap.get(DcMotor.class, "rightBack");
        encoderRight = hardwareMap.get(DcMotor.class, "leftBack");
        encoderBack = hardwareMap.get(DcMotor.class, "rightFront");
    }

    public void update() {
        updateEncoders();
        pose = odo.getPose(pose, left-lastLeft, right-lastRight, back-lastBack);
    }

    private void updateEncoders() {
        lastLeft = left;
        lastRight = right;
        lastBack = back;

        left = encoderLeft.getCurrentPosition();
        right = encoderRight.getCurrentPosition();
        back = encoderBack.getCurrentPosition();
    }

    public OldPose2D getPose() {
        return pose;
    }

    public double getX() {
        return pose.x;
    }

    public double getY() {
        return pose.y;
    }

    public double getHeading() {
        return Math.toDegrees(pose.headingRad);
    }
}
