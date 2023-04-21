package org.firstinspires.ftc.teamcode.powerplay.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.powerplay.Logger;

/**
 * @author kaiwallis
 */
public class OdoController {
    public OdoController() {}

    Odo4 odo = new Odo4(40, 2.5, 15, 5, 8192); //created in cm
    Pose2D pose = new Pose2D();

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
        pose = odo.getFieldPose(pose, lastLeft-left, lastRight-right, lastBack-back);
    }

    private void updateEncoders() {
        lastLeft = left;
        lastRight = right;
        lastBack = back;

        left = encoderLeft.getCurrentPosition();
        right = encoderRight.getCurrentPosition();
        back = encoderBack.getCurrentPosition();
    }

    public Pose2D getPose() {
        return pose;
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public double getHeading() {
        return Math.toDegrees(pose.getHeadingRad());
    }
}
