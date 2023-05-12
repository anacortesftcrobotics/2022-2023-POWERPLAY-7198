package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoController {
    public AutoController() {}

    Odometry odo = new Odo5(39, 2.35, 14.8, 5, 8192); //created in cm

    private DcMotor encoderLeft, encoderRight, encoderBack;

    private int left, right, back;
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
        odo.update(left-lastLeft, right-lastRight, back-lastBack);
    }

    private void updateEncoders() {
        lastLeft = left;
        lastRight = right;
        lastBack = back;

        left = encoderLeft.getCurrentPosition();
        right = encoderRight.getCurrentPosition();
        back = encoderBack.getCurrentPosition();
    }

    public Pose2D getFieldPose() {
        return odo.getFieldPose();
    }

    public Pose2D getDeltaPose() {
        return odo.getDeltaPose();
    }

    public double getX() {
        return odo.getFieldPose().getX();
    }

    public double getY() {
        return odo.getFieldPose().getY();
    }

    public double getHeadingRad() {
        return odo.getFieldPose().getHeadingRad();
    }

    public double getHeadingDeg() {
        return Math.toDegrees(odo.getFieldPose().getHeadingRad());
    }
}
