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

    /**
     * Updates odometry.
     */
    public void update() {
        updateEncoders();
        odo.update(left-lastLeft, right-lastRight, back-lastBack);
    }

    /**
     * Updates the values stored representing encoder postions.
     */
    private void updateEncoders() {
        lastLeft = left;
        lastRight = right;
        lastBack = back;

        left = encoderLeft.getCurrentPosition();
        right = encoderRight.getCurrentPosition();
        back = encoderBack.getCurrentPosition();
    }

    /**
     * Returns the robot's current pose on the field.
     * @return  Pose2D object representing the robot's current position & heading.
     */
    public Pose2D getFieldPose() {
        return odo.getFieldPose();
    }

    /**
     * Returns the latest change in the robot's pose on the field.
     * @return  Pose2D object representing the robot's change in position & heading.
     */
    public Pose2D getDeltaPose() {
        return odo.getDeltaPose();
    }

    /**
     * Returns the robot's x position on the field.
     * @return  x position on the field.
     */
    public double getX() {
        return odo.getFieldPose().getX();
    }

    /**
     * Returns the robot's y position on the field.
     * @return  y position on the field.
     */
    public double getY() {
        return odo.getFieldPose().getY();
    }

    /**
     * Returns the robot's heading on the field in radians.
     * @return  heading on the field in radians.
     */
    public double getHeadingRad() {
        return odo.getFieldPose().getHeadingRad();
    }

    /**
     * Returns the robot's heading on the field in degrees.
     * @return  heading on the field in degrees.
     */
    public double getHeadingDeg() {
        return Math.toDegrees(odo.getFieldPose().getHeadingRad());
    }
}
