package org.firstinspires.ftc.teamcode.kinematics;

/**
 * This class represents a 2-dimensional arm with 2 joints and a wrist.
 * All three joints are articulated on the x/y plane.
 * @author      Kai Wallis
 * @version     %I%, %G%
 * @
 */
public class Arm2D {

    public Joint2D joint1; //joint on turntable
    public Joint2D joint2; //joint attached to Joint1
    public Joint2D wrist; //joint attached to Joint2

    /**
     * Default constructor with standard joint and segment dimensions.
     */
    public Arm2D() {
        this.joint1 = new Joint2D();
        this.joint2 = new Joint2D();
        this.wrist = new Joint2D();
    }

    /**
     * Constructor with custom joints.
     * @param joint1    joint representing lowest segment.
     * @param joint2    joint representing segment attached to joint1.
     * @param wrist     joint representing segment attached to joint2.
     */
    public Arm2D(Joint2D joint1, Joint2D joint2, Joint2D wrist) {
        this.joint1 = joint1;
        this.joint2 = joint2;
        this.wrist = wrist;
    }

    /**
     * Returns the angle of joint1 relative to x-axis.
     * @return      angle of joint1 relative to x-axis.
     */
    public double getJoint1AngleRad() {
        return joint1.getLocalAngleRad();
    }

    /**
     * Returns the angle of joint2 relative to x-axis.
     * @return      angle of joint2 relative to x-axis.
     */
    public double getJoint2AngleRad() {
        return joint2.getAngleRad(joint1);
    }

    /**
     * Returns the angle of the wrist relative to x-axis.
     * @return      angle of wrist relative to x-axis.
     */
    public double getWristAngleRad() {
        return wrist.getAngleRad(getJoint2AngleRad());
    }

    /**
     * Returns a vector representing the position of joint2.
     * @return      vector representing the position of joint2.
     */
    public Vector2D getJoint2Pos() {
        return joint1.getVector2D();
    }

    /**
     * Returns a vector representing the position of the wrist joint.
     * @return      vector representing the position of the wrist joint.
     */
    public Vector2D getWristPos() {
        return joint2.getVector2D().getRotatedRad(joint2.getLocalAngleRad()).plus(getJoint2Pos());
    }

    /**
     * Returns a vector representing the wrist segment's end position.
     * @return      vector representing the wrist segment's end position.
     */
    public Vector2D getGrabberPos() {
        double wristAngle = joint2.getLocalAngleRad() + joint1.getLocalAngleRad();
        return wrist.getVector2D().getRotatedRad(wristAngle).plus(getJoint2Pos());
    }

    @Override
    public String toString() {
        return "Arm2D{" +
                "joint1=" + joint1 +
                ", joint2=" + joint2 +
                ", wrist=" + wrist +
                '}';
    }
}
