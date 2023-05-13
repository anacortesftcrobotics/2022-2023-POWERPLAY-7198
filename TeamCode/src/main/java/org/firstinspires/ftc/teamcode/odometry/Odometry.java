package org.firstinspires.ftc.teamcode.odometry;

public interface Odometry{
    /**
     * Updates the object's position on the field based on the change in encoder positions then returns the updated
     * field position.
     * @param leftEncoder   change in tick position of the left encoder, (+) when moving forward.
     * @param rightEncoder  change in tick position of the right encoder, (+) when moving forward.
     * @param centerEncoder change in tick position of the center encoder, (+) when strafing right.
     * @return              the object's current pose on the field.
     */
    public Pose2D update(int leftEncoder, int rightEncoder, int centerEncoder);

    /**
     * Returns the object's current pose on the field.
     * @return  Pose2D object representing the object's current position & heading.
     */
    public Pose2D getFieldPose();

    /**
     * Returns the latest change in the object's pose on the field
     * @return  Pose2D object representing the object's change in position & heading.
     */
    public Pose2D getDeltaPose();

    /**
     * Resets the object's pose on the field to a specified pose.
     * @param newPose   Pose2D object representing the object's new position & heading.
     */
    public void setFieldPose(Pose2D newPose);

    /**
     * Resets the object's pose on the field, with the x, y, & heading being equal to 0.0.
     */
    public void setFieldPose();
}
