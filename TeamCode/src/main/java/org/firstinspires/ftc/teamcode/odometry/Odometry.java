package org.firstinspires.ftc.teamcode.odometry;

public interface Odometry{
    public Pose2D update(int leftEncoder, int rightEncoder, int centerEncoder);
    public Pose2D getFieldPose();
    public Pose2D getDeltaPose();
    public void setFieldPose(Pose2D newPose);

}
