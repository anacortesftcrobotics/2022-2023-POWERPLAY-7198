package org.firstinspires.ftc.teamcode.powerplay.odometry;

/**
 * @author kaiwallis
 */
public class Pose2D {
    private final double x;
    private final double y;
    private final double headingRad;

    public Pose2D() {
        this.x = 0.0;
        this.y = 0.0;
        this.headingRad = 0.0;
    }

    public Pose2D(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    public Pose2D plus(Pose2D otherPose) {
        return new Pose2D(x + otherPose.x, y + otherPose.y, headingRad + otherPose.headingRad);
    }

    public Pose2D minus(Pose2D otherPose) {
        return new Pose2D(x - otherPose.x, y - otherPose.y, headingRad - otherPose.headingRad);
    }

    public Pose2D times(double scalar) {
        return new Pose2D(x * scalar, y * scalar, headingRad * scalar);
    }

    public Pose2D divide(double scalar) {
        return new Pose2D(x / scalar, y / scalar, headingRad / scalar);
    }

    /**
     * @param relativePose new pose state relative to current pose.
     * @return
     */
    public Pose2D move(Pose2D relativePose) {
        double x = relativePose.x * Math.cos(this.headingRad) - relativePose.y * Math.sin(this.headingRad);
        double y = relativePose.x * Math.sin(this.headingRad) + relativePose.y * Math.cos(this.headingRad);
        return new Pose2D(this.x + x, this.y + y, this.headingRad + relativePose.headingRad);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingRad() {
        return headingRad;
    }

    @Override
    public String toString() {
        return "x=" + x +
                ", y=" + y +
                ", headingRad= " + headingRad;
    }
}
