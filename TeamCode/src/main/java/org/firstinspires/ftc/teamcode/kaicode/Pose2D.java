package org.firstinspires.ftc.teamcode.kaicode;

/**
 * @author kaiwallis
 */
public class Pose2D {
    public final double x;
    public final double y;
    public final double headingRad;

    /**
     * Creates a default Pose2D object, the position being at the origin with a heading of 0.
     */
    public Pose2D() {
        this.x = 0.0;
        this.y = 0.0;
        this.headingRad = 0.0;
    }

    /**
     * Creates a Pose2D object based on x, y, and heading values.
     * @param x             x coordinate.
     * @param y             y coordinate.
     * @param headingRad    heading in radians, (+) being counterclockwise.
     */
    public Pose2D(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    /**
     * @param relativePose  a Pose representing the object's movement relative to this Pose, the origin
     *                          representing the object's Pose specified by this Pose. (inc. heading)
     * @return              a Pose representing the object's new Pose including the movement from relativePose.
     */
    public Pose2D move(Pose2D relativePose) {
        double x = relativePose.x * Math.cos(this.headingRad) - relativePose.y * Math.sin(this.headingRad);
        double y = relativePose.x * Math.sin(this.headingRad) + relativePose.y * Math.cos(this.headingRad);
        return new Pose2D(this.x + x, this.y + y, this.headingRad + relativePose.headingRad);
    }

    /**
     * Returns a string representation of the object.
     * @return  a string representation of the object.
     */
    @Override
    public String toString() {
        return this.getClass().getSimpleName() +
                "{x=" + x +
                ", y=" + y +
                ", headingRad= " + headingRad + "}";
    }
}
