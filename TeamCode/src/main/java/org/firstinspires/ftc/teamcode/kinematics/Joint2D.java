package org.firstinspires.ftc.teamcode.kinematics;

/**
 * This class represents an arm joint and attached segment for a 2-dimensional arm.
 * @author      Kai Wallis
 * @version     %I%, %G%
 * @
 */
public class Joint2D {
    private double lowerLimitRad = Double.NEGATIVE_INFINITY;
    private double upperLimitRad = Double.POSITIVE_INFINITY;
    private final double length;
    private double localAngleRad = 0.0;

    /**
     * Default constructor, length=0.0.
     */
    public Joint2D() {
        this.length = 0.0;
    }

    /**
     * Constructor with attached segment length & starting angle.
     * Constructs with no upper or lower limits.
     * @param length        length of attached segment.
     * @param localAngleRad     angle of segment relative to attached joint when
     *                          0.0 deg = inside segment it's attached to.
     */
    public Joint2D(double length, double localAngleRad) {
        this.length = length;
        this.localAngleRad = localAngleRad;
    }

    /**
     * Creates a joint with a custom length, starting angle, and limit values.
     * @param length        length of attached segment.
     * @param localAngleRad     segment's angle relative to attached joint when
     *                          0.0 deg = inside segment it's attached to.
     * @param lowerLimitRad     lower angle measurement to not exceed.
     * @param upperLimitRad     upped angle measurement to not exceed.
     */
    public Joint2D(double length, double localAngleRad, double lowerLimitRad, double upperLimitRad) {
        this.length = length;
        this.localAngleRad = localAngleRad;
        this.lowerLimitRad = lowerLimitRad;
        this.upperLimitRad = upperLimitRad;
    }

    /**
     * Returns boolean representing if angle is within limits.
     * @return      returns true if within limits, false if not.
     */
    public boolean inLimits() {
        return !(this.localAngleRad < lowerLimitRad) && !(this.localAngleRad > upperLimitRad);
    }

    /**
     * Returns boolean representing if a given angle is within limits.
     * @param localAngleRad     angle to check against limits
     * @return      returns true if within limits, false if not.
     */
    public boolean inLimits(double localAngleRad) {
        return !(localAngleRad < lowerLimitRad) && !(localAngleRad > upperLimitRad);
    }

    /**
     * updates the Joint2D's angle
     * @param localAngleRad     angle of Joint2D relative to attached joint when
     *                          0.0 deg = inside segment it's attached to.
     */
    public void setLocalAngleRad(double localAngleRad) {
        this.localAngleRad = localAngleRad;
    }

    /**
     * Sets limit values
     * @param lowerLimitRad     lower angle measurement to not exceed.
     * @param upperLimitRad     upped angle measurement to not exceed.
     */
    public void setLimitsRad(double lowerLimitRad, double upperLimitRad) {
        this.lowerLimitRad = lowerLimitRad;
        this.upperLimitRad = upperLimitRad;
    }

    /**
     * Returns the segment's angle relative to attached joint.
     * 0.0 deg = inside segment it's attached to.
     * @return      angle relative to attached joint in radians.
     */
    public double getLocalAngleRad() {
        return localAngleRad;
    }

    /**
     * Returns the segment's angle relative to the x-axis.
     * @param baseAngleRad  current angle to segment Joint2D is attached to.
     * @return      angle relative to the x-axis in radians.
     */
    public double getAngleRad(double baseAngleRad) {
        return localAngleRad + baseAngleRad;
    }

    /**
     * Returns the segment's angle relative to lower segment's mount.
     * @param baseJoint2D  segment Joint2D is attached to.
     * @return      angle relative to the plane baseJoint2D is attached to in radians.
     */
    public double getAngleRad(Joint2D baseJoint2D) {
        return localAngleRad + baseJoint2D.getLocalAngleRad();
    }

    /**
     * Returns the segment's length.
     * @return  segment's length.
     */
    public double getLength() {
        return length;
    }

    /**
     * Returns the x coordinate of the segment's end.
     * @return  x coordinate of the segment's end.
     */
    public double getX() {
        return length * Math.cos(localAngleRad);
    }

    /**
     * Returns the y coordinate of the segment's end.
     * @return  y coordinate of the segment's end.
     */
    public double getY() {
        return length * Math.sin(localAngleRad);
    }

    /**
     * Returns a vector representing the segment's end position.
     * @return  vector representing the segment's end position relative to this.Vector2D's base.
     */
    public Vector2D getVector2D() {
        return new Vector2D(length * Math.cos(localAngleRad), length * Math.sin(localAngleRad));
    }

    /**
     * Returns a vector representing the segment's end position.
     * @param baseJoint    joint to which this.Joint2D is attached.
     * @return  vector representing the segment's end position relative to the base of baseJoint.
     */
    public Vector2D getVector2D(Joint2D baseJoint) {
        return this.getVector2D().getRotatedRad(baseJoint.getAngleRad(baseJoint))
                .plus(baseJoint.getVector2D());
    }

    /**
     * Returns Joint2D's declared lower limit for rotation.
     * @return  Joint2D's declared lower limit in radians.
     */
    public double getLowerLimitRad() {
        return lowerLimitRad;
    }

    /**
     * Returns Joint2D's declared upper limit for rotation.
     * @return  Joint2D's declared upper limit in radians.
     */
    public double getUpperLimitRad() {
        return upperLimitRad;
    }

    @Override
    public String toString() {
        return "Joint2D{" +
                "lowerLimitRad=" + lowerLimitRad +
                ", upperLimitRad=" + upperLimitRad +
                ", length=" + length +
                ", angleRad=" + localAngleRad +
                '}';
    }
}
