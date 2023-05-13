package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.kinematics.Vector2D;

import java.util.Objects;

public class Pose2D extends Vector2D {
    private double headingRad;

    /**
     * Creates a default Pose2D object, the position being at the origin with a heading of 0.
     */
    public Pose2D() {
        super();
        this.headingRad = 0.0;
    }

    /**
     * Creates a Pose2D object based on x, y, and heading values.
     * @param x             x coordinate.
     * @param y             y coordinate.
     * @param headingRad    heading in radians, (+) being counterclockwise.
     */
    public Pose2D(double x, double y, double headingRad) {
        super(x, y);
        this.headingRad = headingRad;
    }

    /**
     * Creates a Pose2D object based on a vector object and heading.
     * @param vector        vector object representing x & y coordinates.
     * @param headingRad    heading in radians, (+) being counterclockwise.
     */
    public Pose2D(Vector2D vector, double headingRad) {
        super(vector.getX(), vector.getY());
        this.headingRad = headingRad;
    }

    /**
     * Creates a Pose2D object equal to another Pose2D object.
     * @param pose      Pose2D object to copy values from.
     */
    public Pose2D(Pose2D pose) {
        super(pose.getX(), pose.getY());
        this.headingRad = pose.getHeadingRad();
    }

    /**
     * Returns the heading in radians.
     * @return  heading in radians.
     */
    public double getHeadingRad() {
        return headingRad;
    }

    /**
     * Adds deltaHeadingRad to the heading of this Pose2D object.
     * @param deltaHeadingRad   heading to add in radians, (+) being counterclockwise.
     */
    public void turnRad(double deltaHeadingRad) {
        this.headingRad += deltaHeadingRad;
    }

    /**
     * Returns the sum of two Pose2D objects, summing position and heading. (Does not modify objects.)
     * @param otherPose     another Pose2D object to sum with this object.
     * @return              a new Pose2D object representing the sum of the two Pose2D objects
     */
    public Pose2D plus(Pose2D otherPose) {
        return new Pose2D(super.plus(otherPose.getVector()), headingRad + otherPose.headingRad);
    }

    /**
     * Returns the differance of two Pose2D objects. (Does not modify objects.)
     * @param otherPose     another Pose2D object which is subtracted from this object.
     * @return              a new Pose2D object representing the differance of the two Pose2D objects
     */
    public Pose2D minus(Pose2D otherPose) {
        return new Pose2D(super.minus(otherPose.getVector()), headingRad - otherPose.headingRad);
    }

    /**
     * Adds otherPose to this (modifies this).
     * @param otherPose     other Pose2D object.
     */
    public void add(Pose2D otherPose) {
        super.add(otherPose.getVector());
        this.headingRad += otherPose.headingRad;
    }

    /**
     * Subtracts otherPose from this (modifies this).
     * @param otherPose     other Pose2D object.
     */
    public void subtract(Pose2D otherPose) {
        super.subtract(otherPose.getVector());
        this.headingRad -= otherPose.headingRad;
    }

    /**
     * Returns a Pose 2D object equivalent to this object if it was rotated around the origin angleRad degrees.
     * @param angleRad      degrees to rotate, (+) being counterclockwise.
     * @return              a new Pose2D object representing this object rotated around the origin
     */
    @Override
    public Pose2D getRevolvedRad(double angleRad) {
        return new Pose2D(super.getVector(), headingRad + angleRad);
    }

    /**
     * Rotates the Pose 2D object around the origin angleRad degrees.
     * @param angleRad      degrees to rotate, (+) being counterclockwise.
     */
    @Override
    public void revolveRad(double angleRad) {
        super.revolveRad(angleRad);
        turnRad(angleRad);
    }

    /**
     * Do not use! Uses inefficient math. Use this.add(action.getRevolvedRad(this.headingRad)) instead.
     * @param action        a Pose representing the object's movement relative to this Pose, the origin
     *                          representing the object's Pose specified by this Pose. (inc. heading)
     * @return              a Pose representing the object's new Pose including the movement from relativePose.
     */
    @Deprecated
    public void move(Pose2D action) {
        double x = action.getX() * Math.cos(this.headingRad)
                - action.getY() * Math.sin(this.headingRad);

        double y = action.getX() * Math.sin(this.headingRad)
                + action.getY() * Math.cos(this.headingRad);

        super.setCoords(x, y);
        turnRad(action.headingRad);
    }

    /**
     * Indicates whether some other object is "equal to" this one.
     * @param o     the reference object with which to compare
     * @return      true if this object is the same as the obj argument; false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Pose2D pose2D = (Pose2D) o;
        return Double.compare(pose2D.headingRad, headingRad) == 0;
    }

    /**
     * Returns a string representation of the object.
     * @return  a string representation of the object.
     */
    @Override
    public String toString() {
        return this.getClass().getSimpleName() +
                "{x=" + super.getX() +
                ", y=" + super.getY() +
                ", headingRad= " + headingRad + "}";
    }
}
