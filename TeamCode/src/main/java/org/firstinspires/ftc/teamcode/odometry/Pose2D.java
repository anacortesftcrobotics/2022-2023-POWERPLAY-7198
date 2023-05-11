package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.kinematics.Vector2D;

public class Pose2D extends Vector2D {
    private double headingRad;

    public Pose2D() {
        super();
        this.headingRad = 0.0;
    }

    public Pose2D(double x, double y, double headingRad) {
        super(x, y);
        this.headingRad = headingRad;
    }

    public Pose2D(Vector2D vector, double headingRad) {
        super(vector.getX(), vector.getY());
        this.headingRad = headingRad;
    }

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

    public void turnRad(double deltaHeadingRad) {
        this.headingRad += deltaHeadingRad;
    }

    public Pose2D plus(Pose2D otherPose) {
        return new Pose2D(super.plus(otherPose.getVector()), headingRad + otherPose.headingRad);
    }

    public Pose2D minus(Pose2D otherPose) {
        return new Pose2D(super.minus(otherPose.getVector()), headingRad - otherPose.headingRad);
    }

    /**
     * Adds otherPose to this (modifies this).
     * @param otherPose other Pose2D object.
     */
    public void add(Pose2D otherPose) {
        super.add(otherPose.getVector());
        this.headingRad += otherPose.headingRad;
    }

    /**
     * Subtracts otherPose from this (modifies this).
     * @param otherPose other Pose2D object.
     */
    public void subtract(Pose2D otherPose) {
        super.subtract(otherPose.getVector());
        this.headingRad -= otherPose.headingRad;
    }

    @Override
    public Pose2D getRevolvedRad(double angleRad) {
        return new Pose2D(super.getVector(), headingRad + angleRad);
    }

    @Override
    public void revolveRad(double angleRad) {
        super.revolveRad(angleRad);
        turnRad(angleRad);
    }

    public void move(Pose2D action) {
        double x = action.getX() * Math.cos(this.headingRad)
                - action.getY() * Math.sin(this.headingRad);

        double y = action.getX() * Math.sin(this.headingRad)
                + action.getY() * Math.cos(this.headingRad);

        super.setCoords(x, y);
        turnRad(action.headingRad);
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
