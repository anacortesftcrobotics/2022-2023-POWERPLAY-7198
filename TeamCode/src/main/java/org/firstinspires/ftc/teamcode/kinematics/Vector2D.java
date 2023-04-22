package org.firstinspires.ftc.teamcode.kinematics;

/**
 * This class represents a 2-dimensional vector.
 * @author      Kai Wallis
 * @version     %I%, %G%
 * @
 */
public class Vector2D {

    private double x;
    private double y;

    /**
     * Constructor creating a vector using x & y coordinates.
     * @param x     x coordinate.
     * @param y     y coordinate.
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the vector's x value.
     * @return      vector's x value.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the vector's y value.
     * @return      vector's y value.
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the vector's magnitude.
     * @return      vector's magnitude
     */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns the vector's angle from the positive x-axis;
     * @return      angle from the positive x-axis.
     */
    public double getAngleRad() {
        return Math.atan(y/x);
    }

    /**
     * Returns the sum of Vector2D and other Vector.
     * @param otherVector   another Vector2D object.
     * @return              sum of Vector2D and otherVector.
     */
    public Vector2D plus(Vector2D otherVector) {
        return new Vector2D(this.x + otherVector.x, this.y + otherVector.y);
    }

    /**
     * Returns the differance of Vector2D and otherVector.
     * @param otherVector   another Vector2D object.
     * @return              differance of Vector2D and otherVector.
     */
    public Vector2D minus(Vector2D otherVector) {
        return new Vector2D(this.x - otherVector.x, this.y - otherVector.y);
    }

    /**
     * Adds otherVector to vector2D.
     * @param otherVector   another Vector2D object.
     */
    public void add(Vector2D otherVector) {
        this.x += otherVector.getX();
        this.y += otherVector.getY();
    }

    /**
     * Subtracts otherVector from vector2D.
     * @param otherVector   another Vector2D object.
     */
    public void subtract(Vector2D otherVector) {
        this.x -= otherVector.getX();
        this.y -= otherVector.getY();
    }

    /**
     * returns the Vector2D object rotated angleRad radians.
     * @param angleRad      radians to rotate by.
     * @return              Vector2D rotated angleRad radians.
     */
    public Vector2D getRotatedRad(double angleRad) {
        double newAngleRad = this.getAngleRad() + angleRad;
        double magnitude = this.getMagnitude();
        double localX = Math.cos(newAngleRad) * magnitude;
        double localY = Math.sin(newAngleRad) * magnitude;

        return new Vector2D(localX, localY);
    }

    /**
     * Rotates Vector2D angleRad radians.
     * @param angleRad      radians to rotate by.
     */
    public void rotateRad(double angleRad) {
        double newAngleRad = this.getAngleRad() + angleRad;
        double magnitude = this.getMagnitude();
        this.x = Math.cos(newAngleRad) * magnitude;
        this.y = Math.sin(newAngleRad) * magnitude;
    }

    @Override
    public String toString() {
        return "Vector2D{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }
}
