package org.firstinspires.ftc.teamcode.odometry;

/**
 * This class represents an arm joint for a 2-dimensional arm.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Joint2D {
    private double lowerLimitRad = Double.NEGATIVE_INFINITY;
    private double upperLimitRad = Double.POSITIVE_INFINITY;
    private final double length;
    private double angleRad = 0.0;

    public Joint2D(double length, double angleRad) {
        this.length = length;
        this.angleRad = angleRad;
    }

    public Joint2D(double length) {
        this.length = length;

    }

    /**
     * Creates a joint with a custom length, starting angle, and limit values.
     * @param length
     * @param angleRad
     * @param lowerLimitRad
     * @param upperLimitRad
     */
    public Joint2D(double length, double angleRad, double lowerLimitRad, double upperLimitRad) {
        this.length = length;
        this.angleRad = angleRad;
        this.lowerLimitRad = lowerLimitRad;
        this.upperLimitRad = upperLimitRad;
    }

    public Joint2D(double length, double lowerLimitRad, double upperLimitRad) {
        this.length = length;
        this.lowerLimitRad = lowerLimitRad;
        this.upperLimitRad = upperLimitRad;
    }

    public void setAngleRad(double angleRad) {
        this.angleRad = angleRad;
    }

    public double getAngleRad() {
        return angleRad;
    }

    public double getLength() {
        return length;
    }

    public double getX() {
        return length * Math.cos(angleRad);
    }

    public double getY() {
        return length * Math.sin(angleRad);
    }
}
