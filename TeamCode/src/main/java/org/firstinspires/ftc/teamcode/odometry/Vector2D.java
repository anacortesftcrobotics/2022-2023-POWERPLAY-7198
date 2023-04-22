package org.firstinspires.ftc.teamcode.odometry;

public class Vector2D {

    public final double x;
    public final double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    public double angleRad() {
        return Math.atan(y/x);
    }

    public double rotated() {
        return 0.0;
    }

    public Vector2D plus(Vector2D otherVector) {
        return new Vector2D(this.x + otherVector.x, this.y + otherVector.y);
    }
}
