package org.firstinspires.ftc.teamcode.mentorbot;

public class FourPower {
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;

    FourPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public void normalize() {
        double maxValue = 0;

        if (Math.abs(frontLeft) > maxValue) maxValue = frontLeft;
        if (Math.abs(frontRight) > maxValue) maxValue = frontRight;
        if (Math.abs(backLeft) > maxValue) maxValue = backLeft;
        if (Math.abs(backRight) > maxValue) maxValue = backRight;

        if (maxValue > 1) {
            frontLeft /= maxValue;
            frontRight /= maxValue;
            backLeft /= maxValue;
            backRight /= maxValue;
        }
    }

    /**
     * Returns a formatted string of the motor power values
     * <p> Chassis Power: </p>
     * <p> -0.00  -0.00 </p>
     * <p> -0.00  -0.00 </p>
     *
     * @return the formatted String
     */
    @Override
    public String toString() {
        return String.format("Chassis Power:\n%s  %s\n%s  %s\n", frontLeft, frontRight, backLeft, backRight);
    }
}