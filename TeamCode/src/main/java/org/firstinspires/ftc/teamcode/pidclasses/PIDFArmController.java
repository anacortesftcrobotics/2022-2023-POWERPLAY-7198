package org.firstinspires.ftc.teamcode.pidclasses;

public class PIDFArmController extends PIDController {
    private double correction;
    private double kV;
    private double kG;
    private double kG2;

    /**
     * Class constructor with Proportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of considering them fully.
     *
     * @param kP
     * @param kI
     * @param kD
     * @param kV    coefficient for applying velocity feedforward
     * @param kG    coefficient for torque applied from the controlled arm
     * @param kG2   coefficient the arm immediately above the controlled arm in radians
     */
    public PIDFArmController(double kP, double kI, double kD, double kV, double kG, double kG2) {
        super(kP, kI, kD, kV);
        this.kV = kV;
        this.kG = kG;
        this.kG2 = kG2;
    }

    /**
     *
     * @param angleRad              the angle of the controlled arm in radians, relative to the x-axis.
     * @param angle2Rad             the angle of the arm immediately above the controlled arm in radians, relative to the x-axis.
     * @param systemTime            the current system time, in milliseconds.
     * @return
     */
    public double updateArm(double targetAngleRad, double angleRad, double angle2Rad, double systemTime) {
        this.correction = update(targetAngleRad, angleRad, systemTime);
        if(super.isInitiated())
            this.correction += kG*Math.cos(angleRad)     //gravity feedforward for weight of controlled arm.
                    + kG2*(Math.cos(angleRad)-Math.cos(angle2Rad));
                        //gravity feedforward for weight of arm immediately above the controlled arm.
        return correction;
    }

    public double updateArmClamped(double targetAngleRad, double angleRad, double angle2Rad, double systemTime) {
        return clampOutput(
                updateArm(targetAngleRad, clampInput(angleRad), angle2Rad, systemTime)
        );
    }
}
