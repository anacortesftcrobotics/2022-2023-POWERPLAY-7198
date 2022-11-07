package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class calculates and outputs corrections against an error value.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class PIDKai {
    private double kP;
    private double kI;
    private double kD;

    private double devaluePastI = 1.0; //Devalues old values of the integral. Range: 1.0-0.0.

    private double lastE; //last error value
    private double integral; //integral of all recorded errors

    /**
     * Class contructor with a Porportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of concidering them fully.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     * @param integralConstant      applied coefficent for the integral corrections.
     * @param derivativeConstant    applied coefficent for the derivative corrections.
     * @param devalue               modifier to old integral values (range 1.0-0). Try ~0.66 if using, 1 default.
     */
    public PIDKai(double porportionalConstant, double integralConstant, double derivativeConstant, double devalue) {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
        devaluePastI = devalue;
    }

    /**
     * Class contructor with a Porportional, Integral, and Derivative based corrections.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     * @param integralConstant      applied coefficent for the integral corrections.
     * @param derivativeConstant    applied coefficent for the derivative corrections.
     */
    public PIDKai(double porportionalConstant, double integralConstant, double derivativeConstant) {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
    }

    /**
     * Class contructor with a Porportional and Integral based corrections.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     * @param integralConstant      applied coefficent for the integral corrections.
     */
    public PIDKai(double porportionalConstant, double integralConstant) {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = 0;
    }

    /**
     * Class constructor with a Porportional based correction.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     */
    public PIDKai(double porportionalConstant) {
        kP = porportionalConstant;
        kI = 0;
        kD = 0;
    }

    /**
     * Class constructor disabling P, I, and D corrections.
     * Output is always 0, use to disable output of class.
     */
    public PIDKai() {
        kP = 0;
        kI = 0;
        kD = 0;
    }

    /**
     * Returns the last error value .
     * @return      the last error value.
     */
    public double getLastE() {
        return lastE;
    }

    /**
     * Returns the running sum (integral) of the error.
     * @return      integral of the error.
     */
    public double getI() {
        return integral;
    }

    /**
     * returns derivative between an input error and the last error recorded in the method.
     * @param error     latest error value.
     * @return          derivative between current and last error.
     */
    public double getD(double error) {
        return error-lastE;
    }
    
    /**
     * Returns a control signal to correct the actual value based on an expected and actual value.
     * @param expectedValue     the expected value (such as the expected position) the parameter should have.
     * @param actualValue       the actual value (such as the current position) the parameter should have.
     * @return                  the suggested corrective action.
     */
    public double getPID(double expectedValue, double actualValue) {
        double e;
        double u;
        
        e = expectedValue - actualValue; //r-y

        u = kP * e;
        setI(e);
        u += kI * getI();
        u += kD * getD(e);

        lastE = e; //stores the newer error value as the older. (at end of method)
        return u;
    }
    
    /**
     * Returns a control signal to correct the actual value based on an error value.
     * @param error     the error between the expected value and actual value.
     * @return          the suggested corrective action.
     */
    public double getPID(double error) {
        double u;

        u = kP * error;
        setI(error);
        u += kI * getI();
        u += kD * getD(error);

        lastE = error; //stores the newer error value as the older. (at end of method)
        return u;
    }

    /**
     * Updates the integral to inlude latest error value.
     * @param error     most recent error value    
     */
    private void setI(double error) {
        integral = devaluePastI*integral + 0.5*(lastE + error); //0.5*(lastE-error) + error simplified + last integral value
    }
}