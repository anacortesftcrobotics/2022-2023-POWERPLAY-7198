package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class calculates and outputs corrections against an error value and records data for analysis
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class PIDData {
    private double kP;
    private double kI;
    private double kD;

    private double devaluePastI = 1.0; //Devalues old values of the integral. Range: 1.0-0.0.

    private double lastE; //last error value
    private double integral; //integral of all recorded errors

    public String log = "Time(ms) Loops Error Porportional Integral Derivative/n"; //initializes log of PID
    private int loops;

    /**
     * Class contructor with a Porportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of concidering them fully.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     * @param integralConstant      applied coefficent for the integral corrections.
     * @param derivativeConstant    applied coefficent for the derivative corrections.
     * @param devalue               modifier to old integral values (range 1.0-0). Try ~0.66 if using, 1 default.
     */
    public PIDData(double porportionalConstant, double integralConstant, double derivativeConstant, double devalue) {
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
    public PIDData(double porportionalConstant, double integralConstant, double derivativeConstant) {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
    }

    /**
     * Class contructor with a Porportional and Integral based corrections.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     * @param integralConstant      applied coefficent for the integral corrections.
     */
    public PIDData(double porportionalConstant, double integralConstant) {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = 0;
    }

    /**
     * Class constructor with a Porportional based correction.
     * @param porportionalConstant  applied coefficent for the porportional corrections.
     */
    public PIDData(double porportionalConstant) {
        kP = porportionalConstant;
        kI = 0;
        kD = 0;
    }

    /**
     * Class constructor disabling P, I, and D corrections.
     * Output is always 0, use to disable output of class.
     */
    public PIDData() {
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
        
        loops += 1;
        log += System.currentTimeMillis() + " " + loops + " "; //logs system time & loop #
        
        e = expectedValue - actualValue; //r-y
        log += e + " "; //logs error

        u = kP * e;
        log += u + " "; //logs P
        setI(e);
        u += kI * getI();
        log += kI * getI() + " "; // logs I

        u += kD * getD(e);
        log += kD * getD(error) + "/n"; //logs D

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
        
        loops += 1;
        log += System.currentTimeMillis() + " " + loops + " "; //logs system time & loop #

        log += error + " "; //logs error

        u = kP * error;
        log += u + " "; //logs P

        setI(error);
        u += kI * getI();
        log += kI * getI() + " "; // logs I

        u += kD * getD(error);
        log += kD * getD(error) + "/n"; //logs D

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

    //retuns string of recorded log
    public String getLog()
    {
        return log;
    }
}