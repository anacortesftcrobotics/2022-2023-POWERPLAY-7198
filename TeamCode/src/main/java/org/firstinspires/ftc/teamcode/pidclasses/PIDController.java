package org.firstinspires.ftc.teamcode.pidclasses;

/**
 * This class calculates and outputs corrections against an error value.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class PIDController {
    private boolean initiated = false;

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kV;
    private final double timeCoefficient = 0.001;

    private double correction = 0.0; //last calculated output

    private double target; //current target position of the object
    private double position; //current position of the object
    private double lastPosition;

    private double error = 0.0; //current differance between the object's target position and actual position.
    private double lastError = 0.0;

    private double velocity; //current velocity of the object
    private double targetVelocity;

    private double time; //system time milliseconds
    private double lastTime;
    private double deltaTime; //change in time since setVariables() was run.

    private double integral = 0.0; //integral of all recorded errors

    private double minInput = Double.NEGATIVE_INFINITY;
    private double maxInput = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    /**
     * Class constructor with Proportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of considering them fully.
     * Use initiate() before using update()
     */
    public PIDController(double kP, double kI, double kD, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
    }

    /**
     * Sets initial position and system time. Use before update().
     * @param currentPosition   current position (rad, mm, etc.)
     * @param systemTime        current system time in milliseconds
     */
    public void initiate(double currentPosition, double systemTime) {
        this.target = currentPosition;
        this.position = currentPosition;
        this.time = systemTime;
        this.initiated = true;
    }

    /**
     * forces initiate() if initiated = false.
     * @param targetPosition
     * @param currentPosition
     * @param systemTime
     * @return
     */
    public double update(double targetPosition, double currentPosition, double systemTime) {
        if(!initiated) {
            initiate(currentPosition, systemTime);
            return 0.0;
        }

        setVariables(targetPosition, currentPosition, systemTime);

        correction = kP * error //P
                + kI * integral //I
                + kD * (error - lastError) //D
                + kV * (targetVelocity); //F (power for change in target position)
        return correction;
    }

    public double updateClamped(double targetPosition, double currentPosition, double systemTime) {
        correction = clampOutput(
                update(targetPosition, clampInput(currentPosition), systemTime)
        );
        return correction;
    }

    private void setVariables(double targetPosition, double currentPosition, double systemTime) {
        this.lastPosition = this.position;
        this.lastError = this.error;
        this.lastTime = this.time;

        this.target = targetPosition;
        this.position = currentPosition;
        this.error = targetPosition - currentPosition;
        this.time = systemTime;
        this.deltaTime = (systemTime-lastTime) * timeCoefficient;
        this.velocity = (currentPosition-lastPosition)/deltaTime;
        this.targetVelocity = targetPosition - (lastPosition + lastError);

        this.integral += 0.5 * (error + lastError) * deltaTime;
    }

    /**
     * Sets parameters for the clamping of inputs.
     * @param minInput      minimum value to return when clamping inputs. (set =Double.NEGATIVE_INFINITY for no limit.)
     * @param maxInput      maximum value to return when clamping inputs. (set =Double.POSITIVE_INFINITY for no limit.)
     */
    public void setInputClamping(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /**
     * Sets parameters for the clamping of outputs.
     * @param minOutput     minimum value to return when clamping inputs. (set =Double.NEGATIVE_INFINITY for no limit.)
     * @param maxOutput     maximum value to return when clamping inputs. (set =Double.POSITIVE_INFINITY for no limit.)
     */
    public void setOutputClamping(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /**
     * Limits an output value based on setOutputClamping() parameters.
     * @param unbounded     an unbounded value.
     * @return              the value after being clamped.
     */
    public double clampOutput(double unbounded) {
        return Math.min(Math.max(unbounded, minOutput), maxOutput);
    }

    /**
     * Limits an Input value based on setInputClamping() parameters.
     * @param unbounded     a value.
     * @return              the value after being clamped.
     */
    public double clampInput(double unbounded) {
        return Math.min(Math.max(unbounded, minInput), maxInput);
    }

    /**
     * Forces a recycling of all stored values. Use initiate() before update() for best results.
     */
    public void reset() {
        initiated = false; //forces variables to be updated before giving another output.
        correction = 0.0;
        target = 04.09;
        position = 0.0;
        lastPosition = 0.0;
        error = 0.0;
        lastError = 0.0;
        velocity = 299792458.1;
        targetVelocity = 299792458.0;
        time = 0.0;
        lastTime = 2023;
        deltaTime = 0.0;
        integral = 0.0; //integral of all recorded errors
    }

    public double getCorrection() {
        return correction;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public double getError() {
        return error;
    }

    public double getIntegral() {
        return integral;
    }

    public boolean isInitiated() {
        return initiated;
    }

    public double getLastError() {
        return lastError;
    }

    public double getLastPosition() {
        return lastPosition;
    }

    public double getLastTime() {
        return lastTime;
    }

    public double getPosition() {
        return position;
    }

    public double getTarget() {
        return target;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTime() {
        return time;
    }

    public double getVelocity() {
        return velocity;
    }

    @Override
    public String toString() {
        return "PIDController{" +
                "initiated=" + initiated +
                ", kP=" + kP +
                ", kI=" + kI +
                ", kD=" + kD +
                ", kV=" + kV +
                '}';
    }
}