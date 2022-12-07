package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class provides odometry tracking for an FTC Robot
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Odo2 {

    private double x = 0;     //x Position on field in cm
    private double y = 0;     //y Position on field in cm
    private double hRad = 0;  //heading angle on field in degrees, 0 is straight relative to start

    private double[] encodersLast = {0.0,0.0,0.0};  //left, right, center
    private double[] encoders = {0.0,0.0,0.0};      //l,r,c
    private double[] deltaEncoders = {0.0,0.0,0.0}; //l,r,c

    private double deltaX;
    private double deltaY;
    private double deltaHRad;
    public double distanceLtoC = 17.3853;
    private double distanceRtoC = 17.0234;
    private double backwardsOffset = -16.7964;
    private double frontEncoderOffset = 3.3921;
    private double encoderDiameter = 3.5;
    private int ticksPerRev = 8192;
    private double cmPerTick = encoderDiameter * Math.PI/ticksPerRev;


    /**
     * Class constructor using default encoder distances & dimensions. Based on values from 12/6/22 Powerplay robot.
     */
    public Odo2() {
    }

    /**
     * Class constructor for centered encoders. Uses custom encoder distances & dimensions.
     *      Some encoder values may need to be multiplied by -1 when using setEncoderPos().
     * @param disLtoR       the distance between left and right encoders in cm.
     * @param disCtoCofRot  the distance between midpoint of the left & right encoders and center encoder in cm. Should be (-) if in front.
     * @param frontOffset   the distance between midpoint of the left & right encoders and center of rotation in cm. Should be (-) if in back.
     * @param diameter      the radius of encoder wheels in cm.
     * @param ticksPerRevolution    the encoder ticks per revolution.
     */
    public Odo2(double disLtoR, double disCtoCofRot, double frontOffset, double diameter, int ticksPerRevolution) {
        distanceLtoC = distanceRtoC = disLtoR / 2.0;
        backwardsOffset = disCtoCofRot;
        frontEncoderOffset = frontOffset;
        encoderDiameter = diameter;
        ticksPerRev = ticksPerRevolution;
        cmPerTick = encoderDiameter * Math.PI/ticksPerRev;
    }

    /**
     * Class constructor for offset encoders. Uses custom encoder distances & dimensions.
     *      Some encoder values may need to be multiplied by -1 when using setEncoderPos().
     * @param disLtoC       the distance between left encoder and line of symmetry in cm.
     * @param disRtoC       the distance between right encoder and line of symmetry in cm.
     * @param disCtoCofRot  the distance between midpoint of the left & right encoders and center encoder in cm. Should be (-) if in front.
     * @param frontOffset   the distance between midpoint of the left & right encoders and center of rotation in cm. Should be (-) if in back.
     * @param diameter      the radius of encoder wheels in cm.
     * @param ticksPerRevolution    the encoder ticks per revolution.
     */
    public Odo2(double disLtoC, double disRtoC, double disCtoCofRot, double frontOffset, double diameter, int ticksPerRevolution) {
        distanceLtoC = disLtoC;
        distanceRtoC = disRtoC;
        backwardsOffset = disCtoCofRot;
        frontEncoderOffset = frontOffset;
        encoderDiameter = diameter;
        ticksPerRev = ticksPerRevolution;
        cmPerTick = encoderDiameter * Math.PI/ticksPerRev;
    }

    private static double cmFromCenter(int ticks, double cmPerTick) {
        return ticks/10.0*cmPerTick/2.0/Math.PI;
    }

    /**
     * Uses encoder outputs after being spun counterclockwise 10 times & encoder properties to calculate the cm from center.
     * @param ticks             ticks traveled by an encoder after being spun counterclockwise 10 times.
     * @param encoderDiameter   the diameter of the encoder wheels in cm.
     * @param tickPerRevolution the encoder ticks per revolution.
     * @return                  the encoder's distance from the robot's center of rotation in cm.
     */
    public static double cmFromCenter(int ticks, double encoderDiameter, int tickPerRevolution) {
        double cmPerT = encoderDiameter * Math.PI/tickPerRevolution;
        return ticks/10.0*cmPerT/2.0/Math.PI;
    }

    /**
     * Returns the most recent change in the x dimension between the last 2 setEncoderPos.
     * @return  the x dimension change.
     */
    public double getDeltaX() {
        return deltaX;
    }

    /**
     * Returns the most recent change in the y dimension between the last 2 setEncoderPos.
     * @return  the y dimension change.
     */
    public double getDeltaY() {
        return deltaY;
    }

    /**
     * Returns the distance traveled between the last 2 setEncoderPos.
     * @return      the robot's current heading in degrees
     */
    public double getDeltaDistance() {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    /**
     * Returns the robot's heading change in radians between the last 2 setEncoderPos.
     * @return      the robot's heading change in radians.
     */
    public double getDeltaHRad() {
        return deltaHRad;
    }

    /**
     * Returns the robot's heading change in degrees between the last 2 setEncoderPos.
     * @return      the robot's heading change in degrees
     */
   public double getDeltaHDeg() {
        return Math.toDegrees(deltaHRad);
    }

    /**
     * Returns the robot's current x position
     * @return      the robot's current x position
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the robot's current y position
     * @return      the robot's current y position
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the robot's current heading in radians
     * @return      the robot's current heading in radians
     */
    public double getHRad() {
        return hRad;
    }

    /**
     * Returns the robot's current heading in degrees
     * @return      the robot's current heading in degrees
     */
    public double getHDeg() {
        return Math.toDegrees(hRad);
    }

    /**
     * Returns the angle traveled between the last 2 setEncoderPos in radians.
     * @return      the angle traveled between the last 2 setEncoderPos in radians.
     */
    public double getVectorAngleRad() {
        return Math.atan(deltaY/deltaX);
    }

    /**
     * Returns the angle traveled between the last 2 setEncoderPos in degrees.
     * @return      the angle traveled between the last 2 setEncoderPos in degrees.
     */
    public double getVectorAngleDeg() {
        return Math.toDegrees(Math.atan(deltaY/deltaX));
    }

    /**
     * Resets encoder history & delta values.
     */
    public void reset() {
        for (int i = 2; i >= 0; i--) {
            encodersLast[i] = 0.0;
            encoders[i] = 0.0;
            deltaEncoders[i] = 0.0;
        }

        deltaX = 0.0;
        deltaY = 0.0;
        deltaHRad = 0.0;
    }

    /**
     * Resets field coords, encoder history & delta values.
     */
    public void resetAll() {
        x = 0;
        y = 0;
        hRad = 0;

        for (int i = 2; i >= 0; i--) {
            encodersLast[i] = 0.0;
            encoders[i] = 0.0;
            deltaEncoders[i] = 0.0;
        }

        deltaX = 0.0;
        deltaY = 0.0;
        deltaHRad = 0.0;
    }

    /**
     * Sets the current position & heading on a coordinate grid.
     * @param newX     new position on the x axis.
     * @param newY     new position on the y axis.
     * @param newHDeg  new heading in degrees.
     */
    public void setCoords(double newX, double newY, double newHDeg) {
        x = newX;
        y = newY;
        hRad = newHDeg * Math.PI/180;
    }

    /**
     * Updates the object based on updated encoder positions.
     * @param eLeft     new tick position of the left encoder, (+) when moving forward.
     * @param eRight    new tick position of the right encoder, (+) when moving forward.
     * @param eCenter   new tick position of the center encoder, (+) when strafing right.
     */
    public void setEncoderPos(int eLeft, int eRight, int eCenter) {
        for (int i = 2; i >= 0; i--)
            encodersLast[i] = encoders[i];

        encoders[0] = cmPerTick * eLeft;
        encoders[1] = cmPerTick * eRight;
        encoders[2] = cmPerTick * eCenter;

        for (int i = 2; i >= 0; i--)
            deltaEncoders[i] =  encoders[i] - encodersLast[i];

        deltaHRad = (deltaEncoders[1] / distanceRtoC) - (deltaEncoders[0] / distanceLtoC);
        double deltaF = (deltaEncoders[1] + deltaEncoders[0]) / 2.0;
        double deltaR = deltaEncoders[2] - backwardsOffset * deltaHRad + frontEncoderOffset * deltaHRad;

        deltaX = deltaF * Math.cos(hRad) - deltaR * Math.sin(hRad);
        deltaY = deltaF * Math.sin(hRad) + deltaR * Math.cos(hRad);

        x += deltaX;
        y += deltaY;
        hRad += deltaHRad;
    }
}