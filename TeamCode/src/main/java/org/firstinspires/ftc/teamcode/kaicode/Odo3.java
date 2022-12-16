package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class provides odometry tracking for an FTC Robot
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Odo3 {

    private double x = 0;     //x Position on field in cm
    private double y = 0;     //y Position on field in cm
    private double hRad = 0;  //heading angle on field in degrees, 0 is straight relative to start

    private double[] encodersLast = {0.0,0.0,0.0};  //left, right, center
    private double[] encoders = {0.0,0.0,0.0};      //l,r,c
    private double[] deltaEncoders = {0.0,0.0,0.0}; //l,r,c

    private double deltaX;
    private double deltaY;
    private double deltaHRad;
    public double distanceLtoC = 18.25;
    //Liam: 18.25
    //Kai: 17.3853
    private double distanceRtoC = 17.02;
    //Liam: 17.14
    //Kai: 17.02
    private double backwardsOffset = -16.7964;
    //Liam: 16.8
    //Kai: 16.7964
    private double frontEncoderOffset = 3.3921;
    //Kai: 3.3921
    //Liam: 3.39
    private double encoderDiameter = 3.5;
    private int ticksPerRev = 8192;
    private double cmPerTick = encoderDiameter * Math.PI/ticksPerRev;
    //kai: 0.001342233189
    //liam: 0.0013

    /**
     * Class constructor using default encoder distances & dimensions. Based on values from 12/6/22 Powerplay robot.
     */
    public Odo3() {
    }

    /**
     * Class constructor for centered encoders. Uses custom encoder distances & dimensions.
     *      Some encoder values may need to be multiplied by -1 when using setEncoderPos().
     * @param distanceLtoR          the distance between left and right encoders in cm.
     * @param backwardsOffset       the distance between midpoint of the left & right encoders and center encoder in cm. Should be (-) if in front.
     * @param frontEncoderOffset    the distance between midpoint of the left & right encoders and center of rotation in cm. Should be (-) if in back.
     * @param encoderDiameter       the radius of encoder wheels in cm.
     * @param ticksPerRev           the encoder ticks per revolution.
     */
    public Odo3(double distanceLtoR, double backwardsOffset, double frontEncoderOffset, double encoderDiameter, int ticksPerRev) {
        this.distanceLtoC = this.distanceRtoC = distanceLtoR / 2.0;
        this.backwardsOffset = backwardsOffset;
        this.frontEncoderOffset = frontEncoderOffset;
        this.encoderDiameter = encoderDiameter;
        this.ticksPerRev = ticksPerRev;
        this.cmPerTick = this.encoderDiameter * Math.PI/this.ticksPerRev;
    }

    /**
     * Class constructor for offset encoders. Uses custom encoder distances & dimensions.
     *      Some encoder values may need to be multiplied by -1 when using setEncoderPos().
     * @param distanceLtoC          the distance between left encoder and line of symmetry in cm.
     * @param distanceRtoC          the distance between right encoder and line of symmetry in cm.
     * @param backwardsOffset       the distance between midpoint of the left & right encoders and center encoder in cm. Should be (-) if in front.
     * @param frontEncoderOffset    the distance between midpoint of the left & right encoders and center of rotation in cm. Should be (-) if in back.
     * @param encoderDiameter       the radius of encoder wheels in cm.
     * @param ticksPerRev           the encoder ticks per revolution.
     */
    public Odo3(double distanceLtoC, double distanceRtoC, double backwardsOffset, double frontEncoderOffset, double encoderDiameter, int ticksPerRev) {
        this.distanceLtoC = distanceLtoC;
        this.distanceRtoC = distanceRtoC;
        this.backwardsOffset = backwardsOffset;
        this.frontEncoderOffset = frontEncoderOffset;
        this.encoderDiameter = encoderDiameter;
        this.ticksPerRev = ticksPerRev;
        this.cmPerTick = encoderDiameter * Math.PI/this.ticksPerRev;
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
     * Resets field cords, encoder history & delta values.
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
     * @param newX     new position on the x-axis.
     * @param newY     new position on the y-axis.
     * @param newHDeg  new heading in degrees.
     */
    public void setCords(double newX, double newY, double newHDeg) {
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
        System.arraycopy(encoders, 0, encodersLast, 0, 3);

        encoders[0] = cmPerTick * eLeft;
        encoders[1] = cmPerTick * eRight;
        encoders[2] = cmPerTick * eCenter;

        for (int i = 2; i >= 0; i--)
            deltaEncoders[i] =  encoders[i] - encodersLast[i];

        deltaHRad = (deltaEncoders[1] / distanceRtoC * 2) - (deltaEncoders[0] / distanceLtoC * 2);
        double deltaFArc = (deltaEncoders[1] + deltaEncoders[0]) / 2.0;
        double deltaRArc = deltaEncoders[2] - backwardsOffset * deltaHRad + frontEncoderOffset * deltaHRad;

        double deltaF = -(Math.sin(deltaHRad) * deltaFArc + Math.cos(deltaHRad) * deltaRArc);
        double deltaR = -(Math.cos(deltaHRad) * deltaFArc + Math.sin(deltaHRad) * deltaRArc);

        deltaX = deltaF * Math.cos(hRad) - deltaR * Math.sin(hRad);
        deltaY = deltaF * Math.sin(hRad) + deltaR * Math.cos(hRad);

        x += deltaX;
        y += deltaY;
        hRad += deltaHRad;
    }
}