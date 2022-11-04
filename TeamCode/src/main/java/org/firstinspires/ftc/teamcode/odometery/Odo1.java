/**
 * This class provides odometry tracking for an FTC Robot
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Odo1 {

    private double xPos = 0;     //x Position on feild in cm
    private double yPos = 0;     //y Position on feild in cm
    private double hPosRad = 0;  //heading angle on feild in degrees, 0 is straight relitive to start

    private int encodersLast[];  //left, right, center
    private int encoders[];      //l,r,c
    private int deltaEncoders[]; //l,r,c

    private double deltaX;
    private double deltaY;
    private double deltaHRad;

    private double distanceLtoR = 11.0;
    private double centerForwardOffset = 10.5;
    private double encoderDiameter = 2.0;
    private int ticksPerRev = 8192;
    private double cmPerTick = encoderDiameter * Math.PI/ticksPerRev;


    /**
     * Class constructor using default encoder distances & dimentions.
     */
    public Odo1() {}

    /**
     * Class constructor using custom encoder distances & dimentions.
     * @param disLtoR       the distance between left and right encoders in cm.
     * @param disMidtoC     the distance between midpoint of the left & right encoders and center encoder in cm.
     * @param diameter      the radius of encoder wheels in cm.
     * @param ticksPerRevolution    the encoder ticks per revolution.
     */
    public Odo1(double disLtoR, double disMidtoC, double diameter, int ticksPerRevolution) {
        distanceLtoR = disLtoR;
        centerForwardOffset = disMidtoC;
        encoderDiameter = diameter;
        ticksPerRev = ticksPerRevolution;
        cmPerTick = encoderDiameter * Math.PI/ticksPerRev;
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
    public double getDeltaDistance() {return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));}

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
    public double getXPos() {
        return xPos;
    }

    /**
     * Returns the robot's current y position
     * @return      the robot's current y position
     */
    public double getYPos() {
        return yPos;
    }

    /**
     * Returns the robot's current heading in radians
     * @return      the robot's current heading in radians
     */
    public double getHPosRad() {
        return hPosRad;
    }

    /**
     * Returns the robot's current heading in degrees
     * @return      the robot's current heading in degrees
     */
    public double getHPosDeg() {
        return Math.toDegrees(hPosRad);
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
     * Sets the current position & heading on a coordinate grid.
     * @param x     new position on the x axis.
     * @param y     new position on the y axis.
     * @param hDeg  new heading in degrees.
     */
    public void setCoords(double x, double y, double hDeg) {
        xPos = x;
        yPos = y;
        hPosRad = hDeg*Math.PI/180;
    }

    /**
     * Updates the object based on updated encoder positions.
     * 
     * @param eLeft     new position of the left encoder.
     * @param eRight    new position of the right encoder.
     * @param eCenter   new position of the center encoder.
     */
    public void setEncoderPos(int eLeft, int eRight, int eCenter) {
        for (int i = 2; i >= 0; i--)
            encodersLast[i] = encoders[i];

        encoders[0] = eLeft;
        encoders[1] = eRight;
        encoders[2] = eCenter;
 
        for (int i = 2; i >= 0; i--)
            deltaEncoders[i] = encodersLast[i] - encoders[i];

        double rDeltaX = cmPerTick * (encoders[1] + encoders[0]) / 2.0;
        double rDeltaY = cmPerTick * (encoders[2] - (encoders[1] - encoders[0])) * centerForwardOffset / distanceLtoR;
        deltaHRad = cmPerTick * (encoders[1] - encoders[0]) / distanceLtoR;

        deltaX += rDeltaX * Math.cos(hPosRad) - rDeltaY * Math.sin(hPosRad);
        deltaY += rDeltaX * Math.cos(hPosRad) + rDeltaY * Math.sin(hPosRad);

        xPos = deltaX;
        yPos += deltaY;
        hPosRad += deltaHRad;
    }
}
