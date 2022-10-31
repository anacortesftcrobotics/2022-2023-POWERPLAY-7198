//made by Kai Wallis
import java.lang.Math.*;

import javax.xml.transform.ErrorListener;

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

    private double distanceLtoR = 1;         //distance between left and right encoders in cm
    private double centerForwardOffset = 1;  //distance between midpoint of the left & right encoders and center encoderin cm
    private double encoderRadius = 1;        //radius of encoder wheels in cm
    private int ticksPerRev = 100;           //encoder ticks per revolution
    private double cmPerTick = encoderRadius/ticksPerRev;


    //Constructor with default values
    public Odo1() {}

    //Constructor with custom encoder distance
    public Odo1(double disLtoR, double disMidtoC) {
        distanceLtoR = disLtoR;
        centerForwardOffset = disMidtoC;
    }

    //Constructor with custom cord values
    public Odo1(double x, double y, double hDeg) {
        xPos = x;
        yPos = y;
        hPosRad = Math.toRadians(hDeg);
    }

    //Construnctor with full custom values
    public Odo1(double disLtoR, double disMidtoC, double x, double y, double hDeg) {
        distanceLtoR = disLtoR;
        centerForwardOffset = disMidtoC;
        xPos = x;
        yPos = y;
        hPosRad = Math.toRadians(hDeg);
    }

    public double getDeltaX() {
        return deltaX;
    }

    public double getDeltaY() {
        return deltaY;
    }

    //returns heading on field in radians
    public double getDeltaHRad() {
        return deltaHRad;
    }

    //returns heading on field in degrees
    public double getDeltaHDeg() {
        return Math.toDegrees(deltaHRad);
    }

    public double getXPos() {
        return xPos;
    }

    public double getYPos() {
        return yPos;
    }

    public double getHPosRad() {
        return hPosRad;
    }

    public double getHPosDeg() {
        return Math.toDegrees(hPosRad);
    }

    //sets coordinate position to custom value
    public void setCoords(double x, double y, double hDeg) {
        xPos = x;
        yPos = y;
        hPosRad = hDeg*Math.PI/180;
    }

    //sets distance between encoders to custom length
    public void setEncoderDistance(double disLtoR, double disMidtoC) {
        distanceLtoR = disLtoR;
        centerForwardOffset = disMidtoC;
    }

    //updates function with new encoder position
    public void setEncoderPos(int eLeft, int eRight, int eCenter) {
        for (i = 2; i >= 0; i--)
            encodersLast[i] = encoders[i];

        encoders[0] = eLeft;
        encoders[1] = eRight;
        encoders[2] = eCenter;
 
        for (i = 2; i >= 0; i--)
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
