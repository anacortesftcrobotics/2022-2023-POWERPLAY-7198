package org.firstinspires.ftc.teamcode.mentorcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.mentorcode.XyhVector;

public class OdometryWheels {
    public DcMotor encoderLeft = null;
    public DcMotor encoderRight = null;
    public DcMotor encoderAux = null;

    private final static double L = 20.12;      // distance between encoder 1 and encoder 2 in cm
    private final static double B = 11.5;       // distance between the midpoint of encoder 1 and encoder 2 and 3 in cm
    private final static double R = 3.0;        // wheel radius in cm
    private final static double N = 8192;       // encoder ticks per revolution REV encoder
    private final static double cm_per_tick = 2.0 * Math.PI * R / N;

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    public int oldRightPosition = 0;
    public int oldLeftPosition = 0;
    public int oldAuxPosition = 0;

    public XyhVector START_POS = null;
    public XyhVector pos;

    /* Constructor

     */
    public OdometryWheels(DcMotor xLeft, DcMotor xRight, DcMotor y) {
        encoderLeft = xLeft;
        encoderRight = xRight;
        encoderAux = y;

        START_POS = new XyhVector(213,102,Math.toRadians(-174));
        pos = new XyhVector(213,102,Math.toRadians(-174));
    }

    public void pose() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = -encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        double dtheta = cm_per_tick * (dn2 - dn1) / L;
        double dx = cm_per_tick * (dn1 + dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L);

        double theta = pos.h + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }
}
