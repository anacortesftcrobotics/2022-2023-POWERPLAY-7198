package org.firstinspires.ftc.teamcode.mentorcode;

import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.teamcode.mentorcode.XyhVector;
/**
 * OdometryWheels class used to keep track of robot movement on field using
 * odometry wheels.
 * @author Coach Jenkins
 */

public class OdometryWheels {
    /**
     * private properties for each limit switch
     */
    public DcMotor encoderXLeft, encoderXRight, encoderCenter;

    // set direction of left and right
    private int left_dir = 1;
    private int right_dir = -1;

    // distance between left and right parallel encoders (in cm)
    private double trackwidth;
    // distance between the midpoint of encoder 1 and encoder 2 and 3 in cm
    private double forwardOffset;
    private double diameter;    // odometry wheel diameter in cm
    //private double ticksPerRevolution;       // encoder ticks per revolution REV encoder
    public double cm_per_tick;

    public int prev_left_encoder_pos = 0;
    public int prev_right_encoder_pos = 0;
    public int prev_center_encoder_pos = 0;

    public int left_encoder_pos = 0;
    public int right_encoder_pos = 0;
    public int center_encoder_pos = 0;

    public double leftX_cm = 0.0;
    public double rightX_cm = 0.0;
    public double y_cm = 0.0;

    // public XyhVector pos;

    /**
     * Constructor for OdometryWheels
     *
     * @param xLeft     left x odometry motor
     * @param xRight    right x odometry motor
     * @param center    y axis odometry motor
     * @param trackwidth    distance in cm between left and right odometry wheels (center to center)
     * @param forwardOffset distance in cm robot center of rotation to perpendicular (y) odometry wheel
     *                      back of robot is negative offset, front of robot is positive offset
     * @param wheeldiameter diameter in cm of odometry wheel
     * @param ticksPerRevolution how many ticks odometry encoder has per one revolution of wheel
     */
    public OdometryWheels(DcMotor xLeft, DcMotor xRight, DcMotor center,
                          double trackwidth, double forwardOffset,
                          double wheeldiameter, int ticksPerRevolution) {

        this.encoderXLeft = xLeft;
        this.encoderXRight = xRight;
        this.encoderCenter = center;
        this.trackwidth = trackwidth;
        this.forwardOffset = forwardOffset;
        this.diameter = wheeldiameter;
        this.cm_per_tick = (Math.PI * wheeldiameter) / ticksPerRevolution;
        this.leftX_cm = 0.0;
        this.rightX_cm = 0.0;
        this.y_cm = 0.0;

        // declare first value of each position vector
        // should get set to starting position of robot relative to field?
        // pos = new XyhVector(0,0,Math.toRadians(0), cm_per_tick);
    }

    public void setCurrentPosition() {
        left_encoder_pos = encoderXLeft.getCurrentPosition() * left_dir;
        right_encoder_pos = encoderXRight.getCurrentPosition() * right_dir;
        center_encoder_pos = encoderCenter.getCurrentPosition();
    }

    public void setCurrentCM() {
        leftX_cm = left_encoder_pos * cm_per_tick;
        rightX_cm = right_encoder_pos * cm_per_tick;
        y_cm = center_encoder_pos * cm_per_tick;
    }
    /**
     * pose() sets the most recent version of location of robot on field
     *
     */
    public void pose() {
        setCurrentPosition();
        setCurrentCM();

        int delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        int delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        int delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        double phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth;
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        double delta_perp_pos = delta_center_encoder_pos - forwardOffset * phi;

        //double delta_x = delta_middle_pos * pos.hcos() - delta_perp_pos * pos.hsin();
        //double delta_y = delta_middle_pos * pos.hsin() + delta_perp_pos * pos.hcos();

        //pos.setX(pos.getX() + delta_x);
        //pos.setY(pos.getY() + delta_y);
        //pos.h += phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;
    }
}