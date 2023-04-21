package org.firstinspires.ftc.teamcode.powerplay.odometry;

import org.firstinspires.ftc.teamcode.powerplay.Logger;

/**
 * This class provides odometry tracking for an FTC Robot
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Odo4 {

    private final double diagonal; //distance between the center of rotation and a forward encoder.
    private final double rotationalConstant;
    private final double backEncoderOffset;
    private final double distancePerTick;
    //kai: 0.001342233189
    //liam: 0.0013
    private Logger logger = Logger.getInstance();
    private double hRad = 0;

    /**
     * Class constructor for centered encoders. Uses custom encoder distances & dimensions.
     * Some encoder values may need to be multiplied by -1 when using getDeltaPos().
     *
     * @param distanceLtoR       the distance between left and right encoders.
     * @param backEncoderOffset  the distance between the center of rotation and the encoder. Should be (+) if in front.
     * @param frontEncoderOffset the distance between midpoint of the left & right encoders and center of rotation. Should be (+) if in back.
     * @param encoderDiameter    the radius of encoder wheels.
     * @param ticksPerRev        the encoder ticks per revolution.
     */
    public Odo4(double distanceLtoR, double frontEncoderOffset, double backEncoderOffset, double encoderDiameter, int ticksPerRev) {
        this.diagonal = Math.sqrt(frontEncoderOffset * frontEncoderOffset + distanceLtoR * distanceLtoR / 4.0);
        this.backEncoderOffset = backEncoderOffset;
        this.rotationalConstant = Math.atan(Math.cos(2 * frontEncoderOffset / distanceLtoR));
        this.distancePerTick = encoderDiameter * Math.PI / ticksPerRev;
    }

    /**
     * returns the object's new position relative to its last. Faces positive x-axis.
     *
     * @param leftEncoder   change in tick position of the left encoder, (+) when moving forward.
     * @param rightEncoder  change in tick position of the right encoder, (+) when moving forward.
     * @param centerEncoder change in tick position of the center encoder, (+) when strafing left.
     * @return
     */
    public Pose2D getRelativePose(int leftEncoder, int rightEncoder, int centerEncoder) {
        double left = distancePerTick * leftEncoder;
//        Logger.setLine1(Double.toString(left));
        double right = distancePerTick * rightEncoder;
//        Logger.setLine2(Double.toString(right));
        double center = distancePerTick * centerEncoder;
//        Logger.setLine3(Double.toString(center));

        double deltaHRad = (right - left) / 2.0 * rotationalConstant / diagonal;
//        Logger.setLine1(Double.toString(deltaHRad));
        double forward = (left + right) / 2.0;
        double strafe = center - deltaHRad * backEncoderOffset;

        hRad += deltaHRad;
        Logger.setLine1(Double.toString(Math.toDegrees(hRad)));
        Logger.setLine2(Double.toString(hRad));

        return new Pose2D(forward, strafe, deltaHRad);
    }

    public Pose2D getFieldPose(Pose2D lastPose, int leftEncoder, int rightEncoder, int centerEncoder) {
        return lastPose.move(getRelativePose(leftEncoder, rightEncoder, centerEncoder));
    }
}