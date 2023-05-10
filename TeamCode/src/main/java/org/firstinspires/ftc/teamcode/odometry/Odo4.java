package org.firstinspires.ftc.teamcode.odometry;

/**
 * This class provides odometry tracking for an FTC Robot
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Odo4 {

    private final double diagonal; //distance between the center of rotation and a forward encoder.
    private final double rotationalConstant;
            // rotationalConstant is a constant which when multiplied with forward encoder values gives an approximated
            // distance traveled along an arc with the focus as the center of rotation. Value assumes movement is
            // purely rotational and non-rotational movement is canceled out by other encoders.
    private final double centerEncoderOffset;
    private final double distancePerTick;

    /**
     * Class constructor using custom encoder distances & dimensions.
     *
     * @param distanceLtoR          the distance between the left and right encoders.
     * @param centerEncoderOffset   the distance the center encoder is behind the center of rotation.
     * @param frontEncodersOffset   the distance the left & right encoders are forward of the center of rotation.
     * @param encoderDiameter       the radius of the encoder wheels.
     * @param ticksPerRev           the encoder's ticks per revolution.
     */
    public Odo4(double distanceLtoR, double frontEncodersOffset, double centerEncoderOffset,
                double encoderDiameter, int ticksPerRev) {
        this.diagonal = Math.sqrt(frontEncodersOffset * frontEncodersOffset + distanceLtoR * distanceLtoR / 4.0);
        this.centerEncoderOffset = centerEncoderOffset;
        this.rotationalConstant = 1.0 / Math.cos(Math.atan(frontEncodersOffset / distanceLtoR * 2.0));
        this.distancePerTick = encoderDiameter * Math.PI / ticksPerRev;
    }

    /**
     * Returns the object's change in position based on the change in encoder position.
     *
     * @param leftEncoder   change in tick position of the left encoder, (+) when moving forward.
     * @param rightEncoder  change in tick position of the right encoder, (+) when moving forward.
     * @param centerEncoder change in tick position of the center encoder, (+) when strafing right.
     * @return              the object's pose relative to last position, the origin representing the last position.
     *                          forward movement is (+) in the y-axis, right strafing is (+) in the x-axis, and
     *                          heading is (+) when turning counterclockwise.
     */
    public Pose2D getMovement(int leftEncoder, int rightEncoder, int centerEncoder) {
        double left = distancePerTick * leftEncoder;
        double right = distancePerTick * rightEncoder;
        double center = distancePerTick * centerEncoder;

        double deltaHRad = (right - left) * rotationalConstant * 0.5 / diagonal;
        double forward = (left + right) / 2.0;
        double strafe = center - deltaHRad * centerEncoderOffset;

        return new Pose2D(strafe, forward, deltaHRad);
    }

    /**
     * Returns the object's position on the field based on the change in encoder position and the last field position.
     *
     * @param lastPose      Pose2D representing the object's position before the object changed position.
     * @param leftEncoder   change in tick position of the left encoder, (+) when moving forward.
     * @param rightEncoder  change in tick position of the right encoder, (+) when moving forward.
     * @param centerEncoder change in tick position of the center encoder, (+) when strafing right.
     * @return              the object's pose relative to last position, the origin representing the last position.
     *                          forward movement is (+) in the y-axis, right strafing is (+) in the x-axis, and
     *                          heading is (+) when turning counterclockwise.
     */
    public Pose2D getPose(Pose2D lastPose, int leftEncoder, int rightEncoder, int centerEncoder) {
        return lastPose.move(getMovement(leftEncoder, rightEncoder, centerEncoder));
    }
}