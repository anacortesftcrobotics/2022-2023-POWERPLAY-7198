package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.text.DecimalFormat;

public class PoseTracker extends Subsystem{

    String name = "POSE";

    final double CM_FROM_CENTER_TO_FRONT_ENCODER_AXIS = 3.3921;
    final double CM_FROM_CENTER_TO_BACK_ENCODER_AXIS = -16.7964;

    double leftWheelTurnRadius;
    double rightWheelTurnRadius;
    double backWheelTurnRadius;

    DecimalFormat outputFormat;

    Matrix poseByPose;
    Matrix poseByEncoders;
    Matrix localPose;
    Matrix encoderInputs;
    DcMotor leftEncoderPort;
    DcMotor rightEncoderPort;
    DcMotor backEncoderPort;
    BNO055IMU boschImu;
    State currentState;

    public PoseTracker(double leftWheelTurnRadius, double rightWheelTurnRadius, double backWheelTurnRadius) {
        this.leftWheelTurnRadius = leftWheelTurnRadius;
        this.rightWheelTurnRadius = rightWheelTurnRadius;
        this.backWheelTurnRadius = backWheelTurnRadius;

        outputFormat = new DecimalFormat("##0.0");

        poseByPose = new Matrix(3, 3)
                .initRow(0, new double[]{0, 0, backWheelTurnRadius})
                .initRow(1, new double[]{0, 0, 0.5 * (-leftWheelTurnRadius + rightWheelTurnRadius)})
                .initRow(2, new double[]{1.0 / (3 * backWheelTurnRadius), -1.0 / (3 * leftWheelTurnRadius) + 1 / (3 *rightWheelTurnRadius), 0});

        poseByEncoders = new Matrix(3, 3)
                .initRow(0, new double[]{0, 0, 1})
                .initRow(1, new double[]{0.5, 0.5, 0})
                .initRow(2, new double[]{1.0 / (3 * leftWheelTurnRadius), 1.0 / (3 * rightWheelTurnRadius), -1.0 / (3 * backWheelTurnRadius)});

        localPose = new Matrix(3, 1)
                .initRow(0, new double[]{0})
                .initRow(0, new double[]{0})
                .initRow(0, new double[]{0});

        encoderInputs = new Matrix(3, 1)
                .initRow(0, new double[]{0})
                .initRow(0, new double[]{0})
                .initRow(0, new double[]{0});
    }

    /**
     * Registers the hardware devices used by the subsystem.
     * The HardwareMap must belong to the currently running opMode
     *
     * @param hardwareMap the HardwareMap of the connected robot
     */
    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        leftEncoderPort = hardwareMap.get(DcMotor.class, "leftBack");
        rightEncoderPort = hardwareMap.get(DcMotor.class, "rightBack");
        backEncoderPort = hardwareMap.get(DcMotor.class, "rightFront");
        boschImu = hardwareMap.get(BNO055IMU.class, "imu");
        initImu();
    }

    /**
     * Checks the status of all devices required by the Subsystem and provides them as a string.
     * The format will be as follows:
     * <p>
     * <code>device: status[\n...]</code>
     *
     * @return the String of concatenated device statuses for this subsystem
     */
    @Override
    public String getHardwareStatus() {
        return "Left Encoder: " + leftEncoderPort.getConnectionInfo() +
                "\nRight Encoder: " + rightEncoderPort.getConnectionInfo() +
                "\nRear Encoder: " + backEncoderPort.getConnectionInfo();
    }

    /**
     * Updates the Subsystem operation based on user input
     *
     * @param gamepad1 should match TeleOp's gamepad1
     * @param gamepad2 should match TeleOp's gamepad2
     * @return the current status of the Subsytem
     */
    @Override
    public String update(Gamepad gamepad1, Gamepad gamepad2) {
        switch (currentState) {
            case UNINITIALIZED:
                return "Not initialized";
            case IDLE:
                currentState = State.NAIVE_ENCODERS;
                return "idle";
            case NAIVE_ENCODERS:
                return naiveEncoderPoseUpdate();
            case STOP:
                return "STOPPED";
            default:
                return "Bad State";
        }
    }

    /**
     *
     */
    @Override
    public void stop() {

    }

    /**
     * Initializes the IMU with appropriate parameters
     */
    private void initImu() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        boschImu.initialize(imuParameters);
    }

    /**
     * Updates localPose based on raw sensor data from all three encoder wheels.
     * @return the updated local pose as X Y Theta
     */
    private String naiveEncoderPoseUpdate() {
        encoderInputs.setRow(0, new double[]{leftEncoderPort.getCurrentPosition()});
        encoderInputs.setRow(1, new double[]{rightEncoderPort.getCurrentPosition()});
        encoderInputs.setRow(2, new double[]{backEncoderPort.getCurrentPosition()});

        localPose = Matrix.add(
                Matrix.multiply(poseByPose, localPose),
                Matrix.multiply(poseByEncoders, encoderInputs));
        return outputFormat.format(localPose.matrixArray[0][0]) + " " +
                outputFormat.format(localPose.matrixArray[1][0]) + " " +
                outputFormat.format(Math.toDegrees(localPose.matrixArray[2][0]));
    }

    private enum State {
        UNINITIALIZED,
        IDLE,
        NAIVE_ENCODERS,
        STOP
    }
}
