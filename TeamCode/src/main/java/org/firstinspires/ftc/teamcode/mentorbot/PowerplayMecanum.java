package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;

public class PowerplayMecanum extends OmnidirectionalChassis {

    String name = "CHAS";
    State currentState;

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    String telemetry;

    public PowerplayMecanum() {
        currentState = State.UNINITIALIZED;
    }

    /**
     * Sets the power for every motor on a 4 motor chassis.
     * @param powers the FourPower set of all motor powers
     */
    public void setMotorPowers(FourPower powers) {
        frontLeft.setPower(powers.frontLeft);
        frontRight.setPower(powers.frontRight);
        backLeft.setPower(powers.backLeft);
        backRight.setPower(powers.backRight);
    }


    /**
     * Sets the power of the motors on the chassis based on dimensioned normal vectors.
     * Assumes that the positive x-axis of the robot is forward pointing.
     *
     * @param x the x vector size
     * @param r the rotation vector size
     */
    @Override
    public void setPowerXR(double x, double r) {
        FourPower powers = new FourPower(x + r, x - r,
                                         x + r, x - r);
        powers.normalize();
        setMotorPowers(powers);
    }

    /**
     * Sets the distance the motors should travel automatically based on dimensioned vectors
     * The size of vectors x and y are in this Chassis' current {@link DistanceUnit}, and r is in this Chassis'
     * current {@link AngleUnit}
     *
     * @param x the x vector size
     * @param r the rotation vector size
     */
    @Override
    public void moveDistanceXR(double x, double r) {

    }

    /**
     * Sets the power of the motors on the chassis based on dimensioned normal vectors.
     * Assumes that the positive x-axis of the robot is forward pointing.
     *
     * @param x the x vector size
     * @param y the y vector size
     * @param r the rotation vector size
     */
    @Override
    public void setPowerXYR(double x, double y, double r) {
        FourPower powers = new FourPower(x + r + y, x - r - y,
                                         x + r - y, x - r + y);
        powers.normalize();
        setMotorPowers(powers);
    }

    /**
     * Sets the distance the motors should travel automatically based on dimensioned vectors
     * The size of vectors x and y are in this Chassis' current {@link DistanceUnit}, and r is in this Chassis'
     * current {@link AngleUnit}
     *
     * @param x the x vector size
     * @param y the y vector size
     * @param r the rotation vector size
     */
    @Override
    public void moveDistanceXYR(double x, double y, double r) {

    }

    /**
     * Registers the hardware devices used by the subsystem.
     * The HardwareMap must belong to the currently running opMode
     *
     * @param hardwareMap the HardwareMap of the connected robot
     */
    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.tryGet(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.tryGet(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.tryGet(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.tryGet(DcMotorEx.class, "backRight");
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

        return "FrontLeft: " + frontLeft.getConnectionInfo() +
               "FrontRight: " + frontRight.getConnectionInfo() +
               "BackLeft: " + backLeft.getConnectionInfo() +
               "BackRight: " + backRight.getConnectionInfo();
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
        switch(currentState) {
            case UNINITIALIZED:
                return "Not initialized";
            case INACTIVE:
                if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                    currentState = State.MANUAL;
                    update(gamepad1, gamepad2);
                }
                return "Inactive";
            case MANUAL:
                setPowerXYR(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
                return "Manual inputs";
            case AUTOMATIC:
                // Check for automatic or cancel request
                return "CURRENT INSTRUCTION STATUS";
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
        frontLeft.setMotorDisable();
        frontRight.setMotorDisable();
        backLeft.setMotorDisable();
        backRight.setMotorDisable();
        frontLeft = null;
        frontRight = null;
        backLeft = null;
        backRight = null;
        currentState = State.STOP;
    }

    private enum State {
        UNINITIALIZED,
        INACTIVE,
        AUTOMATIC,
        MANUAL,
        STOP
    }
}
