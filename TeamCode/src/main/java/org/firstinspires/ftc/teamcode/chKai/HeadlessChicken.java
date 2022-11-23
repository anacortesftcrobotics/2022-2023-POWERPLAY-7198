package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "HeadlessChicken")
public class HeadlessChicken extends LinearOpMode
{
    BNO055IMU imu;
    Orientation angles;

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    //private ColorSensor colorDown;
    //private Servo scythe;
    private double powerSetting = 1;
    private double settingPower = 1;
    private boolean whackerOpen = false;
    private boolean hasChanged = false;
    private boolean hasChanged2 = false;
    private boolean hasChanged3 = false;
    private boolean hasChanged4 = false;
    private double servoSet = 1;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()
    {
        //scythe = hardwareMap.get(Servo.class, "scythe");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit =BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu=hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive())
        {
            // Put run blocks here.
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            //colorDown = hardwareMap.get(ColorSensor.class, "colorDown");

            while (opModeIsActive())
            {

                double x1=(gamepad1.left_stick_x*Math.sin(getAngle())); //sideways movement for FL and BR
                double x2=(gamepad1.left_stick_x*Math.cos(getAngle())); //sideways movement for FR and BL

                double y1=(gamepad1.left_stick_y*Math.cos(getAngle())); //foreward movement for FL and BR
                double y2=-(gamepad1.left_stick_y*Math.sin(getAngle())); //foreward movement for FR and BL

                double FL =(x1+y1+gamepad1.right_stick_x);

                double FR =(x2+y2-gamepad1.right_stick_x);

                double BL =(x2+y2+gamepad1.right_stick_x);

                double BR =(x1+y1-gamepad1.right_stick_x);

                double maxPower=Math.max(Math.max(FL,FR),Math.max(BL,BR));
                if (maxPower > 1){
                    FL=FL/maxPower;
                    FR=FR/maxPower;
                    BL=BL/maxPower;
                    BR=BR/maxPower;
                }

                leftFront.setPower(FL);

                rightFront.setPower(FR);

                leftBack.setPower(BL);

                rightBack.setPower(BR);

                telemetry.addData("FL", leftFront.getPower() );
                telemetry.addData("FR", rightFront.getPower() );
                telemetry.addData("BL", leftBack.getPower() );
                telemetry.addData("BR", rightBack.getPower() );
                telemetry.update();

            }
        }
    }
    public double getAngle(){ //gets angle in radian form. 0* is 90* to the right of the starting orientation, and value adds when turning counterclockwise.
        float angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        float angle2 = 0;
        if (angle1<0){
            angle2=angle1+450;
        }else if (angle1>=0){
            angle2=angle1+90;
        }
        telemetry.addData("angle",angle2 );
        double radians=Math.toRadians(angle2);
        return radians;
    }
}
