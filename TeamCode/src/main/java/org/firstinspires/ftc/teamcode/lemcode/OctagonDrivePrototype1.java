package org.firstinspires.ftc.teamcode.lemcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
@Disabled
public class OctagonDrivePrototype1 extends OpMode
{
    //variables
    double D;
    double y;
    double x;
    double rx;
    double speedCoefficient = 0.25;
    double FRPower;
    double FLPower;
    double BRPower;
    double BLPower;
    boolean hasChanged0;
    boolean hasChanged1;
    boolean hasChanged2;
    double ms = 0;
    double lms = 0;
    double sms = 0;

    double FRMod = -1.0;
    double FLMod = 1.0;
    double BRMod = -1.0;
    double BLMod = 1.0;

    //motor initializations
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;

    //servo initializations

    //sensor initializations
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    private ColorSensor colorSensor1;

    //imu

    //camera

    //rumble effects
    Gamepad.RumbleEffect customRumbleEffect0;

    public void init()
    {
        //Drive Motors
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR = hardwareMap.get(DcMotor.class, "backRight");
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift mechanism

        //hand mechanism

        //sensors
        distanceSensor1 = hardwareMap.get(DistanceSensor.class,"distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class,"distanceSensor2");
        colorSensor1 = hardwareMap.get(ColorSensor.class,"colorSensor1");

        //imu

        //camera

        //rumble effects
        customRumbleEffect0 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 125)
                .build();

    }
    public void loop()
    {
        telemetry();
        gamePad1();
    }
    public void gamePad1()
    {
        //Basic Movement
        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        rx = -gamepad1.right_stick_x;


        double D1 = capAtTwelve(distanceSensor1.getDistance(DistanceUnit.INCH));
        double D2 = capAtTwelve(distanceSensor2.getDistance(DistanceUnit.INCH));

        //distance sensor turning test thingamabob
        /*
        if (D1 < D2)
        {
            rx = ((D2 - D1) / 12 * 0.5);
        }
        else if (D1 > D2)
        {
            rx = ((D1 - D2) / 12 * -0.5);
        }
        */

        D = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        x = deadZone(x);
        y = deadZone(y);
        rx = deadZone(rx);

        BLPower = (y - x + rx) / D;
        FLPower = (y + x + rx) / D;
        BRPower = (y + x - rx) / D;
        FRPower = (y - x - rx) / D;

        BLPower = (BLPower * BLMod * speedCoefficient);
        FLPower = (FLPower * FLMod * speedCoefficient);
        BRPower = (BRPower * BRMod * speedCoefficient);
        FRPower = (FRPower * FRMod * speedCoefficient);

        BL.setPower(BLPower);
        FL.setPower(FLPower);
        BR.setPower(BRPower);
        FR.setPower(FRPower);

        //Other stuff
        //motor reversing stuff
        if(!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y)
        {
            hasChanged0 = false;
        }
        if (gamepad1.a && !hasChanged0)
        {
            BRMod = -BRMod / Math.abs(BRMod);
            hasChanged0 = true;
        }
        if (gamepad1.b && !hasChanged0)
        {
            FRMod = -FRMod / Math.abs(FRMod);
            hasChanged0 = true;
        }
        if (gamepad1.x && !hasChanged0)
        {
            BLMod = BLMod / Math.abs(BLMod);
            hasChanged0 = true;
        }
        if (gamepad1.y && !hasChanged0)
        {
            FLMod = -FLMod / Math.abs(FLMod);
            hasChanged0 = true;
        }

        //speed coefficient
        if(!gamepad1.dpad_down && !gamepad1.dpad_up)
        {
            hasChanged1 = false;
        }
        if (gamepad1.dpad_down && !hasChanged1)
        {
            if (speedCoefficient > 0)
            {
                speedCoefficient = speedCoefficient - 0.25;
                gamepad1.runRumbleEffect(customRumbleEffect0);
            }
            hasChanged1 = true;
        }
        if (gamepad1.dpad_up && !hasChanged1)
        {
            if (speedCoefficient < 1)
            {
                speedCoefficient = speedCoefficient + 0.25;
                gamepad1.runRumbleEffect(customRumbleEffect0);
            }
            hasChanged1 = true;
        }
    }
    public void telemetry()
    {
        //rpm thing WIP
        if (sms == 0)
        {
            sms = System.currentTimeMillis();
        }
        lms = ms;
        ms = System.currentTimeMillis();
        telemetry.addData("time",ms - lms);
        telemetry.addData("time",((ms - sms) / 1000));

        //basic telemetry
        telemetry.addData("FL Direction",getDirection(FLMod));
        telemetry.addData("FR Direction",getDirection(FRMod));
        telemetry.addData("BL Direction",getDirection(BLMod));
        telemetry.addData("BR Direction",getDirection(BRMod));
        telemetry.addData("FLPower",FLPower);
        telemetry.addData("FRPower",FRPower);
        telemetry.addData("BLPower",BLPower);
        telemetry.addData("BRPower",BRPower);
        telemetry.addData("speed",100 * speedCoefficient + "%");
        telemetry.addData("1",distanceSensor1.getDistance(DistanceUnit.INCH));
        telemetry.addData("2",distanceSensor2.getDistance(DistanceUnit.INCH));
        telemetry.addData("1",capAtTwelve(distanceSensor1.getDistance(DistanceUnit.INCH)));
        telemetry.addData("2",capAtTwelve(distanceSensor2.getDistance(DistanceUnit.INCH)));
        telemetry.addData("red",colorSensor1.red());
        telemetry.addData("blue",colorSensor1.blue());
        telemetry.addData("green",colorSensor1.green());
        telemetry.addData("argb",colorSensor1.argb());
        telemetry.addData("t",1);
        telemetry.update();
    }
    //misc functions
    public double deadZone(double input)
    {
        if (Math.abs(input) < 0.1)
        {
            return  0;
        }
        else
        {
            return input;
        }
    }
    public String getDirection(double input)
    {
        if (input > 0)
        {
            return ("forward");
        }
        else if (input < 0)
        {
            return ("backward");
        }
        else
        {
            return ("something is wrong");
        }
    }
    public double capAtTwelve(double input)
    {
        if (input > 12)
        {
            return (12);
        }
        else if (input <= 12)
        {
            return (input);
        }
        else
        {
            return (0);
        }

    }
}
