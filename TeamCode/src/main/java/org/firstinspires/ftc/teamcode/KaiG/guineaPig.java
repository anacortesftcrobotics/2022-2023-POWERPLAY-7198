package org.firstinspires.ftc.teamcode.KaiG;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.powerplay.*;
import org.firstinspires.ftc.teamcode.powerplay.PP2.*;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.powerplay.PP2.Chassis2;


@TeleOp(name = "Guinea Pig")
public class guineaPig extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private Servo rightGrab;
    private Servo leftGrab;
    private boolean hasTouched;
    private  double xChange;
    private double yChange;

    private ColorSensor color;


    private RevBlinkinLedDriver led;


    private boolean stickActive = false;

    private boolean hasChanged2;
    private double servoSet;
    private double lastSpeed;
    private NeedForSpeed s = new NeedForSpeed();
    private Odo2 f = new Odo2();

    int Xdisplay = 1;
    int Ydisplay = 1;
    boolean hasTouched2 = false;
    boolean hasTouched3 = false;
    int currentX;
    int currentY;
    //JFrame frame = new JFrame("I hope this Works");
    Controller controller = new Controller();

    String[][] bloop = new String[][] {
            {"i", "i", "i", "i"},
            {"i", "i", "i", "i"},
            {"i", "i", "i", "i"},
            {"i", "i", "i", "i"}
    };

    @Override
    public void runOpMode() {
        
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        color = hardwareMap.get(ColorSensor.class,"color");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, " rightGrab");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        ElapsedTime time = new ElapsedTime();
        Led led = new Led();
        CDS cds = new CDS();
        Lift lift = new Lift();
        Grabber grabber = new Grabber();
        grabber.initializeHardware(hardwareMap );
        led.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
        lift.initializeHardware(hardwareMap);
        s.initializeHardware(hardwareMap);
        f.initializeHardware(hardwareMap);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double x = 0;
                double y = 0;
                if (gamepad1.touchpad_finger_1 && !hasTouched){
                    xChange = gamepad1.touchpad_finger_1_x;
                    yChange = gamepad1.touchpad_finger_1_y;
                }
                hasTouched = gamepad1.touchpad_finger_1;
                if (gamepad1.touchpad_finger_1 && hasTouched){
                    x = gamepad1.touchpad_finger_1_x - xChange;
                    y = gamepad1.touchpad_finger_1_y - yChange;
                }
                if (!gamepad1.touchpad_finger_1){
                    x = 0;
                    y = 0;
                }
                y = -y;
                leftFront.setPower((y - x - gamepad1.right_stick_x));
                leftBack.setPower((y + x - gamepad1.right_stick_x));
                rightFront.setPower((y + x + gamepad1.right_stick_x));
                rightBack.setPower((y - x + gamepad1.right_stick_x));
                if(controller.button(7)){
                    currentX--;
                } else if (controller.button(4)) {
                    currentX++;
                }else if (controller.button(6)) {
                    currentY--;
                }else if (controller.button(5)) {
                    currentY++;
                }
                if(currentX<0){
                    currentX =0;
                } else if (currentX>3) {
                    currentX = 3;
                }
                if(currentY < 0){
                    currentY = 0;
                } else if (currentY>3) {
                    currentY = 3;
                }

                set();
                telemetry.addData("y", f.getY());
                telemetry.addData("x", f.getX());
                telemetry.addData("currentX", currentX);
                telemetry.addData("currentY", currentY);
                telemetry.addLine(bloop[0][0]+" "+bloop[0][1]+" "+bloop[0][2]+" "+bloop[0][3]);
                telemetry.addLine(bloop[1][0]+" "+bloop[1][1]+" "+bloop[1][2]+" "+bloop[1][3]);
                telemetry.addLine(bloop[2][0]+" "+bloop[2][1]+" "+bloop[2][2]+" "+bloop[2][3]);
                telemetry.addLine(bloop[3][0]+" "+bloop[3][1]+" "+bloop[3][2]+" "+bloop[3][3]);
                //touch(gamepad1);
                /*telemetry.addData("XX", Xdisplay);
                telemetry.addData("YY", Ydisplay);*/
                telemetry.update();
            }


        }

    }
    public void set(){//true for up, false for down
        for(int i = 0; i <= 3; i++){
            for (int j = 0; j <= 3; j++){
                if (i == currentX && j == currentY){
                    bloop[i][j]="x";
                }else{
                    bloop[i][j] = "o";

                }
            }
        }
    }

    public void touch (Gamepad gamepad1){
        float firstTouchY=0;
        float firstTouchX=0;
        if (gamepad1.touchpad_finger_1 && !hasTouched2){
            firstTouchY = gamepad1.touchpad_finger_1_y;
            firstTouchX = gamepad1.touchpad_finger_1_x;
        }
        if (gamepad1.touchpad_finger_1 && !hasTouched3){
            if(gamepad1.touchpad_finger_1_x - firstTouchX > 0.5){
                Xdisplay ++;
                hasTouched3 = true;
            } else if (gamepad1.touchpad_finger_1_x - firstTouchX < -0.5) {
                Xdisplay --;
                hasTouched3 = true;
            } else if (gamepad1.touchpad_finger_1_y - firstTouchY > 0.5) {
                Ydisplay ++;
                hasTouched3 = true;
            } else if (gamepad1.touchpad_finger_1_y - firstTouchY < -0.5) {
                Ydisplay --;
                hasTouched3 = true;
            }
            if (!gamepad1.touchpad_finger_1){
                hasTouched3 = false;
            }
        }
        hasTouched2 = gamepad1.touchpad_finger_1;
    }
    public double spedometer(){

        double t = Math.max(s.getSpeedY(1),lastSpeed);
        lastSpeed = t;
        return t;
    }
}