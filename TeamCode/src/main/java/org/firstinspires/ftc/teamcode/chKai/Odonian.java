public class Odonian {
    DcMotor leftFront, rightFront, leftBack, rightBack;
     //imu
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Gyro gyro = new Gyro();

    private double x, y;
    private double lastFL, lastFR, lastBL, lastBR;

    public void initializeHardware(HardwareMap hardwareMap){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastFL = leftFront.getCurrentPosition();
        lastFR = rightFront.getCurrentPosition();
        lastBL = leftBack.getCurrentPosition();
        lastBR = rightBack.getCurrentPosition();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);

        gyro.initializeHardware(hardwareMap);
    }

    public void check(){
        double fl = leftFront.getCurrentPosition() - lastFL;
        double fr = rightFront.getCurrentPosition() - lastFR;
        double bl = leftBack.getCurrentPosition() - lastBL;
        double br = rightBack.getCurrentPosition() - lastBR;
        lastFL = leftFront.getCurrentPosition();
        lastFR = rightFront.getCurrentPosition();
        lastBL = leftBack.getCurrentPosition();
        lastBR = rightBack.getCurrentPosition();

        x += BistroMath.FLBRx(imu) * (fl + br);
        x += BistroMath.FRBLx(imu) * (fr + bl);

        y += BistroMath.FLBRy(imu) * (fl + br);
        y += BistroMath.FRBLy(imu) * (fr + bl);

        gyro.updateHeading();
    }

    public double getX(){return x;}

    public double getY(){return y;}

    public double getR(){return gyro.getHeading();}
}    