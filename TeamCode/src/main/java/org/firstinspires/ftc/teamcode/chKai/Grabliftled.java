package org.firstinspires.ftc.teamcode.chKai;
import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.robotcore.hardware.*;

public class Grabliftled {
    private Servo leftGrab, rightGrab;
    Led led = new Led();
    CDS cds = new CDS();
    Pablo pablo = new Pablo();
    Lift lift = new Lift();
    Grabber grabber = new Grabber();
    private boolean hasChanged = false;
    private boolean grabbable = false;
    private boolean grabbed = false;
    public void initializeHardware(HardwareMap hardwareMap){
        led.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
        pablo.initializeHardware(hardwareMap);
        lift.initializeHardware(hardwareMap);
        grabber.initializeHardware(hardwareMap);
    }
    public void autoGrab(Gamepad gamepad1){
        if (gamepad1.right_bumper && !hasChanged){
            grabbable = !grabbable;
            hasChanged = true;
        }else {
            hasChanged = false;
        }
        if (grabbable){
            if (cds.getDistance()<2){
                grabbed = true;
            }else{
                grabbed = false;
                led.setLed("yellow");
            }

        }else {
            led.teamColors();
            grabbed = false;
        }
        if (grabbed){
            grabber.grab(true,true);
            led.poleCenter();
        }else {
            grabber.grab(false,true);
        }
    }
    public boolean grabbed(){
        return grabbed;
    }
}
