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
    private  boolean hasGrabbed = false;
    private boolean grabbable = false;
    private boolean grabbed = false;
    private  int i = 0;
    private String s = "notGrabbable";
    public void initializeHardware(HardwareMap hardwareMap){
        led.initializeHardware(hardwareMap);
        cds.initializeHardware(hardwareMap);
        pablo.initializeHardware(hardwareMap);
        lift.initializeHardware(hardwareMap);
        grabber.initializeHardware(hardwareMap);
        grabber.grab(false, true);
    }
    public void autoGrab(Gamepad gamepad1){

        if (gamepad1.right_bumper && !hasChanged){
            grabbable = !grabbable;
        }
        if (gamepad1.right_bumper){
            hasChanged = true;
        }else {
            hasChanged = false;
        }
        if (grabbable){
            if (cds.getDistance() < 1){
                grabbed = true;
                grabber.grab(true,false);
            }/*else if (cds.getDistance() > 0.8){
                grabbed = false;
                grabber.grab(false,false);
            }*/

        }else {
            grabbed = false;
            grabber.grab(false,false);
        }

        if (!grabbable){
            led.teamColors();
            s = "notGrabbable";
        } else if (grabbable) {
            if (grabbed){
                led.poleCenter();
                s="grabbed";
            } else if (!grabbed) {
                led.setLed("yellow");
                s="grabbable not grabbed";
            }
        }

        if (grabbed){
            i ++;
        }else{
            i =0;
        }

        if (grabbed && !hasGrabbed && i > 60){
            lift.liftSet(2.0);
        }
        if (!grabbed){
            lift.liftSet(0.0);
        }
        if (grabbed){
            hasGrabbed = true;
        }else{
            hasGrabbed = false;
        }
    }
    public String state(){
        return (String) s+i;

    }
    public boolean grabbed(){
        return grabbed;
    }
}
