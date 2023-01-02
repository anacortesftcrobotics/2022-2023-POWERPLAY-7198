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
    /**
    *automatically grabs and lifts cone when near enough
    *@param button- the button you want to use for basic grabbing
    *to use:
    *press button. the led should turn yellow.
    *drive close to the cone. when the cone is close enough, the grabber will automatically close
    *the led should switch to poleCenter (which means red if its down), and after half a second or so the lift will raise to 2 inches. 
     */
    public void autoGrab(Boolean button){

        if (button && !hasChanged){
            grabbable = !grabbable;
        }
        if (button){
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
    /**
    *Gets the current state of autoGrab for telemetry
    *@return "notGrabbable," "grabbed," or "grabbable not grabbed" depending on the state. also shows i, the iterator that controls the delay between grab and lift.
     */
    public String state(){
        return (String) s+i;

    }
    /**
    *tells you wether or not the grabber is actually closed 
    *@return true if the grabber is closed, else false 
    *use in robot or opMode to control lift stuff
    */
    public boolean grabbed(){
        return grabbed;
    }
}
