package org.firstinspires.ftc.teamcode.chKai;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.powerplay.*;
import com.qualcomm.robotcore.hardware.*;
/**
*this class was made because the stuff inside it didnt really fit inside grabber, lift, or Led.
*autoGrab is the only real thing in here right now
*@author Kai G
 */
public class Grabliftled {
    private Servo leftGrab, rightGrab;
    Led led = new Led();
    CDS cds = new CDS();
    Pablo pablo = new Pablo();
    Lift lift = new Lift();
    Grabber grabber = new Grabber();
    private boolean hasChanged = false;
    private  boolean hasGrabbed = false;
    private  boolean grabbedUp = false;
    private boolean grabbable = false;
    private boolean grabbed = false;
    private  int i = 0;
    private String s = "notGrabbable";
    ElapsedTime elapsedTime = new ElapsedTime();
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
    *@param button- the button you want to use for basic grabbing. Needs to be dBounced
    *to use:
    *press button. the led should turn yellow.
    *drive close to the cone. when the cone is close enough, the grabber will automatically close
    *the led should switch to poleCenter (which means red if its down), and after half a second or so the lift will raise to 2 inches. 
    *press button again to release
     */
    public void autoGrab(Boolean button){
        if (button){
            grabbable = !grabbable;
        }

        if (grabbable){
            if (cds.getDistance() < 1){
                grabbed = true;
                grabber.grab(true,false);
            }
        }else {
            grabbed = false;
            grabber.grab(false,false);
        }
        if(!grabbable){
            led.teamColors();
            s = "not grabbable";
        }else if(grabbable && !grabbed){
            led.setLed("yellow");
            s = "grabbable not grabbed";
        } else if (grabbable && grabbed) {
            led.poleCenter();
            s = "grabbed";
        }

        if (grabbed && elapsedTime.milliseconds() > 500){
            elapsedTime.reset();
            grabbedUp = true;
        }


    }
    /**
    *Gets the current state of autoGrab for telemetry
    *@return "notGrabbable," "grabbed," or "grabbable not grabbed" depending on the state.
     */
    public String state(){
        return (String) s;

    }
    /**
    *tells you wether or not the grabber is actually closed 
    *@return true if the grabber is closed, else false 
    *use in robot or opMode to control lift stuff
    */
    public boolean grabbed(){
        return grabbed;
    }
    public boolean isGrabbedUp(){
        return grabbedUp;
    }

}
