package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Led implements SubsystemManager
{
    public Led ()
    {

    }

    private RevBlinkinLedDriver led;
    Pablo pablo = new Pablo();
    CDS cds = new CDS();
    public void initializeHardware(HardwareMap hardwareMap)
    {
        cds.initializeHardware(hardwareMap);
        pablo.initializeHardware(hardwareMap);
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void setLed(String in)
    {
        int num = 22;
        String[] array = {"hot pink", "dark red", "red", "red orange", "orange", "gold", "yellow", "lawn green", "lime", "dark green", "green", "blue green", "aqua", "sky blue", "dark blue", "blue", "blue violet", "violet", "white", "gray", "dark gray", "black"};
        String input = in.toLowerCase();
        for (int i = 0; i < 22; i++) {
            if (input.equals(array[i]))
                num = i;
        }
        switch (num) {
            case 0:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                break;
            case 1:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                break;
            case 2:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case 3:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                break;
            case 4:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                break;
            case 5:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                break;
            case 6:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case 7:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
                break;
            case 8:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
                break;
            case 9:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                break;
            case 10:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case 11:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                break;
            case 12:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                break;
            case 13:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                break;
            case 14:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                break;
            case 15:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            case 16:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                break;
            case 17:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
            case 18:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
            case 19:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
                break;
            case 20:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
                break;
            case 21:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;
            default:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;
        }
    }
    /**
    *Gives LED feedback to drivers on how to center the cone over the pole
    *questionable if its working yet
    *if the pole is very far away, do team colors
    *if its too far right and too far away, blue 
    *if its too far left and too far away, yellow
    *if its centered and too far away, green
    *if its too close, red
    *if its too far right and the right distance, purple
    *if its too far left and right distance, orange
    *if its the right distance and centered, white
    *@author Kai G
    */
    public void poleCenter(){
        double left = pablo.getLeftDistance();
        double right = pablo.getRightDistance();
        double add = left + right;//distance fore/back
        double diff = left - right;//distance side/side
        double idealAdd = 44;
        double idealDiff = 5.2;
        double addTolerance = 3;
        double diffTolerance = 6;
        if (left>30 && right>30){//if the pole is very far away, do team colors
            teamColors();
        }else {
            if (add > idealAdd + addTolerance) {//if its too far away...
                if (diff < idealDiff - diffTolerance) {//if its too far right and too far away, blue
                    setLed("blue");

                }else if (diff > idealDiff + diffTolerance){//if its too far left and too far away, yellow
                    setLed("orange");
                }else{//if its centered and too far away, green
                    setLed("green");
                }
            } else if (add < idealAdd - addTolerance) {//if its too close, red
                setLed("red");

            } else {//if its the right distance...
                if (diff < idealDiff - diffTolerance) {//if its too far right and the right distance, purple
                    setLed("violet");
                } else if (diff > idealDiff + diffTolerance) {//if its too far left and right distance, orange
                    setLed("orange");
                } else {//if its the right distance and centered, white
                    setLed("white");
                }
            }
        }
    }

    /**
    *uses the identify() method in CDS to match the signal color with the LEDs
    *@author Kai G
    */
    public void signal()
    {
        if (cds.identify()==1){
            setLed("red");
        }else if (cds.identify()==2){
            setLed("green");
        } else if (cds.identify()==3) {
            setLed("blue");
        } else if (cds.identify()==0) {
            teamColors();
        } else if (cds.identify()==4){
            setLed("white");
        }
    }
    /**
    *Sets the LEDs to our team colors
    *should be purple an white waves
    *@author Kai G
    */
    public void teamColors(){
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

}
