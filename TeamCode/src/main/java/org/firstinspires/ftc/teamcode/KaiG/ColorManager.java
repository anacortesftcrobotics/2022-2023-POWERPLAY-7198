package org.firstinspires.ftc.teamcode.KaiG;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorManager
{

    private RevBlinkinLedDriver led;
    private ColorRangeSensor color;
    private DistanceSensor leftDistance, rightDistance;

    public void setUp(HardwareMap hardwareMap)
    {
        color = hardwareMap.get(ColorRangeSensor.class,"color");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
    }
    public void teamColors()
    {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
    }
    public void signal()
    {
        if (identify()==1){
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else if (identify()==2){
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (identify()==3) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (identify()==0) {
            teamColors();
        } else if (identify()==4){
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }
    public int identify (){ //x=s 1 if red, 2 if green, 3 if blue, 4 if white and 0 if nothing.
        int x=0;
        if(color.getDistance(DistanceUnit.CM)>2){
            x= 0;
        }else{
            if (color.red() > color.blue() && color.red() > color.green()){
                x = 1;
            } else if (color.green() > color.blue() && color.green() > color.red()){
                x = 2;
            }else if (color.blue() > color.red() && color.blue() > color.green()){
                if (color.blue()-color.green()>(color.red()+color.green()+color.blue())/5){
                    x = 3;
                }else{
                    x = 4;
                }
            }
        }
        return x;
    }
    public int identify2(){//returns 1 if cyan, 2 if yellow, 3 if magenta, and 0 if nothing.
        int x=0;
        if(color.red() < 150 && color.green()< 150 && color.blue() < 150){
            x= 0;
        }else{
            if (color.blue()>color.red()&&color.green()> color.red()){
                x= 1;
            } else if (color.red()>color.blue()&&color.green()> color.blue()){
                x= 2;
            }else if (color.red()>color.green()&&color.blue()> color.green()){
                x= 1;
            }
        }
        return x;
    }
    public String colorTelemetry(){
        int red = color.red();
        int green = color.green();
        int blue = color.blue();
        double dist = color.getDistance(DistanceUnit.CM);
        return ("red - "+ red+"\ngreen - "+green+"\nblue - "+blue);
    }
    public void pole(){
        /** in ideal position:
         * left-20.9
         * right- 17.7
         * add- 38.6
         * diff- 3.2
         */
        double left = leftDistance.getDistance(DistanceUnit.CM);
        double right = rightDistance.getDistance(DistanceUnit.CM);
        double add = left + right;
        double diff = left - right;
        if (left>30 && right>30){//if the cone is far away, do team colors
            teamColors();
        }else {
            if (diff > 5.2) {//if its too far in one direction, its red
                if (add > 40) {//if its too far away, breath
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                } else if (add < 35) {//if its too close, heartbeat
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                }else{//if its the right distance, solid
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
            } else if (diff < 1.2) {//if its too far in the other direction, its blue
                if (add > 40) {//if its too far away, breath
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                } else if (add < 35) {//if its too close, heartbeat
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                }else{//if its the right distance, solid
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
            } else {//if its centered side to side, its gray
                if (add > 40) {//if its too far away, breath
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
                } else if (add < 35) {//if its too close, heartbeat
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
                } else {//if its the right distance, solid
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
                }
            }
        }
    }
    public String distanceTelemetry(){
        double left = leftDistance.getDistance(DistanceUnit.CM);
        double right = rightDistance.getDistance(DistanceUnit.CM);
        double add = left + right;
        double diff = left - right;
        return ("left- "+left+"\nright- "+right+"\nadd- "+add+"\ndiff- "+diff);
    }


}
