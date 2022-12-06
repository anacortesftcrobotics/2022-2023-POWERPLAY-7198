package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LEDManager
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
        return ("red - "+ red+"\ngreen - "+green+"\nblue - "+blue+"\ndistance -"+dist);
    }
    public void pole(){
        double left = leftDistance.getDistance(DistanceUnit.CM);
        double right = rightDistance.getDistance(DistanceUnit.CM);
        if (left-right>3){
            if(left+right>6){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
            }else if(left+ right<3){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
            }
        }else if (left-right<-3){
            if(left+right>6){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            }else if(left+ right<3){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
            }
        }else{
            if(left+right>6){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            }else if(left+ right<3){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
            }else{
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }
    }
}
