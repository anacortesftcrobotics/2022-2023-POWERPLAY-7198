package org.firstinspires.ftc.teamcode.chKai;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class SensorSystem {
    private ColorSensor color;
    private DistanceSensor distance;

    public void setup(HardwareMap hardwareMap, String sensorName){
        color = hardwareMap.get(ColorSensor.class, "color");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

    }

    public int identify (){ //x=s 1 if red, 2 if green, 3 if blue, and 0 if nothing.
        int x=0;
        if(color.red() < 150 && color.green()< 150 && color.blue() < 150){
            x= 0;
        }else{
            if (color.red()>color.blue()&&color.red()> color.green()){
                x= 1;
            } else if (color.green()>color.blue()&&color.green()> color.red()){
                x= 2;
            }else if (color.blue()>color.red()&&color.blue()> color.green()){
                x= 1;
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
        return ("red - "+ red+"\ngreen - "+green+"\nblue - "+blue);
    }

}

