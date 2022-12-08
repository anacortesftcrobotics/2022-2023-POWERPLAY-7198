package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Led implements SubsystemManager
{
    public Led ()
    {

    }

    private RevBlinkinLedDriver led;

    public void initializeHardware(HardwareMap hardwareMap)
    {
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
}
