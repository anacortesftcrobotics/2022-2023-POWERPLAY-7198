package org.firstinspires.ftc.teamcode.mentorcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * LimitSwitch class used for every limit switch setup on DigitalChannel
 * @author Coach Jenkins
 */
public class LimitSwitch {
    /**
     * private properties for each limit switch
     */
    public final String name;
    private final DigitalChannel limitSwitch;
    // All limit switches are either Normally Open or Normally Closed
    public enum SwitchType {
        NO {
            @Override
            public String toString() {
                return "NO";
            }
        },
        NC {
            @Override
            public String toString() {
                return "NC";
            }
        }
    }
    private final SwitchType switchtype;

    /**
     * Constructor for limit switches being either Normally Open or Normally closed
     *
     * @param opmode  pass through the OpMode so the hardwareMap can be called
     * @param stype Whether switch is Normally Open (NO) or Normally Closed (NC)
     * @param sname Name of the switch in the Configuration File on Robot Controller
     */
    public LimitSwitch(LinearOpMode opmode, LimitSwitch.SwitchType stype, String sname) {
        this.switchtype = stype;
        this.name = sname;
        this.limitSwitch = opmode.hardwareMap.get(DigitalChannel.class, sname);
        this.limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Report current status of limit switch
     * Normally Closed switch is the opposite state of Normally Open
     *
     * @return boolean whether switch is pressed or not
     */
    public boolean isPressed() {
        switch(switchtype) {
            case NO: return limitSwitch.getState();
            case NC: return !limitSwitch.getState();
        }
        return false; // default
    }

    /**
     * Version of toString() function for this class
     * @return boolean return properties of the instance as a string
     */
    public String toString() {
        String pressed;
        if (isPressed()) {
            pressed = "is pressed";
        } else {
            pressed = "is not pressed";
        }

        return String.format("name: %s, type: %s, status: %s", name, switchtype.toString(), pressed);
    }
}
