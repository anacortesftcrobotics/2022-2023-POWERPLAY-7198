package org.firstinspires.ftc.teamcode.mentorcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class LimitSwitch {
    public String name;
    private DigitalChannel myswitch = null;
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
        };
    }
    private SwitchType switchtype;

    /**
     * Constructor for limit switches being either Normally Open or Normally closed
     *
     * @param opmode  pass through the OpMode so the hardwareMap can be called
     * @param switchtype Whether switch is Normally Open (NO) or Normally Closed (NC)
     * @param name Name of the switch in the Configuration File on Robot Controller
     */
    public LimitSwitch(LinearOpMode opmode, LimitSwitch.SwitchType switchtype, String name) {
        this.switchtype = switchtype;
        this.name = name;
        this.myswitch = opmode.hardwareMap.get(DigitalChannel.class, name);
        this.myswitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Report current status of limit switch
     * Normally Closed switch is the opposite state of Normally Open
     */
    public boolean isPressed() {
        switch(switchtype) {
            case NO: return myswitch.getState();
            case NC: return !myswitch.getState();
        }
        return false; // default
    }

    /**
     * Version of toString() function for this class
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
