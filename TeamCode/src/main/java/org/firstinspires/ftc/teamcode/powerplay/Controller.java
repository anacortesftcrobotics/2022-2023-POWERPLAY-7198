package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class should be used to run a gamepad more efficiently.
 * @author      Lemon
 */

public class Controller {
    /*
    0 - a
    1 - b
    2 - x
    3 - y
    4 - dpad_down
    5 - dpad_right
    6 - dpad_left
    7 - dpad_up
    8 - left_stick_button
    9 - right_stick_button
    10 - left_bumper
    11 - right_bumper
    12 - back
    13 - start
    14 - guide
    15 - left trigger
    16 - right trigger
     */
    Gamepad gamepad;
    boolean[] hasChanged = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    boolean[] values = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};


    public Controller() {
    }

    public void initializeHardware() {
        update();
    }

    /**
     * changes the gamepad assigned to a controller object.
     */
    public void setGamepad(Gamepad gamepadX)
    {
        gamepad = gamepadX;
        update();
    }

    public void update(){
        values [0] = gamepad.a;
        values[1]= gamepad.b;
        values[2] = gamepad.b;
        values[3] = gamepad.x;
        values[4] = gamepad.dpad_down;
        values[5] = gamepad.dpad_left;
        values[6] = gamepad.dpad_right;
        values[7] = gamepad.dpad_up;
        values[8] = gamepad.left_stick_button;
        values[9] = gamepad.right_stick_button;
        values[10] = gamepad.left_bumper;
        values[11] = gamepad.right_bumper;
        values[12] = gamepad.back;
        values[13] = gamepad.start;
        values[14] = gamepad.guide;
        values[15] = gamepad.left_trigger>0.5;
        values[16] = gamepad.right_trigger>0.5;
    }

    /**
     * A method that checks if the button is still pressed from the last loop, to prevent something from being called multiple loops in a row.
     * Needs to be written like:
     * <p>
     *  if(button(0,gamepad1.a))
     * </p>
     * <p>
     *      //Do a thing when a is pressed
     * </p>
     * Should be run each loop in order to properly function.
     * @param button the corresponding reference number of the button being used:
     *  (0 - a)
     *     (1 - b)
     *     (2 - x)
     *     (3 - y)
     *     (4 - dpad_down)
     *     (5 - dpad_right)
     *     (6 - dpad_left)
     *     (7 - dpad_up)
     *     (8 - left_stick_button)
     *     (9 - right_stick_button)
     *     (10 - left_bumper)
     *     (11 - right_bumper)
     *     (12 - back)
     *     (13 - start)
     *     (14 - guide)
     *     (15 - left trigger)
     *     (16 - right trigger)
     * @return a boolean value, which should mean whether to run the function.
     */
    public boolean button (int button)
    {
        update();
        if(values[button])
        {
            if(!hasChanged[button])
            {
                hasChanged[button] = true;
                return true;
            }
        }
        else
        {
            hasChanged[button] = false;
        }
        return false;
    }
}
