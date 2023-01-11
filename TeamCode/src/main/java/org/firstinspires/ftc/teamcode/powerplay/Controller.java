package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class should be used to run a gamepad more efficiently.
 * This class is mostly deprecated and no longer functional, except for the button function.
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
    boolean[] hasChanged = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    Gamepad gamepad;

    /**
     * Empty constructor
     */
    Controller()
    {

    }

    /**
     * Empty hardware initialization because the previous version was non functional.
     */
    public void initializeHardware()
    {

    }

    /**
     * changes the gamepad assigned to a controller object.
     */
    public void setGamepad (Gamepad gamepadX)
    {
        gamepad = gamepadX;
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
     * @param button the corresponding hasChanged value of the button being used
     * @param value the boolean output of calling the button. (i.e calling gamepadX.a)
     * @return a boolean value, which should mean whether to run the function.
     */
    public boolean button (int button, boolean value)
    {

        if(value)
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
