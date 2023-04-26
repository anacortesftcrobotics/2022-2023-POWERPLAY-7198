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

    }

    /**
     * changes the gamepad assigned to a controller object.
     */
    public void setGamepad(Gamepad gamepadX)
    {
        gamepad = gamepadX;
        //values = {gamepad.a, gamepad.b, gamepad.x, gamepad.y, gamepad.dpad_down, gamepad.dpad_right, gamepad.dpad_left, gamepad.dpad_up, gamepad.left_stick_button, gamepad.right_stick_button, gamepad.left_bumper, gamepad.right_bumper, gamepad.back, gamepad.start, gamepad.guide, (gamepad.left_trigger>0.5), (gamepad.right_trigger>0.5)};
    }

    public void update(){
        values [0] = gamepad.a;
        values[1]= gamepad.b;
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
