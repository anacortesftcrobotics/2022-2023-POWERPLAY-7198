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
    boolean[] hasChanged = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    Gamepad gamepad;

    Controller()
    {
    }

    Gamepad.RumbleEffect rumbleA;
    Gamepad.RumbleEffect rumbleB;
    Gamepad.RumbleEffect rumbleC;

    Gamepad.LedEffect ledA;
    Gamepad.LedEffect ledB;
    Gamepad.LedEffect ledC;
    
    public void initializeHardware()
    {
        rumbleA = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 500)
                .build();

        rumbleB = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0,1.0,500)
                .build();

        rumbleC = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0,0.0,200)
                .addStep(0.75,0.25,200)
                .addStep(0.5,0.5,200)
                .addStep(0.25,0.75,200)
                .addStep(0.0,1.0,200)
                .build();

        ledA = new Gamepad.LedEffect.Builder()
                .addStep(0.5,0.5,0.5,1000)
                .build();

        ledB = new Gamepad.LedEffect.Builder()
                .addStep(1.0,0.0,0.0,83)
                .addStep(1.0,0.5,0.0,83)
                .addStep(1.0,1.0,0.0,83)
                .addStep(0.5,1.0,0.0,83)
                .addStep(0.0,1.0,0.0,83)
                .addStep(0.0,1.0,0.5,83)
                .addStep(0.0,1.0,1.0,83)
                .addStep(0.0,0.5,1.0,83)
                .addStep(0.0,0.0,1.0,83)
                .addStep(0.5,0.0,1.0,83)
                .addStep(1.0,0.0,1.0,83)
                .addStep(1.0,0.0,0.5,83)
                .build();

        ledC = new Gamepad.LedEffect.Builder()
                .addStep(1.0,1.0,1.0,500)
                .addStep(0.0,0.0,0.0,500)
                .build();
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
