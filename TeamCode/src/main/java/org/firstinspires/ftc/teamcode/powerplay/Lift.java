package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class manages the lift system on the 2022 - 2023 powerplay robot
 * @author Lemon
 */
public class Lift implements SubsystemManager{

    public static HardwareMap hardwareMap;

    public Lift(HardwareMap hwMap)
    {
        hardwareMap = hwMap;
    }

    DcMotor leftLift, rightLift;

    TouchSensor zero;

    //int liftHeight = 0; //The motor encoder ticks
    int liftChange = 0; //The shift from a preset height, combo of manual adjustment, jeff, and lift zeroing.
    int[] storage = {0,0};
    int lastPlace;

    /**
     * The method in all subsystem classes to register the hardware that this class uses.
     * In this case it's the lift motors for the robot and the limit switch at the bottom.
     */
    public void initializeHardware ()
    {
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zero = hardwareMap.get(TouchSensor.class, "bottomLimit");
    }

    /**
     * This function should move the lift to zero and set that as the new zero for the duration of runtime.
     */
    public void liftZero ()
    {
        while(!zero.isPressed())
        {
            setShift(getShift() - 10);
            liftSet(0);
        }
    }

    /**
     * This function should tell the lift to go to a certain height.
     * 0 - zero, bottom, floor
     * 1 - stack 1
     * 2 - stack 2
     * 3 - stack 3
     * 4 - stack 4
     * 5 - stack 5
     * 6 - ground
     * 7 - low
     * 8 - medium
     * 9 - high
     * @param input is a number 1 through 9 that tells the lift where to go.
     */
    public void liftSet(int input)
    {
        leftLift.setTargetPosition(levelToHeight(input) - liftChange);
        rightLift.setTargetPosition(levelToHeight(input) - liftChange);
        lastPlace = levelToHeight(levelToHeight(input) - liftChange);

        double liftError = (leftLift.getCurrentPosition() + liftChange) - (rightLift.getCurrentPosition() + liftChange);
        double leftPower = 0.5;
        double rightPower = 0.5;

        double error = Math.max(0.2, Math.min(-0.2, (liftError / 42)));
        leftPower -= error;
        rightPower += error;

        double leftAMP = ((DcMotorEx) leftLift).getCurrent(CurrentUnit.AMPS);
        double rightAMP = ((DcMotorEx) rightLift).getCurrent(CurrentUnit.AMPS);

        if(leftAMP > 8 || rightAMP > 8)
        {
            leftPower *= (1 - ((leftAMP - 7) / 10));
            rightPower *= (1 - ((rightAMP - 7) / 10));
        }

        leftLift.setPower(leftPower);
        rightLift.setPower(rightPower);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**
     * This function should be run once every loop. This should hopefully stop motors from burning out at the bottom.
     */
    public void liftSafetyCheck()
    {
        if((Math.abs(leftLift.getCurrentPosition()) - lastPlace) > 10)
            if((leftLift.getCurrentPosition() - storage[0]) < 5 && (leftLift.getCurrentPosition() + liftChange) < 50)
                leftLift.setPower(0.0);
        if((Math.abs(rightLift.getCurrentPosition()) - lastPlace) > 10)
            if((rightLift.getCurrentPosition() - storage[0]) < 5 && (rightLift.getCurrentPosition() + liftChange) < 50)
                leftLift.setPower(0.0);

        storage[0] = leftLift.getCurrentPosition();
        storage[1] = rightLift.getCurrentPosition();
    }

    /**
     * This function takes a string input, checks it against a list of preset words, and then tells the
     * @param input is a string that corresponds to different preset levels. ground for 0, etc.
     */
    public void liftSet(String input)
    {
        liftSet(wordSearch(input));
    }

    /**
     * This function is used to convert the strings passed into liftSet and convert them in a number 0 through 9.
     * @param input is the string passed in to be read.
     * @return is the level that the string corresponded to.
     */
    public int wordSearch(String input)
    {
        int level = 0;
        int liftHeight = 0;
        String looking = input;
        for(int i = input.length() - 1; i > 0; i--)
        {
            if(looking.substring(i, i + 1).equals(" "))
            {
                looking = looking.substring(0,i - 1) + looking.substring(i + 1);
                i--;
            }
        }
        looking = looking.toLowerCase();
        if(looking.equals("zero") || looking.equals("bottom") || looking.equals("floor"))
            level = 0; //0
        else if(looking.equals("stack1") || looking.equals("1stack"))
            level = 1; //1
        else if(looking.equals("stack2") || looking.equals("2stack"))
            level = 2; //2
        else if(looking.equals("stack3") || looking.equals("3stack"))
            level = 3; //3
        else if(looking.equals("stack4") || looking.equals("4stack"))
            level = 4; //4
        else if(looking.equals("stack5") || looking.equals("5stack"))
            level = 5; //5
        else if(looking.equals("ground") || looking.equals("g"))
            level = 6; //6
        else if(looking.equals("low") || looking.equals("l"))
            level = 7; //7
        else if(looking.equals("medium") || looking.equals("m"))
            level = 8; //8
        else if(looking.equals("high") || looking.equals("h"))
            level = 9; //9
        else
            level = 0;
        /*
        0 - zero, bottom, floor
        1 - stack 1
        2 - stack 2
        3 - stack 3
        4 - stack 4
        5 - stack 5
        6 - ground
        7 - low
        8 - medium
        9 - high
         */
        return level;
    }

    /**
     * This function takes a number 1 through 9 and converts it to encoder ticks.
     * @param level is the level number.
     * @return is the height in encoder ticks.
     */
    public int levelToHeight (int level) {
        /*
        0 - floor
        1 - stack 1
        2 - stack 2
        3 - stack 3
        4 - stack 4
        5 - stack 5
        6 - ground
        7 - low
        8 - medium
        9 - high
         */
        int liftHeight = 0;
        switch (level) {
            case 0:
                liftHeight = 30;
                break;
            case 1:
                liftHeight = 30;
                break;
            case 2:
                liftHeight = 148;
                break;
            case 3:
                liftHeight = 253;
                break;
            case 4:
                liftHeight = 358;
                break;
            case 5:
                liftHeight = 484;
                break;
            case 6:
                liftHeight = 260;
                break;
            case 7:
                liftHeight = 1230;
                break;
            case 8:
                liftHeight = 2150;
                break;
            case 9:
                liftHeight = 3000;
                break;
            default:
                liftHeight = 30;
        }
        return liftHeight;
    }

    /**
     * This changes the shift of the robot. Should be used for manual adjustment and jeff style depositing.
     * Should always be called something like <p>"lift.setShift(lift.getShift + x);"</p>
     * @param input is the value that the lift change will be set to.
     */
    public void setShift(int input)
    {
        liftChange = input;
    }

    /**
     * This gets the lift change variable and passes it back.
     * @return the lift change variable.
     */
    public int getShift()
    {
        return liftChange;
    }
}
