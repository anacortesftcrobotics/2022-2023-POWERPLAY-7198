package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.sun.tools.javac.util.Pair;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This class manages the lift system on the 2022 - 2023 powerplay robot
 * @author Lemon
 */
public class Lift implements SubsystemManager{

    public Lift()
    {

    }

    DcMotor leftLift, rightLift;

    TouchSensor zero;

    //int liftHeight = 0; //The motor encoder ticks
    int liftChange = 0; //The shift from a preset height, combo of manual adjustment, jeff, and lift zeroing.
    int liftManual = 0; //The new jeff. It wouldn't work with just the one.
    int[] storage = {0,0};
    int lastPlace;

    /**
     * The method in all subsystem classes to register the hardware that this class uses.
     * In this case it's the lift motors for the robot and the limit switch at the bottom.
     */
    public void initializeHardware (HardwareMap hardwareMap)
    {
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            setShift(liftChange - 10);
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
        //trial using old power supply system with new heights
        //adjust power based on distance between points
        //Max(0.2, Min(0.8, x * (1 / 600) ))
        //Then run PID and amp dampening on top of that power calc.

        Pair<Integer, Integer> pair = levelToHeight(input);

        int lx = pair.fst + liftChange + liftManual;
        int rx = pair.snd + liftChange + liftManual;
        leftLift.setTargetPosition(lx);
        rightLift.setTargetPosition(rx);

        lastPlace = input;

        double leftLiftError = (leftLift.getTargetPosition() + liftChange + liftManual) - (leftLift.getCurrentPosition() + liftChange + liftManual);
        double rightLiftError = (rightLift.getTargetPosition() + liftChange + liftManual) - (rightLift.getCurrentPosition() + liftChange + liftManual);
        if(leftLift.getCurrentPosition() > leftLift.getTargetPosition())
            leftLiftError = -leftLiftError;
        if(rightLift.getCurrentPosition() > rightLift.getTargetPosition())
            rightLiftError = -rightLiftError;

        double leftPower = 0.6;
        double rightPower = 0.6;
        double lerror = Math.max(0.2, Math.min(-0.2, (leftLiftError / 100)));
        double rerror = Math.max(0.2, Math.min(-0.2, (rightLiftError / 100)));

        leftPower -= lerror;
        rightPower += rerror;

        double leftAMP = ((DcMotorEx) leftLift).getCurrent(CurrentUnit.AMPS);
        double rightAMP = ((DcMotorEx) rightLift).getCurrent(CurrentUnit.AMPS);

        if(leftAMP > 8 || rightAMP > 8)
        {
            double v = 1 - ((Math.max(leftAMP, rightAMP) - 7) / 10);
            leftPower *= v;
            rightPower *= v;
        }

        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        leftLift.setPower(leftPower); //left power
        rightLift.setPower(rightPower); //right power

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //right left
    //846 831,   838 840    13
    //1709 1679, 1712 1675  23
    //2563 2511, 2562 2506  33
    //1 - 2624 2576         34
    //2 - 2730 2663         35
    /**
     * This function should be run once every loop. This should hopefully stop motors from burning out at the bottom.
     */
    public void liftSafetyCheck()
    {
        if((Math.abs(leftLift.getCurrentPosition()) - lastPlace) > 10)
            if((leftLift.getCurrentPosition() - storage[0]) < 5 && (leftLift.getCurrentPosition() + liftChange + liftManual) < 50)
                leftLift.setPower(0.0);
        if((Math.abs(rightLift.getCurrentPosition()) - lastPlace) > 10)
            if((rightLift.getCurrentPosition() - storage[0]) < 5 && (rightLift.getCurrentPosition() + liftChange + liftManual) < 50)
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
    public Pair<Integer, Integer> levelToHeight (int level) {
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
        int leftLiftHeight = 0;
        int rightLiftHeight = 0;
        switch (level) {//stack 1
            case 2:
            case 6:
                //ground
                //stack 2
                leftLiftHeight = 105;
                rightLiftHeight = 105;
                break;
            case 3:
                //stack 3
                leftLiftHeight = 210;
                rightLiftHeight = 210;
                break;
            case 4:
                //stack 4
                leftLiftHeight = 315;
                rightLiftHeight = 315;
                break;
            case 5:
                //stack 5
                leftLiftHeight = 420;
                rightLiftHeight = 420;
                break;
            case 7:
                //low
                leftLiftHeight = 1133;
                rightLiftHeight = 1125;
                break;
            case 8:
                //medium
                leftLiftHeight = 2045;
                rightLiftHeight = 2005;
                break;
            case 9:
                //high
                leftLiftHeight = 2912;
                rightLiftHeight = 2851;
                break;
            default:
                //floor
                leftLiftHeight = 10;
                rightLiftHeight = 10;
        }
        return new Pair<>(leftLiftHeight, rightLiftHeight);
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


    public void setManual(int input) {liftManual = input;}
}
