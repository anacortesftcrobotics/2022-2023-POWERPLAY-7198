package org.firstinspires.ftc.teamcode.powerplay;

public class AutoOp {
    /*
    This will be the working auto file for now. All others will end up being modifications /  duplicates
    use linear op mode for this
    try the state machine thing here too
        while(opModeIsRunning)
            if(hasn't gotten to spot one)
                go to spot 1 (i.e move(10))
            if(has completed the last step, and hasn't completed this one)
                do the thing
    maybe use the switch for it. add to an int each time, and it'll only run that code for each  loop. Then like update telemetry or something
    1 - move
    2 - turn
    3 - move again
    4 - auto done, shut everything down
    something like that
    the odometry and IMU will be updated every loop, and it'll still be able to go through a complex process.

    "
    int i = 0;
    while(opModeIsRunning)
    {
        switch
            case 0:
                chassis.move(10);
                //move would check if the stop condition has been met, and if not, adjust the motor power values
                {move == done:i++}
                break;
            case 1:
                chassis.turn(90);
                break;
            case 2:
                chassis.move(10);
                break;
            case 3:
                //auto has been completed
                chassis.allStop

                break;
        odometry.update();
        imu.update();
        telemetry.update();
    }
    "
    */
}
