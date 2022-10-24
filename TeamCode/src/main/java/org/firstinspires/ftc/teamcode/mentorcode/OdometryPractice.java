package org.firstinspires.ftc.teamcode.mentorcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Odometry Practice", group = "Coach")
public class OdometryPractice extends LinearOpMode {

    // Create a CoachHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    CoachHardware robot = new CoachHardware(this, true);

    @Override
    public void runOpMode() {
//        double drive = 0;
//        double turn = 0;
//        double arm = 0;
//        double handOffset = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.opmodeRunTime.reset();
//      robot.loopCount = 1;
        boolean runningMotor = false;
        robot.onetimeFirstRun();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.runLoop();

            // turning on single motor to test config is correct
//            if ((!runningMotor) && (robot.loopCount > 0) && (robot.loopCount < 3000)) {
//                robot.setSingleMotorPower(1, 0.5);
//                runningMotor = true;
//            } else if ((robot.loopCount >= 3000)) {
//                robot.setSingleMotorPower(1, 0.0);
//            }
            boolean testSingleMotor = false; //true;

            /*
            if ((!runningMotor) && (testSingleMotor)) {
                for (int port = 0; port < 4; port++) {
                    // give motor power
                    robot.setSingleMotorPower(port, 0.25);
                    robot.setRobotTelemetry();
                    telemetry.update();
                    sleep(500);
                    // stop motor
                    robot.setSimplePower(port, 0);
                    robot.setRobotTelemetry();
                    telemetry.update();
                    sleep(300);
                }
            }
            if (!runningMotor) {
                robot.setSimplePower(0.25, 0.25);
                robot.setRobotTelemetry();
                telemetry.update();
                sleep(500);
                robot.runLoop();
                robot.setSimplePower(0, 0);
                robot.setRobotTelemetry();
                telemetry.update();
                sleep(300);
                runningMotor = true;
            }
            */

            robot.odometrywheels.pose();
            robot.setRobotTelemetry();
            telemetry.update();

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
//            drive = -gamepad1.left_stick_y;
//            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion. Use RobotHardware class
//            robot.driveRobot(drive, turn);

            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
//            if (gamepad1.right_bumper)
//                handOffset += robot.HAND_SPEED;
//            else if (gamepad1.left_bumper)
//                handOffset -= robot.HAND_SPEED;
//            handOffset = Range.clip(handOffset, -0.5, 0.5);
//
//            // Move both servos to new position.  Use RobotHardware class
//            robot.setHandPositions(handOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
//            if (gamepad1.y)
//                arm = robot.ARM_UP_POWER;
//            else if (gamepad1.a)
//                arm = robot.ARM_DOWN_POWER;
//            else
//                arm = 0;
//
//            robot.setArmPower(arm);
//
//            // Send telemetry messages to explain controls and show robot status
//            telemetry.addData("Drive", "Left Stick");
//            telemetry.addData("Turn", "Right Stick");
//            telemetry.addData("Arm Up/Down", "Y & A Buttons");
//            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
//            telemetry.addData("-", "-------");
//
//            telemetry.addData("Drive Power", "%.2f", drive);
//            telemetry.addData("Turn Power", "%.2f", turn);
//            telemetry.addData("Arm Power", "%.2f", arm);
//            telemetry.addData("Hand Position", "Offset = %.2f", handOffset);

            robot.setRobotTelemetry();
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }

//    /** keeps track of the line of the poem which is to be emitted next */
//    int poemLine = 0;
//
//    /** keeps track of how long it's been since we last emitted a line of poetry */
//    ElapsedTime poemElapsed = new ElapsedTime();
//
//    static final String[] poem = new String[] {
//
//            "Mary had a little lamb,",
//            "His fleece was white as snow,",
//            "And everywhere that Mary went,",
//            "The lamb was sure to go.",
//            "",
//            "He followed her to school one day,",
//            "Which was against the rule,",
//            "It made the children laugh and play",
//            "To see a lamb at school.",
//            "",
//            "And so the teacher turned it out,",
//            "But still it lingered near,",
//            "And waited patiently about,",
//            "Till Mary did appear.",
//            "",
//            "\"Why does the lamb love Mary so?\"",
//            "The eager children cry.",
//            "\"Why, Mary loves the lamb, you know,\"",
//            "The teacher did reply.",
//            "",
//            ""
//    };
//
//    @Override public void runOpMode() {
//
//        // The interval between lines of poetry, in seconds
//        double sPoemInterval = 0.6;
//
//        /**
//         * Wait until we've been given the ok to go. For something to do, we emit the
//         * elapsed time as we sit here and wait. If we didn't want to do anything while
//         * we waited, we would just call {@link #waitForStart()}.
//         */
//        while (!isStarted()) {
//            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
//            telemetry.update();
//            idle();
//        }
//
//        // Ok, we've been given the ok to go
//
//        // Reset to keep some timing stats for the post-'start' part of the opmode
//        opmodeRunTime.reset();
//        int loopCount = 1;
//
//        // Go go gadget robot!
//        while (opModeIsActive()) {
//
//            // Emit poetry if it's been a while
//            if (poemElapsed.seconds() > sPoemInterval) {
//                emitPoemLine();
//            }
//
//            // As an illustration, show some loop timing information
//            telemetry.addData("loop count", loopCount);
//            telemetry.addData("ms/loop", "%.3f ms", opmodeRunTime.milliseconds() / loopCount);
//
//            // Show joystick information as some other illustrative data
//            telemetry.addLine("left joystick | ")
//                    .addData("x", gamepad1.left_stick_x)
//                    .addData("y", gamepad1.left_stick_y);
//            telemetry.addLine("right joystick | ")
//                    .addData("x", gamepad1.right_stick_x)
//                    .addData("y", gamepad1.right_stick_y);
//
//        }
//    }
//
//    // emits a line of poetry to the telemetry log
//    void emitPoemLine() {
//        telemetry.log().add(poem[poemLine]);
//        poemLine = (poemLine+1) % poem.length;
//        poemElapsed.reset();
//    }
//
    }
}
