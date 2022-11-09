package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * A class that is Loggable will log activity to the robot log file for various purposes. Although implementations of
 * this class may log many items at the verbose level, any logging level lower than error should be carefully
 * considered to avoid polluting the logs.
 *
 * @author Mentor Liam
 */
public interface Loggable {

    enum LoggingLevel {
        VERBOSE,
        DEBUG,
        INFO,
        WARN,
        ERROR
    }

    void registerLogger(RobotLog log);

    void setLoggingLevel(LoggingLevel level);
}