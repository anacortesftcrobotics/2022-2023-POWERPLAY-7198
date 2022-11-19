package org.firstinspires.ftc.teamcode.mentorbot;

/**
 * An interface for reporting that is formatted for telemetry
 *
 * @author Mentor Liam
 */
public interface Reportable {

    /**
     * Gets all reports from the object
     * @return the line-separated set of reports to put in telemetry
     */
    String getReportUpdate();
}
