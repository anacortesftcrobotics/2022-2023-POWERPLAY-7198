package org.firstinspires.ftc.teamcode.powerplay;

/**
 * @author kaiwallis
 */
public final class Logger {

    private static Logger instance;
    private static String line1 = "";
    private static String line2 = "";
    private static String line3 = "";

    private Logger() {}

    public static Logger getInstance() {
        if(instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public static void setLine1(String text) {
        line1 = text;
    }

    public static String getLine1() {
        return line1;
    }

    public static void setLine2(String text) {
        line2 = text;
    }

    public static String getLine2() {
        return line2;
    }

    public static void setLine3(String text) {
        line3 = text;
    }

    public static String getLine3() {
        return line3;
    }
}