package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class converts directional power levels to motor power values for a mecanum chassis.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Mecanum {

    private Mecanum() {}

    /**
     * Returns motor power levels based on a relative power level forward, right, and CCW.
     * To avoid scaling outputs, ensure forward, right, and rotateCCW add to <= 1.
     * @param forward       a motor power level, 1 full speed forward, -1 full speed backward.
     * @param right         a motor power level, 1 full speed right, -1 full speed left.
     * @param rotateCCW     a motor power level, 1 full speed CCW, -1 full speed CW.
     * @return              returns a double array with motor power levels 1 to -1. 
     *                          Outputs will be in range 1 to -1, scaling power down if needed. 
     *                          Index values: 0-leftBackPower; 1-rightBackPower; 2-leftFrontPower; 3-rightFrontPower.
     */
    public static double[] getMotorPower(double forward, double right, double rotateCCW) {
        double[] out = {0.0, 0.0, 0.0, 0.0};
        double[] f = forward(forward);
        double[] r = right(right);
        double[] rCCW = rotateCCW(rotateCCW);
        double max = 0;

        for (i = 0; i <= 3; i++) {
            out[i] += f[i] + r[i] + rCCW[i];
            if (out[i] > 1)
                max = Math.max(max, Math.abs(out[i]));
        }

        if (max > 1) {
            for (i = 0; i <= 3; i++)
                out[i] /= max;
        }

        return out;
    }

    /**
     * Returns motor power levels based on a relative power level forward, right, and CCW.
     * @param forward       a motor power level, (+) forward, (-) backward.
     * @param right         a motor power level, (+) right, (-) left.
     * @param rotateCCW     a motor power level, (+) CCW, (-) CW.
     * @return              returns a double array with motor power levels.
     *                          Index values: 0-leftBackPower; 1-rightBackPower; 2-leftFrontPower; 3-rightFrontPower.
     */
    public static double[] getMotorPowerNoLimit(double forward, double right, double rotateCCW) {
        double[] out = {0.0, 0.0, 0.0, 0.0};
        double[] f = forward(forward);
        double[] r = right(right);
        double[] rCCW = rotateCCW(rotateCCW);
        double max = 0;

        for (i = 0; i <= 3; i++)
            out[i] += f[i] + r[i] + rCCW[i];
        
        return out;
    }

    /**
     * Returns motor power levels based on a relative power level forward.
     * @param f     a motor power level, 1 full speed forward, -1 full speed backward.
     * @return      returns a double array with motor power levels. Index values: 0-leftBackPower; 
     *                  1-rightBackPower; 2-leftFrontPower; 3-rightFrontPower
     */
    public static double[] forward(double f) {
        double[] out = {0.0, 0.0, 0.0, 0.0};

        out[0] = f;
        out[1] = f;
        out[2] = f;
        out[3] = f;
        return out;
    }

    /**
     * Returns motor power levels based on a relative power level strafing right.
     * @param r     a motor power level, 1 full speed right, -1 full speed left.
     * @return      returns a double array with motor power levels. Index values: 0-leftBackPower; 
     *                  1-rightBackPower; 2-leftFrontPower; 3-rightFrontPower
     */
    public static double[] right(double r) {
        double[] out = {0.0, 0.0, 0.0, 0.0};

        out[0] = -r;
        out[1] = r;
        out[2] = r;
        out[3] = -r;

        return out;
    }

    /**
     * Returns motor power levels based on a relative power level turning left.
     * @param rCCW  a motor power level, 1 full speed CCW, -1 full speed CW.
     * @return      returns a double array with motor power levels. Index values: 0-leftBackPower; 
     *                  1-rightBackPower; 2-leftFrontPower; 3-rightFrontPower
     */
    public static double[] rotateCCW(double rCCW) {
        double[] out = {0.0, 0.0, 0.0, 0.0};

        out[0] = rCCW;
        out[1] = -rCCW;
        out[2] = rCCW;
        out[3] = -rCCW;

        return out;
    }
}