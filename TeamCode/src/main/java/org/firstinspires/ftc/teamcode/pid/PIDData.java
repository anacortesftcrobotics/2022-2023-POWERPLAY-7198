//Draft PID class with data recording

public class PIDKai
{
    //initiates instance variables. Use ObjectReference.VariableName to access outside
    public double kP; //porportional constant
    public double kI; //integral constant
    public double kD; //derivative constant

    public double lastE; //last error value
    public double totalE; //integral of errors

    public String log = "Time(ms) Loops Error Porportional Integral Derivative/n"; //initializes log of PID
    private int loops;

    //Constructor Methods
    public PIDKai(double porportionalConstant, double integralConstant, double derivativeConstant)
    {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
    }

    //P function
    public PIDKai(double porportionalConstant)
    {
        kP = porportionalConstant;
        kI = 0;
        kD = 0;
    }

    //PI function
    public PIDKai(double porportionalConstant, double integralConstant)
    {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = 0;
    }

    //sets function to not affect output
    public PIDKai()
    {
        kP = 0;
        kI = 0;
        kD = 0;
    }
    
    //returns a control signal based on input expected and the actual value.
    public double getPID(double expectedValue, double actualValue)
    {
        double e;
        double u;
        
        loops += 1;
        log += System.currentTimeMillis() + " " + loops + " "; //logs system time & loop #

        e = expectedValue - actualValue; //r-y
        log += e + " "; //logs error

        u = kP * e;
        log += kP * e + " "; //logs P

        u += kI * getI(e);
        log += u - kP * e + " "; // logs I

        u += kD * getD(e);
        log += kD * getD(e) + "/n"; //logs D

        lastE = e; //stores the newer error value as the older. (at end of method)
        return u;
    }

    //adds to total integral based on current and past error
    //private to safegaurd accidental additions to totalE
    private double getI(double error)
    {
        totalE = 0.66*totalE + 0.5*(lastE + error); //0.5*(lastE-error) + error simplified + past value of E (devalued)
        return totalE;
    }

    //returns derivative between error and lastE
    public double getD(double error)
    {
        return error-lastE;
    }

    //retuns string of recorded log
    public String getLog()
    {
        return log;
    }

}