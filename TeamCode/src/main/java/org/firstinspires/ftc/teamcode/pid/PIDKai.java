//Working PID class

public class PIDKai
{
    //initiates instance variables. Use ObjectReference.VariableName to access outside
    public double kP; //porportional constant
    public double kI; //integral constant
    public double kD; //derivative constant

    public double lastE; //last error value
    public double totalE; //integral of errors

    //Constructor Method
    public PIDKai(double porportionalConstant, double integralConstant, double derivativeConstant)
    {
        kP = porportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
    }
    
    //returns a control signal based on input expected and the actual value.
    public double getPID(double expectedValue, double actualValue)
    {
        double e;
        double u;
        
        e = expectedValue - actualValue; //r-y
        u = kP * e + kI * getI(e) + kD * getD(e);

        lastE = e; //stores the newer error value as the older. (at end of method)
        return u;
    }

    //adds derivative between error and lastE to total derivative
    //private to safegaurd accidental additions to totalE
    private double getI(double error)
    {
        totalE += 0.5*(lastE + error); //0.5*(lastE-error) + error simplified
        return totalE;
    }

    //returns derivative between error and lastE
    public double getD(double error)
    {
        return error-lastE;
    }

}