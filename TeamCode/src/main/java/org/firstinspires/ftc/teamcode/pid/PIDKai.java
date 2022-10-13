//work in progress, need to add I & D.
//account for initial value of 0.0 on oldE?

public class PIDKai
{
    //initiates instance variables. Use ObjectReference.VariableName to access outside
    public double kP; //porportional constant
    public double kI; //integral constant
    public double kD; //derivative constant

    public double lastE; //last error value

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
        u = kP*e; //+ getI(e) + getD(e)

        //System.out.println("e: " + e);

        lastE = e; //stores the newer error value as the older at end of method.
        return u;
    }

    //multiplies input by porportional constant.
    public double getP(double error)
    {
        return kP*error;
    }
}