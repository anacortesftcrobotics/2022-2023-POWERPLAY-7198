//Example runner of PIDKai

public class PIDTester
{
    public static void main(String[] args)
    {
        //creates PIDKai object with arbitrary values to test
        PIDKai pid = new PIDKai(0.5, 0.2, 1);

        //runs methods to test
        System.out.println("lastE: " + pid.lastE);
        System.out.println("Out: " + pid.getPID(200, 100));
        System.out.println("lastE: " + pid.lastE);
        System.out.println("Out: " + pid.getPID(-20, 30));
        System.out.println("lastE: " + pid.lastE);

        System.out.println("should equal kP*value: " + pid.getP(0.5));
        System.out.println("Porportional constant: " + pid.kP);
    }
}