//Example runner of PIDKai

public class PIDTester
{
    public static void main(String[] args)
    {
        //creates PIDKai object with arbitrary values to test
        PIDKai pid = new PIDKai(0.5, 0.2, 1);

        //accesses constants
        System.out.println("\nPorportional constant: " + pid.kP);
        System.out.println("Integral constant: " + pid.kI);
        System.out.println("Derivative constant: " + pid.kD);

        //runs methods to test
        System.out.println("lastE: " + pid.lastE);
        System.out.println("totalE: " + pid.totalE);
        System.out.println("Out: " + pid.getPID(200, 100));
        System.out.println("lastE: " + pid.lastE);
        System.out.println("totalE: " + pid.totalE);
        System.out.println("Out: " + pid.getPID(-20, 30));
        System.out.println("lastE: " + pid.lastE);
        System.out.println("totalE: " + pid.totalE);
    }
}