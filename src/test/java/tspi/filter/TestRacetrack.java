package tspi.filter;

import junit.framework.TestCase;
import tspi.rotation.Vector3;

public class TestRacetrack extends TestCase {
    boolean verbose = true;

    double radius = 1.0;
    double velocity = 1.0;
    Vector3 c1 = new Vector3(0,0,0);
    Vector3 c2 = new Vector3(0,2,0);
    Racetrack racetrack = new Racetrack(0.0, c1, c2, radius, velocity);

    double epsilon = 0.000000000001;
    int n = 1000;
    double dt = racetrack.getPerimeter() / n;

    public void testContinuity() {
        Vector3 p0 = new Vector3( racetrack.getPositionVector(0.0) );
        double max = velocity * dt + epsilon;
        if (verbose) System.out.println( "dt="+dt+", max="+max+", epsilon="+epsilon);

        if(verbose) System.out.println(p0.toTupleString());

        for (double t=dt; t<racetrack.getPerimeter()*2; t+=dt) {
            Vector3 p1 = new Vector3( racetrack.getPositionVector(t) );
            double d = new Vector3(p1).subtract(p0).getAbs();

            if(verbose) System.out.println( "t="+t+", p="+p0.toTupleString() + ", d="+d);

            if(d>max)
                System.out.println("error");

            assertTrue(
                    "subsequent points violate velocity bounds at t="+t+", d="+d,
                    (d <= max)
            );
            p0 = p1;
        }
    } // TODO while this is a good sanity check for Trajectories in general, we could just test the transitions between racetrack piecewise defined geometries...

    public void testConstantVelocity() {
        for (double t=0; t<racetrack.getPerimeter()*2; t+=dt) {
            Vector3 p = new Vector3( racetrack.getPositionVector(t) );
            Vector3 v = new Vector3( racetrack.getVelocityVector(t) );
            double speed = v.getAbs();
            if(verbose) System.out.println( "t="+t+", p="+p.toTupleString()+", v="+v.toTupleString());
            assertTrue(
                    "velocity must be constant",
                    (Math.abs(speed - Math.abs(velocity)) <= epsilon)
            );
        }
    }
}
