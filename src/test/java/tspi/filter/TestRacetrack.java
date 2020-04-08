package tspi.filter;

import junit.framework.TestCase;
import tspi.rotation.Vector3;

public class TestRacetrack extends TestCase {

    public void testContinuity() {
        double radius = 1.0;
        double velocity = 1.0;
        Vector3 c1 = new Vector3(0,0,0);
        Vector3 c2 = new Vector3(0,2,0);
        Racetrack racetrack = new Racetrack(0.0, c1, c2, radius, velocity);

        int n = 100000;
        double dt = racetrack.getPerimeter()/n;
        Vector3 p0 = racetrack.getPositionVector(0.0);
        for (double t=dt; t<racetrack.getPerimeter()*2; t+=dt) {
            Vector3 p1 = racetrack.getPositionVector(t);
            Vector3 d = p1.subtract(p0);
            assertTrue(
                    "subsequent points violate velocity bounds at t="+toString(),
                    (d.getAbs() < velocity*dt)
            );
            System.out.println(p0.toTupleString());
            p0 = p1;
        }
    } // TODO while this is a good sanity check for Trajectories in general, we could just test the transitions between racetrack piecewise defined geometries...

}
