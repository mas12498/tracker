package tspi.filter;

import junit.framework.TestCase;
import tspi.rotation.Vector3;
import tspi.util.TVector;

/** Trying to make some sanity tests for the 'racetrack' type target simulator. But there are some ideas here that could
 * apply to arbitrary paths from any simulator; like tests for uniform motion, or closed paths, or basic continuity checks.
 * TODO add numerical path integrals for velocity and acceleration; should be about zero for uniform, closed motion...
 * */
public class TestRacetrack extends TestCase {
    boolean verbose = true;
    double radius = 1.0;
    double velocity = 1.0;
    Vector3 c1 = new Vector3(0,0,0);
    Vector3 c2 = new Vector3(0,2,0);
    Racetrack racetrack = new Racetrack(0.0, c1, c2, radius, velocity);
    double epsilon = 0.000000000001;
    int n = 5000;
    double dt = racetrack.getPerimeter() / n;
    // TODO tests methods should accept racetrack and some parameters as an argument so we can test a variety of configurations

    /** Make sure successive points within some time bound are within some position bound.
     * Should be generalized beyond just position and constant velocity cases... */
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
    }

    /** ensures the racetrack motion really does have uniform speed */
    public void testConstantSpeed() {
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

    /** the analytic instantaneous velocity should compare well with numerical differentiation.
     * This should be made into a general test for all trajectories, and acceleration...*/
    public void testAgainstNumericalDerivative() {
        // crud, I don't know, how much can we expect velocity and finite differences to agree?
        double bound = dt*dt*velocity*velocity; // sure, let's go with that. Keep in mind this changes with sampling rates, velocity, the order of the function, etc.

        // incrementally sample motion
        Vector3 p0 = new Vector3( racetrack.getPositionVector(0.0) );
        for (double t=dt; t<racetrack.getPerimeter()*2; t+=dt) {

            // compute finite difference as an estimate for velocity
            Vector3 p1 = new Vector3( racetrack.getPositionVector(t) );
            Vector3 dp = new Vector3(p1).subtract(p0);
            // might consider interpolating from more points for better bounds...

            // use analytic velocity at endpoint as your truth
            Vector3 v = new Vector3( racetrack.getVelocityVector(t) );

            // find the error
            Vector3 e = new Vector3(v).multiply(dt).subtract(dp);
            double error = e.getAbs();

            if(verbose) System.out.println("e="+e.toTupleString()+", ||e||="+error+", t="+t);
            //System.out.println("p="+p1.toTupleString()+"v="+v.toTupleString()+", e="+e.toTupleString()+", t="+t);

            assertTrue(
                    "finite differences varied more than |e|="+bound+" from analytic velocity at t="+t+", e="+e.toTupleString(),
                    (error<bound)
            );
            p0 = p1;
            v = new Vector3( racetrack.getVelocityVector(t) );
        }
    }

    public void testClosedPath() {
        isClosedPath(racetrack, 0, dt, racetrack.perimeter, 1e-9);
    }
    public void isClosedPath(Trajectory path, double ti, double dt, double tf, double bound) {
        Vector3 vsum = new Vector3(0,0,0);
        for (double t=ti; t<tf; t+=dt) {
            TVector v = new TVector(path.getVelocity(t));
            vsum.add(v);
        }
        double error = vsum.getAbs();
        if(verbose) System.out.println("e="+vsum.toTupleString()+", |e|="+error);
        assertTrue(
                "Path Integral of velocity over a circuit should be zero for a closed path",
                error < bound
        );
    } // NOTE we might just want to extract predicates from these tests, rather than 'assert'ive test code...
}
