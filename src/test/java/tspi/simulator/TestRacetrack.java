package tspi.simulator;

import junit.framework.TestCase;
import tspi.model.Ellipsoid;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;
import tspi.simulator.Racetrack;
import tspi.simulator.Trajectory;
import tspi.util.TVector;

/** Trying to make some sanity tests for the 'racetrack' type target simulator.
 * There are some ideas here that could apply to arbitrary paths from any simulator;
 * like tests for uniform motion, or closed paths, or basic continuity checks.
 * @author Casey
 * */
public class TestRacetrack extends TestCase {

    double lat=10.0, lon=-130, eh=10000;
    Ellipsoid origin = new Ellipsoid(Angle.inDegrees(lat), Angle.inDegrees(lon), eh);
    boolean verbose = true;
    double radius = 1.0;
    double velocity = 1.0;
    Vector3 c1 = new Vector3(0,0,0);
    Vector3 c2 = new Vector3(0,2,0);
    Racetrack racetrack = new Racetrack(0.0, origin, c1, c2, radius, velocity);
    int n = 5000;
    double dt = racetrack.getPerimeter() / (velocity * n);
    // TODO tests methods should accept racetrack and some parameters as an argument so we can test a variety of configurations

    /** Make sure none of the piecewise-defined parts of the racetrack are disjoint */
    public void testContinuity() {
        double endtime = 2 * racetrack.getPerimeter() / racetrack.getVelocity();
        double rotationEpsilon = 0.000000001;
        double bounds = velocity * dt + rotationEpsilon;
        continuity(racetrack, 0, endtime, dt, bounds);
    }

    /** ensures the racetrack motion really does have uniform speed */
    public void testConstantSpeed() {
        double endtime = racetrack.getPerimeter() / racetrack.getVelocity();
        double epsilon = 0.000000000001;
        uniformMotion(racetrack, 0, endtime, dt, velocity, epsilon);
    }

    /** the analytic instantaneous velocity should compare well with numerical differentiation if sampled densely enough.
     * This should be made into a general test for all trajectories, and acceleration...*/
    public void testVelocityNumerically() {
        // crud, I don't know, how much can we expect velocity and finite differences to agree?
        double bound = dt*dt*velocity*velocity; // sure, let's go with that. Keep in mind this changes with sampling rates, velocity, the order of the function, etc.
        double endtime = racetrack.getPerimeter() / racetrack.getVelocity();
        differentiatePosition(racetrack, 0.0, endtime, dt, bound);
    }
    public void testAccelerationNumerically() {
        double bound = dt*velocity*velocity/radius; // max acceleration when turn starts?
        double endtime = racetrack.getPerimeter() / racetrack.getVelocity();
        differentiateVelocity(racetrack, 0.0, endtime, dt, bound);
    }

    /** Furthermore the difference in position over an interval should match the sum of densely sampled velocities over that interval. */
    public void testPositionNumerically() {
        double endtime = racetrack.getPerimeter() / racetrack.getVelocity();
        double epsilon = 0.000000000001;
        integrateVelocity(racetrack, 0, endtime, n, epsilon);
    }

    /** Make sure successive points within some time bound are within some position bound.
     * Should be generalized to handle velocity as well... */
    public void continuity(Racetrack path, double t0, double t1, double dt, double bound) {
        if (verbose) System.out.println( "\nt\tPi\tPj\tPk\t|dP|");

        Vector3 p0 = new TVector( path.getPosition(t0) );
        if(verbose) System.out.println( "0.0\t"+p0.toString()+"\t0.0");

        for (double t=dt; t<t1; t+=dt) {
            Vector3 p1 = new TVector( path.getPosition(t) );
            double d = new Vector3(p1).subtract(p0).getAbs();

            if (verbose) System.out.println( t+"\t"+p0.toString()+"\t"+d);

            if (d>bound) System.out.println("error");
            assertTrue(
                    "subsequent points violate velocity bounds at t="+t+", d="+d,
                    (d <= bound)
            );
            p0 = p1;
        }
    }

    /** Sample the path over the specified interval, ensuring that velocity magnitude remains constant */
    public void uniformMotion(Racetrack path, double t0, double t1, double dt, double velocity, double bound) {
        if (verbose) System.out.println("target speed:"+velocity+"\nt\tVi\tVj\tVk\t|dV|");

        for (double t=0; t<t1; t+=dt) {
            Vector3 v = new TVector( path.getVelocity(t) );
            double speed = v.getAbs();

            if(verbose) System.out.println( t+"\t"+v.toString()+"\t"+speed);
            assertTrue(
                    "velocity differs too greatly to be uniform motion",
                    (Math.abs(speed - Math.abs(velocity)) <= bound)
            );
        }
    }

    /** Sample the path over the specified interval, and ensure positional finite differences and
     * analytic velocity agree to within a given bound. */
    public void differentiatePosition(Racetrack path, double t0, double t1, double dt, double bound) {
        if (verbose) System.out.println("t\tP(t)\tV(t)\t|dP-dt*V|");

        // incrementally sample motion
        Vector3 p0 = new TVector( path.getPosition(t0) );
        for (double t=dt; t<t1; t+=dt) {

            // compute finite difference as an estimate for velocity
            Vector3 p1 = new TVector( path.getPosition(t) );
            Vector3 dp = new Vector3(p1).subtract(p0);
            // might consider interpolating from more points for better bounds...

            // use analytic velocity at endpoint as your truth
            Vector3 v = new TVector( path.getVelocity(t) );

            // find the error
            Vector3 e = new Vector3(v).multiply(dt).subtract(dp);
            double error = e.getAbs();

            if (verbose) System.out.println(t+"\t"+p1+"\t"+v+"\t"+error);
            assertTrue(
                    "finite differences varied more than |e|="+bound+
                            " from analytic velocity at t="+t+", e="+e.toTupleString(),
                    (error<bound)
            );
            p0 = p1;
//            v = new Vector3( path.getVelocityVector(t) );
        }
    }

    public void differentiateVelocity(Racetrack path, double t0, double t1, double dt, double bound) {
        if (verbose) System.out.println("t\tV(t)\tdt*A(t)\t|dV-dt*A|");

        // incrementally sample motion
        Vector3 v0 = new TVector( path.getVelocity(t0) );
        for (double t=dt; t<t1; t+=dt) {

            // compute finite difference as an estimate for velocity
            Vector3 v1 = new TVector( path.getVelocity(t) );
            Vector3 dv = new Vector3(v1).subtract(v0);
            // might consider interpolating from more points for better bounds...

            // use analytic velocity at endpoint as your truth
            Vector3 a = new TVector( path.getAcceleration(t) );//.multiply(dt);

            // find the error
            Vector3 e = new Vector3(a).multiply(dt).subtract(dv);
            double error = e.getAbs();

            if (verbose) System.out.println(t+"\t"+v1+"\t"+a+"\t"+error);
            assertTrue(
                    "finite differences varied more than |e|="+bound+
                            " from analytic velocity at t="+t+", e="+e.toTupleString(),
                    (error<bound)
            );
            v0 = v1;
        }
    }

    /**  Sample the path over the given interval, ensuring the sum of velocity agrees with the positional difference. */
    public void integrateVelocity(Trajectory path, double t0, double t1, int samples, double bound) {
        // compute Velocity integral using the fundamental theorem of calculus
        Vector3 p0 = new TVector(path.getPosition(t0));
        Vector3 p1 = new TVector(path.getPosition(t1));
        Vector3 dp = new Vector3(p1).subtract(p0);

        // Numerically integrate velocity in a naive fashion
        Vector3 vsum = new Vector3(0,0,0);
        for (double k=0; k<samples; k++) {
        // for (double t=t0; t<t1; t+=dt) { // more round-off error doing it this way...
            double t = t0 + (k/samples)*(t1-t0);
            Vector3 v = new TVector(path.getVelocity(t));//.multiply(dt);
            vsum.add(v);
        }
        vsum.divide(samples);

        // compare the error between the two methods
        Vector3 e = new Vector3(dp).subtract(vsum);
        double error = e.getAbs();
        if (verbose) System.out.println("samples = "+samples+"\ndP = "+dp+"\nsum(dV) = "+vsum+"\n|e| = "+error);
        assertTrue(
                "Path Integral of velocity over a circuit should be zero for a closed path",
                error < bound
        );
    } // NOTE we might just want to extract predicates from these tests, rather than 'assert'ive test code...
}
