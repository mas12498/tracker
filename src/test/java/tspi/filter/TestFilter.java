package tspi.filter;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import tspi.model.*;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;
import tspi.simulator.*;
import tspi.util.TVector;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

// TODO the simulator already generates a truth nav file- should we remove it from this one?

/** Applies a KalmanFilter to a sequence of Observations */
class TestFilter {

	KalmanFilter filter;
	Observations observations;

	public TestFilter(Observations observations) {
		this.observations = observations;
		this.filter = new KalmanFilter( observations.getEnsemble() );
	}

	KalmanFilter getFilter() { return this.filter; }
	Observations getObservations() { return this.observations; }

	void setFilter(KalmanFilter filter) { this.filter = filter; }
	void setObservations(Observations observations) { this.observations = observations; }

	/** Creates two files describing the filter performance and true trajectory as the filter tracks the observations. */
	public void track( PrintStream stream, PrintStream navs ) {

		// print the headers
		navs.println("time, NLat, ELon, eHgt");
		stream.append("time, S0, S1, S2, S3, S4, S5, S6, S7, S8, "
				+ "dS0, dS1, dS2, dS3, dS4, dS5, dS6, dS7, dS8");
		stream.println();

		// for every set of observations
		while (observations.hasNext()) {
			Ensemble ensemble = observations.next();
			Double time = observations.getTime();

			// get the true nav plot
			Vector3 t = observations.getTruth();
			double[] ta = {t.getX(), t.getY(), t.getZ()};
			ArrayRealVector truth = new ArrayRealVector(ta);
			Ellipsoid llh = new Ellipsoid();
			llh.setGeocentric(t);

			// tabulate the current true target position
			navs.append(Double.toString(observations.getTime()));
			navs.append(", " + llh.getNorthLatitude() + ", " + llh.getEastLongitude() + ", " + llh.getEllipsoidHeight()); //nav plot data
			navs.println();

			// update the filter with the noisy measurements
			RealVector state = filter.filter(time, ensemble).copy(); // just added a copy to make sure I wasn't clobbering any leaked state...

			// compare the filter state with the truth
			state.subtract(truth);

			// tabulate the filter results into CSV
			stream.append(Double.toString(time)); // time
			for (double d : truth.toArray())
				stream.append(", " + d);// true state
			for (double d : state.toArray())
				stream.append(", " + d);// state delta

			// Do we want to include residuals?
//			RealVector residuals = filter.getResiduals();
//			RealVector innovations = filter.getInnovations();

			stream.println();
		}
	}
	// TODO use descriptive statistics and print a summary to screen ? Use them for unit test?
	/* Do we want to adapt this into a test? How can we do that?
	 *  - make sure tracker state is within some epsilon of the truth?
	 *  - monitor the internal residuals of the filter? */

	/** loads a constellation of pedestals and their associated observations as they track a target.
	 * Their measurements are combined in a Kalman filter, whose state is written out to a file as observations are added.
	 *
	 * Tracker <pedestal input> <observations input> <state output> <truth output> */
	public static void main( String args[] ) {

		File pedestals = new File(args[0]);
		File observations = new File(args[1]);
		PrintStream stream = System.out;
		PrintStream navs = System.out;

		ObservationsReader reader;

		try {
			if (args[2] != null)
				stream = new PrintStream(new FileOutputStream(new File(args[2])));
			if (args[3] != null)
				navs = new PrintStream(new FileOutputStream(new File(args[3])));

			reader = new ObservationsReader(pedestals, observations);

			Pedestal pedestal = reader.getEnsemble().getOrigin();
			Ellipsoid origin = pedestal.getLocationEllipsoid();
			System.out.println("ORIGIN:" + pedestal.getLocation().toString(3));

			TestFilter test = new TestFilter(reader);
			test.track(stream, navs);

			// dispose IO
			// navs.close();
			stream.close();

		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
	}

//	public Trajectory getKinematic() {
//
//		//Set up track profile:
//		double t0 = 0.0;   //seconds initial frame time
//		double dt = 0.020; //seconds interval between frames
//		int Nt = 500;      //number of frames
//
//		//Profile Kinematics starting reference:
//		TVector pos0 = new TVector(3135932.588, -5444754.209, 1103864.549); //geocentric position EFG m
//		TVector vel0 = new TVector(0.0, 100.0, 0.0);                         //velocity EFG m/s
//		TVector acc0 = new TVector(0.0, 0.0, 2.0);                          //acceleration EFG m/s/s
//
//		//ProcessNoise for track profile
//		double processNoise = 16; //16; 	//Q m/s/s
//
//		//track cueing offsets:
//		TVector pOff = new TVector(80, -60, -100);  //position cueing discrepency m
//		TVector vOff = new TVector(8, -6, -3);      //velocity cueing discrepency m/s
//
//		//initial track filter edits
//		TVector p0 = new TVector(pOff.add(pos0).subtract(Pedestal.getOrigin()));     //init filter position
//		TVector v0 = new TVector(vOff);                                              //init filter velocity
//
//		// create the target trajectory
//		Trajectory trajectory = new Kinematic(
//				t0,
//				pos0.arrayRealVector(),
//				vel0.arrayRealVector(),
//				acc0.arrayRealVector());
//		return trajectory;
//	}
//
//	public Racetrack getRacetrack() {
//
//		Vector3 c1 = new Vector3(0.0, 0.0,-5000.0);
//		Vector3 c2 = new Vector3( 2000.0, 10000.0, -5000.0 );
//		double radius = 1000.0;
//		double velocity = 150.0;
//		double start = 0.0;
//		double dt = 0.020; //seconds interval between frames
//
//		Ellipsoid origin = new Ellipsoid(Angle.inDegrees(10.0), Angle.inDegrees(-60.0), 0);
//
//		Racetrack trajectory = new Racetrack( start, origin, c1, c2, radius, velocity);
//		int n = (int)Math.floor((trajectory.getPerimeter() / velocity) / dt); // one circuit of the racetrack
//
//		return trajectory;
//	}
}
