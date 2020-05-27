package tspi.filter;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import tspi.model.*;
import tspi.rotation.Vector3;
import tspi.simulator.Kinematic;
import tspi.simulator.Racetrack;
import tspi.simulator.Trajectory;
import tspi.util.TVector;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

/** Exercise the filter */
class TestFilter {
	
	// constant seed right now for reproducibility
	static Random random = new Random(1);
	
	/** loads a constellation of pedestals,
	 * which track a simulated trajectory,
	 * whose perturbed outputs are fused using a Kalman filter,
	 * whose output is written to a target file,
	 * which is compatible with earlier increments. 
	 * Tracker <pedestal file> <output file> */
	public static void main(String args[]) {
		
		Pedestal pedestals[];
		File in = new File(args[0]);//pedestals file
		File out = new File(args[1]);//filter track-state file
		File nav = new File(args[2]); //nav track position plots tLLh

		// initialize track filter IO
		PrintStream stream = System.out;
		PrintStream navs = System.out;
		try {
			if ((out!=null)&&(nav!=null)) {
				stream = new PrintStream(new FileOutputStream(out));
				navs = new PrintStream(new FileOutputStream(nav));
			}
			pedestals = loadPedestals(in);
			
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
		
		//Set up track profile:
		double t0 = 0.0;   //seconds initial frame time
		double dt = 0.020; //seconds interval between frames
		int Nt = 500;      //number of frames

		//Profile Kinematics starting reference:
		TVector pos0 = new TVector(3135932.588, -5444754.209, 1103864.549); //geocentric position EFG m
		TVector vel0 = new TVector(0.0, 100.0, 0.0);                         //velocity EFG m/s
		TVector acc0 = new TVector(0.0, 0.0, 2.0);                          //acceleration EFG m/s/s
		
		//ProcessNoise for track profile 	
		double processNoise = 16; //16; 	//Q m/s/s		
		
		//track cueing offsets: 
		TVector pOff = new TVector(80,-60,-100);  //position cueing discrepency m
		TVector vOff = new TVector(8,-6,-3);      //velocity cueing discrepency m/s
		
		//initial track filter edits
		TVector p0 = new TVector(pOff.add(pos0).subtract(Pedestal.getOrigin()));     //init filter position
		TVector v0 = new TVector(vOff);                                              //init filter velocity

		// create the target trajectory
//		Trajectory trajectory = new Kinematic(
//				t0,
//				pos0.arrayRealVector(),
//				vel0.arrayRealVector(),
//				acc0.arrayRealVector()  );
		Vector3 c1 = new Vector3(0.0, 0.0,-5000.0);
		Vector3 c2 = new Vector3( 2000.0, 10000.0, -5000.0 );
		double radius = 1000.0;
		double velocity = 150.0;
		double start = 0.0;

		// set the origin to the first sensor
		Pedestal pedestal = pedestals[0];
		Ellipsoid origin = pedestal.getLocationEllipsoid();

		Racetrack trajectory = new Racetrack( start, origin, c1, c2, radius, velocity);
		int n = (int)Math.floor((trajectory.getPerimeter() / velocity) / dt); // one circuit of the racetrack


		// create Kalman 'group' filter track from pedestal instruments selected... loaded ped states
		Filter kalman = new KalmanFilter( pedestals );
//		//Filter cheat = new CheatFilter( trajectory );
		
		// test the filter on the trajectory with pedestals simulated...time,frameInsterval,frames,stream
		//NOTE: pedestal measurement models of simulation might want different from pedestals of filter.
		demoFilter( kalman, trajectory, pedestals, t0, dt, n, stream, navs );
//		//demoFilter( cheat, trajectory, pedestals, 0.0, 0.02, 500, stream ); //defined below as trivial truth passer...

		// dispose IO
//		navs.close();
		stream.close();
	}

	/** Read an array of modeled pedestals from the given file */
	public static Pedestal[] loadPedestals(File file) throws Exception {
		// use the pedestal model to parse the file
		PedestalModel model;
		model = new PedestalModel();
		model.load( file );
		
		// convert pedestal model to a primitive array
		ArrayList<Pedestal> list = model.asList();
		Pedestal pedestals[] = new Pedestal[list.size()];
		list.toArray(pedestals);

		//Assume first pedestal in file is filter origin
		Pedestal.setOrigin(pedestals[0].getLocation());
		System.out.println("ORIGIN:" + Pedestal.getOrigin().toString(3));
		
		//Compute local coordinates wrt filter origin defined 
		int pNum = pedestals.length;
		for (int p = 0; p < pNum; p++) {
			pedestals[p].setLocalOriginCoordinates();
		}
		
		//return list with pedestals located and filter origin defined:
		return pedestals;
	} //TODO should extract this to some ensemble class, gathered with mass pointing and error perturbation. 
	
	/** Applies the given filter to a simulated set of track data. The target's
	 * motion is simulated using a {@link Trajectory Trajectory object},
	 * and each {@link tspi.model.Pedestal Pedestal} is pointed at the object
	 * and perturbed by their error model. The array of noisy pedestal
	 * measurements are then given to the filter incrementally over an interval
	 * of time. */
	public static void demoFilter(
            Filter filter, Trajectory trajectory, Pedestal pedestals[],
            double t0, double dt, int n, PrintStream stream, PrintStream navs ) {

		Ellipsoid trueNav = new Ellipsoid();
		Vector3 nav = new Vector3(Vector3.EMPTY);

		// print the headers
		navs.append("time, NLat, ELon, eHgt");
		navs.println();

		stream.append("time, S0, S1, S2, S3, S4, S5, S6, S7, S8, "
				+ "dS0, dS1, dS2, dS3, dS4, dS5, dS6, dS7, dS8");
		for (Pedestal pedestal : pedestals) {
			stream.append(", " + pedestal.getSystemId() + "_rg");
			stream.append(", " + pedestal.getSystemId() + "_az");
			stream.append(", " + pedestal.getSystemId() + "_el");
		}
		stream.println();

		// Generate stream (measurements over time)
		for (double t = t0; t < t0 + n * dt; t += dt) {

			// get the true object state
			RealVector truth = trajectory.getState(t);

			// get the true nav plot
			nav.set(truth.getEntry(0), truth.getEntry(1), truth.getEntry(2)); //functional to get LLh
			trueNav.setGeocentric(nav);

			//tabulate nav plots into csv
			navs.append((Double.toString(t)));
			navs.append(", " + trueNav.getNorthLatitude() + ", " + trueNav.getEastLongitude() + ", " + trueNav.getEllipsoidHeight()); //nav plot data
			navs.println();

			// take perturbed measurements
			trajectory.simulateTrack(t, pedestals, random);
//			// Propagate perturbed measurements... replaced 'simulateTrack' with free partial model of just 'track'...
//			trajectory.track( t, pedestals, random );

			// update the filter with the noisy measurements
			RealVector state = filter.filter(t, pedestals).copy(); // just added a copy to make sure I wasn't clobbering any leaked state...

			// compare the measurements
			state.subtract(truth);

			// tabulate the results into CSV
			stream.append(Double.toString(t)); // time

			for (double d : truth.toArray())
				stream.append(", " + d);// true state

			for (double d : state.toArray())
				stream.append(", " + d);// state delta

			// Do we want to include residuals?
//			RealVector residuals = filter.getResiduals();
//			RealVector innovations = filter.getInnovations();

			stream.println();
		}
		// TODO use descriptive statistics and print a summary to screen ? Use them for unit test?
		/* Do we want to adapt this into a test? How can we do that?
		 *  - make sure tracker state is within some epsilon of the truth?
		 *  - monitor the internal residuals of the filter? */
	}
}
