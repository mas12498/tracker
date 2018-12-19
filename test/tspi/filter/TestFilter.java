package tspi.filter;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import rotation.Vector3;
import tspi.model.Pedestal;
import tspi.model.PedestalModel;
import tspi.model.Polar;

/** Exercise the filter */
class TestFilter {
	
	// constant seed right now for reproducibility
	static Random random = new Random(1);

	public static void main(String args[]) {
		
		Pedestal pedestals[];
		File in = new File("./tracker/data/pedestalsIncrement.csv");
		File out = new File("./tracker/data/testFilter.csv");
		PrintStream stream = System.out;

		// initialize IO
		try {
			if (out!=null)
				stream = new PrintStream( new FileOutputStream(out) );
			pedestals = loadPedestals(in);
		}
		catch (Exception e) {
			e.printStackTrace();
			return;
		}
		
		// create the Trajectory model
		double a[] = {-10.0, 0.0, 0.0}; // mpss
		double v[] = {0.0, 300.0, 0.0}; // mps
		double p[] = {4500000.0, 0.0, 0.0}; // meter
		Trajectory trajectory = new Kinematic(
				new ArrayRealVector(a),
				new ArrayRealVector(v),
				new ArrayRealVector(p) );
		// moving west over intersection of meridian and equator at about 700m altitude?
		// we're in meters right? TODO I should just use the EFG classes to construct this correctly...
				
		// create the filter
		Filter cheat = new CheatFilter( trajectory );
		// TODO add the real filter
		// Filter kalman = new KalmanFilter( pedestals );
		
		// test the filter
		demoFilter( cheat, trajectory, pedestals, stream );
		
		//TODO devise some tests 
		
		// dispose IO
		stream.close();
	}
	
	/** Read an array of pedestals from the given file */
	public static Pedestal[] loadPedestals(File file) throws Exception {
		// use the pedestal model to parse the file
		PedestalModel model;
		model = new PedestalModel();
		model.load( file );
		
		// convert pedestal model a primitive array
		ArrayList<Pedestal> list = model.asList();
		Pedestal pedestals[] = new Pedestal[list.size()];
		list.toArray(pedestals);
		return pedestals;
	}
	
	/** runs a demo of the filter tracking a simple kinematic, . */
	public static void demoFilter(
			Filter filter, Trajectory trajectory, Pedestal pedestals[], PrintStream stream
	) {		
		// generate measurements over time 
		double start=0, end=100, dt=.02;
		for (double t=start; t<end; t+=dt) {
			
			// get the true object state
			RealVector truth = trajectory.getState( t );
			
			// take perturbed measurements
			Polar measurements[] = trackTrajectory( t, trajectory, pedestals );
			
			// update the filter with the noisy measurements
			RealVector state = filter.filter(t, measurements);
			
			
			// compare the measurements
			
			// tabulate the results into CSV
			// time
			// true state
			// state delta
			// residue
			//  pre, post
			//   az, el
			
			// TODO use descriptive statistics
			
		}
		
		//TODO extract test which compares given filter to cheating filter... 
	}
	
	/** Obtain a ideal point from the trajectory model then generate a measurement vector 
	 * by pointing each pedestal at the ideal point then perturbing it by the
	 * pedestals' error model.
	 * @param time
	 * @param trajectory the truth source for the tracked object's movement
	 * @param pedestals the sensors which will take measurements of the moving object, including error
	 * @return an array containing all the pedestals' measurements of the moving object. It's 
	 * indices correspond to the indices of the pedestal array. */
	static Polar[] trackTrajectory( double time, Trajectory trajectory, Pedestal pedestals[] ) {
		
		// find the value of the parametric model at the given time
		RealVector p = trajectory.getState(time);
		Vector3 efg = new Vector3( p.getEntry(0), p.getEntry(1), p.getEntry(2) );
		
		// have each pedestal take a measurement, including error
		Polar measurements[] = new Polar[pedestals.length];
		for (int n=0; n<pedestals.length; n++) {
			pedestals[n].pointToLocation( efg );
			measurements[n] = pedestals[n].getPerturbedLocal(random);
		}
		
		return measurements;
	} // TODO make sensors intermittently drop measurements
	
}

/** You ain't cheating you ain't trying */
class CheatFilter implements Filter {
	Trajectory cheat;
	
	public CheatFilter(Trajectory hint) {
		this.cheat = hint;
	}
	
	@Override
	public RealVector filter(double time, Polar[] measurements) {
		return cheat.getState(time);
	}

	@Override
	public Polar[] getResidualsPrediction(double dt) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Polar[] getResidualsUpdate(double dt) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public RealVector getState() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public RealMatrix getCovariance() {
		// TODO Auto-generated method stub
		return null;
	}
	
}

