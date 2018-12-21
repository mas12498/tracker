package tspi.filter;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import tspi.model.Pedestal;
import tspi.model.PedestalModel;
import tspi.model.Polar;

/** Exercise the filter */
class TestFilter {
	
	// constant seed right now for reproducibility
	static Random random = new Random(1);

	public static void main(String args[]) {
//		double[]vd = {1,2,3};
//		double[][] ad = {{1,0,1},{0,1,2},{0,0,3}};
//		RealVector v = MatrixUtils.createRealVector(vd);
//		RealMatrix a = MatrixUtils.createRealMatrix(ad);
//		RealVector y = a.operate(v);
//		RealVector  w = a.preMultiply(v);
		
		Pedestal pedestals[];
		File in = new File("H:/git/mas12498/tracker/data/pedestalsIncrement.csv");
		File out = null;//new File("/home/mike/photon/workspace/github/tracker/data/testFilter.csv");
//		File in = new File("./tracker/data/pedestalsIncrement.csv");
//		File out = null;//new File("./tracker/data/testFilter.csv");
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
		Trajectory trajectory = new Kinematic( 0.0, // t0
				3146814.7105773017, -5435097.18852511, 1114582.200504945, // p0
				0.0, 0.0, 10.0, // v0
				0.0, 2.0, 0.0 ); // a0 
//		Trajectory trajectory = new Kinematic( 0.0, // t0
//				3146814.7105773017, 0.0, 0.0, // p0
//				0.0, 300.0, 0.0, // v0
//				-10.0, 0.0, 0.0 ); // a0 
		// moving west over intersection of meridian and equator at about 700m altitude?
		// we're in meters right? TODO I should just use the EFG classes to construct this correctly...
				
		// create the filter
		Filter cheat = new CheatFilter( trajectory );
		
		// TODO add the real filter
		Filter kalman = new KalmanFilter( pedestals );
		
		// test the filter
		//demoFilter( cheat, trajectory, pedestals, 0.0, 0.02, 500, stream );
		demoFilter( kalman, trajectory, pedestals, 0.0, 0.02, 500, stream );
		
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
			Filter filter, Trajectory trajectory, Pedestal pedestals[],
			double t0, double dt, int n, PrintStream stream
	) {
		// print the header
		stream.append("time, S0, S1, S2, S3, S4, S5, S6, S7, S8, "
				+ "dS0, dS1, dS2, dS3, dS4, dS5, dS6, dS7, dS8");
		for (Pedestal pedestal : pedestals) {
			stream.append( ", "+pedestal.getSystemId()+"_az");
			stream.append( ", "+pedestal.getSystemId()+"_el");
		}
		stream.println();
					
		// generate measurements over time
		for (double t=t0; t<t0+n*dt; t+=dt) {
			
			// get the true object state
			RealVector truth = trajectory.getState( t );
			
			// take perturbed measurements
			/////Polar measurements[] = trajectory.track( t, pedestals, random );
			trajectory.simulateTrack( t, pedestals, random );
			
			// update the filter with the noisy measurements
			RealVector state = filter.filter(t, pedestals);
			//RealVector state = KalmanFilter.filter(t, pedestals);
			
			// compare the measurements
			state.subtract( truth );
			
			// tabulate the results into CSV
			stream.append(Double.toString(t)); // time
			
			for (double d : truth.toArray())
				stream.append(", "+d);// true state
			
			for (double d : state.toArray())
				stream.append(", "+d);// state delta
			
			// TODO I don't think if residue is meaningful for the cheatFilter...
			// put this back in when you're ready?
//			Polar[] residue = filter.getResidualsPrediction(t);
//			for(Polar polar : residue) {
//				stream.append( ", "+polar.getSignedAzimuth() );
//				stream.append( ", "+polar.getElevation() );
//			} // pre
//			
//			residue = filter.getResidualsUpdate(t);
//			for(Polar polar : residue) {
//				stream.append( ", "+polar.getSignedAzimuth() );
//				stream.append( ", "+polar.getElevation() );
//			} // post
			
			stream.println();
		}
		// TODO use descriptive statistics and print a summary to screen ? Use them for unit test?
	}
	
}


/** Uses the Trajectory model to get the state, therefore all errors should be zero. */
class CheatFilter implements Filter {
	double time;
	Trajectory cheat;
	
	public CheatFilter(Trajectory hint) {
		//just pass it thru...
		this.cheat = hint;
	}
	
	
	
	@Override
	public RealVector filter(double time, Pedestal[] measurements) {
		this.time = time;
		//below is replaced by the filter state[?]
		return cheat.getState(time);
	}

	
	
	@Override
	public RealVector getState() {
		return cheat.getState(time);
	}

	@Override
	public RealMatrix getCovariance() {
		// TODO Auto-generated method stub
		return null;
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

}

