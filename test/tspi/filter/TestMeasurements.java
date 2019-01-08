package tspi.filter;

import java.io.File;
import java.io.PrintStream;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.util.TVector;

//trajectory -> targets file
//target + ensemble -> measurements file
//measurements file + filter -> ?

/** This example is for applying a Filter against the pre-recorded measurements of an ensemble.
 * It also provides the ability to generate some example measurement data using a
 * {@link tspi.filter.Trajectory Trajectory} and a Pedestal ensemble. */
public class TestMeasurements {

	static Random random = new Random(1);
	static final String separator = ", ";
	static NumberFormat numberFormat = new DecimalFormat("0.00000000");
	
	/** Generates an example measurement file
	 * usage: TestMeasurements <pedestal file> <output file> */
	// TestMeasurements "C:\Users\shiel\Documents\workspace\tracker\data\pedestalsIncrement.csv" "C:\Users\shiel\Documents\workspace\tracker\data\measurementsTrajectory.csv"
	public static void main(String[] args) {
		
		File pedestals = new File(args[0]);
		double t0 = 0.0;
		double dt = 1.0;
		int n = 100;
		File measurements = new File(args[1]);
		
		try {
			Ensemble ensemble = Ensemble.load(pedestals);
			PrintStream stream = new PrintStream( measurements );
			Trajectory trajectory = getTrajectory();
			createMeasurements(trajectory, ensemble, t0, dt, n, stream);
			
		} catch (Exception exception) {
			exception.printStackTrace();
		}
	}

	/** Creates a CSV file containing a sequence of measurement vectors.
	 * the first column is time and subsequent columns are the polar measurements in the order of the 
	 * sensors in the pedestal array. */
	public static void createMeasurements(
			Trajectory trajectory,
			Ensemble ensemble,
			double t0, double dt, int n,
			PrintStream stream
	) {
		// print a file header appropriate for the given ensemble
		stream.append("time");
		for (Pedestal pedestal : ensemble) {
			if (pedestal.getMapAZ())
				stream.print( separator + pedestal.getSystemId() + "_az" );
			
			if (pedestal.getMapEL())
				stream.print( separator + pedestal.getSystemId() + "_el" );
			
			if (pedestal.getMapRG())
				stream.print( separator + pedestal.getSystemId() + "_rg" );
		}
		stream.println();
		
		// iterate over the specified interval
		double t = t0;
		for (int i=0; i<n; i++) {
			
			// print the time
			stream.append( ""+t );
			
			// compute the current location of the trajectory
			TVector position = new TVector( trajectory.getPosition( t ) );
			
			// point the sensors at that point
			ensemble.point(position, random);
									
			// output all sensor measurements used in the ensemble
			for (Pedestal pedestal : ensemble) {
				Polar measure = pedestal.getLocal();
				
				// output value according to sensor model
				if (pedestal.getMapAZ())
					stream.append( separator + numberFormat.format( measure.getUnsignedAzimuth().getDegrees() ) );
				
				if (pedestal.getMapEL())
					stream.append( separator + numberFormat.format( measure.getElevation().getDegrees() ) );
				
				if (pedestal.getMapRG())
					stream.append( separator + numberFormat.format( measure.getRange() ) );
			}
			
			stream.println();
			t+=dt;
		}
		
	} // TODO this is very similar to Ensemble test.generate- how can we remove duplicate code...
	// TODO a stream might be a more natural intermediate representation than an array or file...
	
	/** I still don't know the best way to construct this, so I'm just using this in the interim. */
	public static Trajectory getTrajectory() {
		//Set up track profile:
		double t0 = 0.0;   //seconds initial frame time
//		double dt = 0.020; //seconds interval between frames
//		int Nt = 1000;      //number of frames
		
		//Profile Kinematics starting reference:
		TVector pos0 = new TVector(3135932.588, -5444754.209, 1103864.549); //geocentric position EFG m
		TVector vel0 = new TVector(0.0, 10.0, 0.0);                         //velocity EFG m/s
		TVector acc0 = new TVector(0.0, 0.0, 2.0);                          //acceleration EFG m/s/s
		
		// create the target trajectory
		Trajectory trajectory = new Kinematic(
				t0,
				pos0.arrayRealVector(),
				vel0.arrayRealVector(),
				acc0.arrayRealVector());
		
		return trajectory;
	}
}
