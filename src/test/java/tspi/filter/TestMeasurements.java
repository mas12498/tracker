package tspi.filter;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Angle;
import tspi.simulator.Kinematic;
import tspi.simulator.Trajectory;
import tspi.util.TVector;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintStream;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

//trajectory -> targets file
//target + ensemble -> measurements file
//pedestals + measurements + filter -> tracking file?

/** This example is for applying a Filter against the pre-recorded measurements of an ensemble.
 * It also provides the ability to generate some example measurement data using a
 * {@link Trajectory Trajectory} and a Pedestal ensemble. */
public class TestMeasurements {

	static Random random = new Random(1);
	static final String separator = ", ";
	static NumberFormat numberFormat = new DecimalFormat("0.00000000");
	static final String usage = "usage: TestMeasurements (pedestal file) (measurement file) [tracking output]"; 
	
	/** Generates an example measurement file
	 * usage: TestMeasurements (pedestal file) (measurement file) [tracking output]
	 * Two arguments outputs a measurement file using a default Trajectory.
	 * Three arguments generates a tracking solution file from the pedestal and measurement file using a default filter...*/
	// TestMeasurements 
	//       "C:\Users\shiel\Documents\workspace\tracker\data\pedestalsIncrement.csv" 
	//       "C:\Users\shiel\Documents\workspace\tracker\data\measurementsTrajectory.csv" 
	//       "C:\Users\shiel\Documents\workspace\tracker\data\tracking.csv" 
	
	//		"H:/filterPedestals001.csv"
	//      "H:/measurementsTrajectory.csv"
	//      "H:/tracking.csv" 
	
	public static void main(String[] args) {
		
		double t0 = 0.0;
		double dt = 0.020;
		int n = 80;
		
		try {
		
			if (args.length!=2 && args.length!=3)
				throw new Exception(usage);
			
			File pedestals = new File( args[0] );
			Ensemble ensemble = Ensemble.load( pedestals );
			
			File measurements = new File( args[1] );
			
			if (args.length==2) {
				Trajectory trajectory = getTrajectory(  );
				createMeasurements( trajectory, ensemble, t0, dt, n, measurements );
			}
			
			else if (args.length==3) {
				File tracking = new File( args[2] );
				Filter filter = getFilter( ensemble );
				track( ensemble, measurements, filter, tracking );	
			}
			
		} catch (Exception exception) {
			exception.printStackTrace();
		}
	}

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
	
	/** TODO this needs to be configurable too... */
	public static Filter getFilter(Ensemble ensemble ) {
		
		//Profile Kinematics starting reference:
		TVector pos0 = new TVector(3135932.588, -5444754.209, 1103864.549); //geocentric position EFG m
		TVector vel0 = new TVector(0.0, 10.0, 0.0);                         //velocity EFG m/s
		TVector acc0 = new TVector(0.0, 0.0, 2.0);                          //acceleration EFG m/s/s
		
		//ProcessNoise for track profile 	
		double processNoise = 10; 	//Q m/s/s		
		
		//track cueing offsets: 
		TVector pOff = new TVector(800,-600,-1000);  //position cueing discrepency m
		TVector vOff = new TVector(80,-60,-30);      //velocity cueing discrepency m/s
		
		//initial track filter edits
		TVector p0 = new TVector(pOff.add(pos0).subtract(Pedestal.getOrigin()));     //init filter position
		TVector v0 = new TVector(vOff);
		
		Filter kalman = new KalmanFilter( ensemble.toArray() ); // , p0, v0); //, processNoise );
		
		return kalman;
	}
	
	/** Iteratively updates a filter using pre-recorded pedestal measurements. */
	public static void track(
			Ensemble ensemble,
			File measurements,
			Filter filter,
			File track // TODO do we really want to accept some stream instead of a file...
	)
		throws Exception
	{
		BufferedReader reader = new BufferedReader( new FileReader( measurements ) );
		PrintStream stream = new PrintStream( track );
		
		// consume the header
		String header = reader.readLine( );
		
		// read each line
		String line = reader.readLine();
		while (line!=null) {
			
			String values[] = line.split( separator );
			
			double time = Double.parseDouble( values[0] );
			
			// copy measurements from the file to the ensemble
			int n = 1;
			for (Pedestal pedestal : ensemble) {
				Polar measure = pedestal.getLocal();
				if (pedestal.getMapAZ()) {
					double az = Double.parseDouble( values[n++] );
					measure.setAzimuth( Angle.inDegrees( az ) );
				}
				if (pedestal.getMapEL()) {
					double el = Double.parseDouble( values[n++] );
					measure.setElevation( Angle.inDegrees( el ) );
				}
				if (pedestal.getMapRG()) {
					double rg = Double.parseDouble( values[n++] );
					measure.setRange( rg );
				}
				pedestal.point(measure);
			}
			
			// Update the filter with the adjusted Ensemble measurements
			RealVector state = filter.filter( time, ensemble.toArray() );
			
			// print out the time and state.
			stream.append( "" + time );
			for (double d : state.toArray())
				stream.append( separator + d );
			stream.println();
			
			//TODO do we need to output residuals
			
			// advance to the next line
			line = reader.readLine();
		}
		
		// close the files
		reader.close();
		stream.close();
	}
	
	/** Creates a CSV file containing a sequence of measurement vectors.
	 * the first column is time and subsequent columns are the polar measurements in the order of the 
	 * sensors in the pedestal array. */
	public static void createMeasurements(
			Trajectory trajectory,
			Ensemble ensemble,
			double t0, double dt, int n,
			File measurements
			
	)
		throws Exception
	{
		PrintStream stream = new PrintStream( measurements );
		
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
		
		stream.close();
	} // TODO this is very similar to Ensemble test.generate- how can we remove duplicate code...
	// TODO a stream might be a more natural intermediate representation than an array or file...
	
}
