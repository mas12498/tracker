package tspi.controller;

import java.io.File;
import java.io.PrintStream;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

import rotation.Angle;
import tspi.model.Pedestal;
import tspi.model.PedestalModel;
import tspi.model.Polar;
import tspi.model.Solution;
import tspi.model.Target;
import tspi.model.TargetModel;

/** I'm getting confused on the architecture; I think Solution got conflated with 
 * Target to make some view tasks convenient. Probably my fault. Since all these 
 * iterations are one-off demonstrators, I wasn't too worried about it- however;
 * We should really take careful stock of our architecture before starting a 
 * large NRT application.
 * 
 * To avoid kludging to meet another demonstrator or deep conflicts with the 
 * master branch, I just wrote stuff rather procedurally here to avoid messing
 * with the model objects right now in interest of getting the data out...
 */
public class MonteCarloTest {
	
	// note our random number generator always uses the same seed for repeatability...
	private static Random Generator = new Random( 1L );
	// TODO might want to use the commons distributions API instead...
	 
	public static void main(String args[]) {
		// Adjust these values; let me know if this is what you need.
		long time = 0;
		double lat = 10.00, lon = -60.00, height=1000.0;
		double step = 0.01;
		int count = 20, trials = 1000;
		String input = ".\\data\\pedestalsWithOrigin.csv"; //"C:\Users\shiel\Documents\workspace\tracker\data\pedestalsWithOrigin.csv"
		String output = ".\\data\\GridTest100.csv";
		
		// I need the origin as a pedestal, so I can point it and compute spherical errors against Solutions.
		// you did want spherical errors from the synthetic origin right? Not EFG?
		Pedestal origin = new Pedestal( "origin", true, true, 
				Angle.inDegrees(0.0), Angle.inDegrees(0.0), 0.0);
		
		// I don't understand the origin name convention; is it tied specifically to increment3 for being able to change origin on selection? 
		// I think we should just add a header bar that lets you select it from a drop down whose list model is derived from the pedestal model... 
		// I don't know, we'll talk it through.
		
		// I don't think I need this, because I'm computing all errors by pointing an origin pedestal...
		Pedestal.setOrigin( origin.getLocation() );
		// I feel the origin should be an explicit argument more than a global state...
		// for example, to get local coordinates you might add pedestal.getRelative(origin)
		// then whatever application is running it would track the origin, whether it is a gui or Solution etc...
		// this would prevent subtle global interactions, and reduces the complexity of Pedestal...
		
		PedestalModel model = new PedestalModel();
		try {
			PrintStream stream = new PrintStream( new File(output) );
			model.load( new File(input) );
			testGrid(
					time, lat, lon, height,
					step, count, trials, 
					origin, model.asList(), stream);
			
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	public static void testGrid(
			long time, double lat, double lon, double height,
			double step, int count, int trials, 
			Pedestal origin, List<Pedestal> pedestals, PrintStream stream )
	{
		// print the header
		stream.append("Lat, Lon, H, AZmin, AZmean, AZmode, AZmax, AZdev, ELmin, ELmean, ELmode, ELmax, ELdev, Rmin, Rmean, Rmode, Rmax, Rdev\n");
		stream.flush();
		
		// declare statistics for az, el, r in that order
		DescriptiveStatistics statistics[] = new DescriptiveStatistics[3];
		for (int n=0; n<3; n++)
			statistics[n] = new DescriptiveStatistics();
		// unfortunately there is no lib for multivariate descripive stats -i.e. stats containing in memory percentile methods
		
		// generate a grid of targets
		TargetModel model = generateTargetGrid( time, lat, lon, height, count, step );
		
		// For every target on that grid
		for (Target target: model) {
			
			// initialize stats for new round of trials
			for (DescriptiveStatistics statistic:statistics)
				statistic.clear();
			
			// compute the correct, unperturbed pointing coordinates
			origin.pointToLocation( target.getGeocentricCoordinates() );
			Polar truth = origin.getLocal();

			// for every trial
			for( int trial=0; trial<trials; trial++ ) {
				
				// point the pedestals at the target with a random perturbation generated from their error model
				pointPedestals(pedestals, target);
				
				// Compute a target solution, and point the origin at it 
				Solution solution = new Solution( pedestals );
				origin.pointToLocation( solution._position_EFG );
				Polar measurement = origin.getLocal();
				// TODO make sure origin is right in this solution!!!
				// TODO Solution pretty much needs to start from scratch every time right? I shouldn't be re-using something right?
				
				// Now compute error in spherical terms
				double error[] = {
						measurement.getUnsignedAzimuth().subtract( truth.getUnsignedAzimuth() ).getDegrees(),
						measurement.getElevation().subtract( truth.getElevation() ).getDegrees(),
						measurement.getRange() - truth.getRange()
				};
				
				// and update the descriptive statistics
				for (int n=0; n<3; n++)
					statistics[n].addValue(error[n]);
				// TODO SummaryStatistics has versions with contributing hierarchies and synchronization;
				// if we want to try getting a multi-threading speed boost for larger tests...
				// hell, it might be feasible to progressively refine a plot as trials are run if we make one of these per grid point...
			}
			
			// print out the min, mean, mode and max of errors in AER
			Character separator = ',';
			stream.append( Double.toString( target.getLatitude().getDegrees() ) );
			stream.append( separator );
			stream.append( Double.toString( target.getLongitude().getDegrees() ) );
			stream.append( separator );
			stream.append( Double.toString( target.getHeight() ) );
			for (int i=0; i<3; i++) {
				stream.append( separator );
				stream.append( Double.toString( statistics[i].getMin() ) );
				stream.append( separator );
				stream.append( Double.toString( statistics[i].getMean() ) );
				stream.append( separator );
				stream.append( Double.toString( statistics[i].getPercentile(50.0) ) );
				stream.append( separator );
				stream.append( Double.toString( statistics[i].getMax() ) );
				stream.append( separator );
				stream.append( Double.toString( statistics[i].getStandardDeviation() ) );
			}
			stream.append("\r\n"); // TODO should we extract more info from Solution too?
		}
	}
	
	public static TargetModel generateTargetGrid(
			long time, double lat, double lon, double height,
			int count, double step
	) {
		TargetModel model = new TargetModel();
		for (int i=-count; i<=count; i++) {
			for (int j=-count; j<=count; j++) {
				Target target = new Target(time, lat+(i*step), lon+(j*step), height);
				model.add(model.getRowCount(), target);
			}
		}
		return model;
	}
	
	public static void pointPedestals( List<Pedestal> model, Target target) {
		for (Pedestal pedestal : model) {
			pedestal.pointToLocation( target.getGeocentricCoordinates() );
			Polar pertubed = pedestal.getPerturbedLocal( Generator );
			pedestal.point(pertubed);
		}
	}

	// meh, we might eventually want to produce some model for these results when we start passing them to a gui...
//	class GridAccuracy {
//		long time;
//		Angle lat, lon;
//		double height;
//		
//		final int azimuth=0, elevation=1, range=2;
//		final int min=0, mean=1, mode=2, max=3, deviation=4;
//		double stat[][] = new double [3][5];
//				
//		GridAccuracy( Target target, DescriptiveStatistics statistics[] ) {
//			time = target.getTime();
//			lat = target.getLatitude();
//			lon = target.getLongitude();
//			height = target.getHeight();
//			stat[azimuth][min] = statistics[azimuth].getMin();
//			stat[azimuth][mean] = statistics[azimuth].getMean();
//			stat[azimuth][mode] = statistics[azimuth].getPercentile(50.0);
//			stat[azimuth][max] = statistics[azimuth].getMax();
//			stat[azimuth][deviation] = statistics[azimuth].getStandardDeviation();
//			stat[elevation][min] = statistics[elevation].getMin();
//			stat[elevation][mean] = statistics[elevation].getMean();
//			stat[elevation][mode] = statistics[elevation].getPercentile(50.0);
//			stat[elevation][max] = statistics[elevation].getMax();
//			stat[elevation][deviation] = statistics[elevation].getStandardDeviation();
//			stat[range][min] = statistics[range].getMin();
//			stat[range][mean] = statistics[range].getMean();
//			stat[range][mode] = statistics[range].getPercentile(50.0);
//			stat[range][max] = statistics[range].getMax();
//			stat[range][deviation] = statistics[range].getStandardDeviation();
//		}
//	}
}

