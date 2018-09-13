package tspi.controller;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

import rotation.Vector3;
import tspi.model.Pedestal;
import tspi.model.PedestalModel;
import tspi.model.Polar;
import tspi.model.Solution;
import tspi.model.Target;
import tspi.model.TargetModel;

/** So I'm not to happy with the architecture, and it's probably my fault;
 * Mainly because Solution got conflated with Target to make some view tasks convenient.
 * Since all these iterations are little one-off demonstrators, I wasn't too worried about it- however;
 * We should really take careful stock of our architecture before starting a large NRT application.
 * 
 * To avoid kludging to meet another demonstrator or deep conflicts with the master branch,
 * I just wrote stuff rather procedurally here to avoid messing with the model objects right now in interest of getting the data out...
 */
public class MonteCarloTest {
	
	private static Random Generator;

	public static void main(String args[]) {
		long seed = 1;
		long time = 0;
		double lat = 0.0, lon = 0.0, height=0.0;
		double step = 0.05;
		int count = 24, trials = 1000;
		
		Generator = new Random( seed );
		// TODO probably want to use the commons distributions API instead...
		
		ArrayList<Pedestal> array;		
		PedestalModel model = new PedestalModel();
		File input = new File("");
		
		try {
			model.load(input);
			array = model.asList();
			
			Pedestal origin = model.getPedestal(0); // or something...
			
//			double grid[][][][] = 
			testGrid(
					time, lat, lon, height, step,
					origin, model.asList(),
					count, trials);
			
		} catch(Exception e) {
			e.printStackTrace();
		}

		
	}
	
	public static void testGrid(
			long time, double lat, double lon, double height, double step,
			Pedestal origin, List<Pedestal> pedestals,
			int count, int trials ) {

		// declare statistics for az, el, r in that order
		DescriptiveStatistics statistics[] = new DescriptiveStatistics[3];
		for (DescriptiveStatistics statistic : statistics)
			statistic = new DescriptiveStatistics(); // unfortunately there is no lib for multivariate descripive stats -i.e. stats containing in memory percentile methods
		
		// generate a grid of targets
		TargetModel model = generateTargetGrid( time, lat, lon, height, count, step );
		
		// For every target on that grid
		for (Target target: model) {
			
			// initialize stats for new round of trials
			for (DescriptiveStatistics statistic:statistics)
				statistic.clear();
			
			// compute the correct, unperturbed location
			origin.pointToLocation( target.getGeocentricCoordinates() );
			Polar truth = origin.getLocal();

			// for every trial
			for( int trial=0; trial < trials; trial++ ) {
				
				// point the pedestals at the target with a random perturbation generated from their error model
				pointPedestals(pedestals, target);
				
				// Compute a target solution, and point the origin at it 
				Solution solution = new Solution( pedestals ); // TODO make sure origin is right in this solution!!!
				origin.pointToLocation( solution.position_EFG );
				Polar measurement = origin.getLocal();
				
				// Now compute error in spherical terms
				double error[] = {
						measurement.getAzimuth().subtract( truth.getAzimuth() ).getDegrees(),
						measurement.getElevation().subtract( truth.getElevation() ).getDegrees(),
						measurement.getRange() - truth.getRange()
				};
				
				// and update the descriptive statistics
				for (int n=0; n<3; n++)
					statistics[n].addValue(error[n]);
			} // TODO SummaryStatistics has versions with contributing hierarchies and synchronization if we want to try getting a multithreading speed boost for larger tests!
			
			// compute min, mean, mode and max of errors in AER
			statistics[0].getMean();
			// TODO store the error in the target grid? attached to some internal solution?
			// or in a 2d array?

		}
			
		
	}
	
	public static TargetModel generateTargetGrid(
			long time, double lat, double lon, double height,
			int count, double step
	) {
		TargetModel model = new TargetModel();
		for (int n=-count; n<count; n++) { 
			Target target = new Target(time, lat+(count*step), lon+(count*step), height);
			model.add(model.getRowCount(), target);
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
	
	class GridAccuracy {
		final int azimuth=0, elevation=1, range=2;
		final int min=0, mean=1, mode=2, max=3, deviation=4;
		double stat[][] = new double [3][5];
		long time;
		double lat, lon, height;
	}
	
	public static double ftime( Date time ) {
		Calendar c = Calendar.getInstance();
		c.setTime( time );
		double millis = 0.0;
		millis += c.get(Calendar.HOUR_OF_DAY)*3600000;
		millis += c.get(Calendar.MINUTE)*60000;
		millis += c.get(Calendar.SECOND)*1000;
		millis += c.get(Calendar.MILLISECOND);
		return millis/1000.0;
	}
}

