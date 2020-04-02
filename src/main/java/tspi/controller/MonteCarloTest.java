package tspi.controller;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import tspi.model.*;

import java.io.File;
import java.io.PrintStream;
import java.util.List;
import java.util.Random;

/**
 * I'm getting confused on the architecture; I think Solution got conflated with
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
/**
 * Computes Monte Carlo statistics from solutions made from Gaussian perturbations of pedestals iterating a set of ideal plots...
 *  -- Origin is first line in pedestals file... whether is (or not) contributor of single-point solutions
 *  -- This module loads pedestals file supplied measurement model deviations to inject noise to each pedestal los
 * 	-- Single-point solutions for pedestals lay-down geometry.
 *
 * 	Usage:
 *  MonteCarloTest originIndex, trials, count, step, <input pedestal path> <output targets path>
 *
 */

	// note our random number generator always uses the same seed for
	// repeatability...
	private static Random Generator = new Random(1L);
	// TODO might want to use the commons distributions API instead...

	public static void main(String args[]) {
// Adjust these values; let me know if this is what you need.
//      originIndex = 0
//		double step = 0.01;
//		int count   = 20, 
//		trials      = 100;
//String input      = "/home/mike/photon/workspace/github/tracker/data/pedestalsTest100.csv"; 
//String output     = "/home/mike/photon/workspace/github/tracker/data/GridTest100.csv"; 

		try {
			if (args.length > 0) {
				int originIndex = Integer.parseInt(args[0]); // 0
				int trials = Integer.parseInt(args[1]); // 100
				int count = Integer.parseInt(args[2]); // 20
				double step = Double.parseDouble(args[3]); // 0.01d
				File input = new File(args[4]); // String input =
												// "/home/mike/photon/workspace/github/tracker/data/pedestalsTest100.csv";
				File output = new File(args[5]); // String output =
													// "/home/mike/photon/workspace/github/tracker/data/GridTest100.csv";

				PedestalModel model = new PedestalModel();
				model.load(input);

				// I need the origin as a pedestal, so I can point it and compute spherical
				// errors against Solutions.
				Pedestal origin = model.getPedestal(originIndex);

				long time = 0; // initialize time?
				double lat = origin.getLocationEllipsoid().getNorthLatitude().getDegrees(); // grid stream reference
				double lon = origin.getLocationEllipsoid().getEastLongitude().getDegrees(); // grid stream reference
				double height = origin.getLocationEllipsoid().getEllipsoidHeight() + 3000d; // grid stream reference

				Pedestal.setOrigin(origin.getLocation());

				for (Pedestal p : model) {
					p.setLocalOriginCoordinates();
				}
				PrintStream stream = new PrintStream(output);

				//This is the single-point statistical differences output for the generated grid points:
				testGrid(time, lat, lon, height, step, count, trials, origin, model.asList(), stream);

			} else {
				System.out.println(
						"Usage:\n MonteCarloTest: originIndex, trials, count, step, <input pedestal path> <output targets path> \n");
			}

		} catch (Exception exception) {
			exception.printStackTrace();
		}

	}

	public static void testGrid(long time, double lat, double lon, double height, double step, int count, int trials,
                                Pedestal origin, List<Pedestal> pedestals, PrintStream stream) {
		// print the header
		stream.append(
				"Lat, Lon, H, AZmin, AZmean, AZmode, AZmax, AZdev, ELmin, ELmean, ELmode, ELmax, ELdev, Rmin, Rmean, Rmode, Rmax, Rdev\n");
		stream.flush();

		// declare statistics for az, el, r in that order
		DescriptiveStatistics statistics[] = new DescriptiveStatistics[3];
		for (int n = 0; n < 3; n++)
			statistics[n] = new DescriptiveStatistics();
		// unfortunately there is no lib for multivariate descripive stats -i.e. stats
		// containing in memory percentile methods

		// generate a grid of targets
		TargetModel model = generateTargetGrid(time, lat, lon, height, count, step);

		// For every target on that grid
		for (Target target : model) {

			// initialize stats for new round of trials
			for (DescriptiveStatistics statistic : statistics)
				statistic.clear();

			// compute the correct, unperturbed pointing coordinates
			origin.pointToLocation(target.getGeocentricCoordinates());
			Polar truth = origin.getLocal();

			// for every trial
			for (int trial = 0; trial < trials; trial++) {

				// point the pedestals at the target with a random perturbation generated from
				// their error model
				pointPedestals(pedestals, target);

				// Compute a target solution, and point the origin at it
				Solution solution = new Solution(pedestals);
				origin.pointToLocation(solution._position_EFG);
				Polar measurement = origin.getLocal();
				// TODO make sure origin is right in this solution!!!
				// TODO Solution pretty much needs to start from scratch every time right? I
				// shouldn't be re-using something right?

				// Now compute error in spherical terms
				double error[] = { measurement.getUnsignedAzimuth().subtract(truth.getUnsignedAzimuth()).getDegrees(),
						measurement.getElevation().subtract(truth.getElevation()).getDegrees(),
						measurement.getRange() - truth.getRange() };

				// and update the descriptive statistics
				for (int n = 0; n < 3; n++)
					statistics[n].addValue(error[n]);
				// TODO SummaryStatistics has versions with contributing hierarchies and
				// synchronization;
				// if we want to try getting a multi-threading speed boost for larger tests...
				// hell, it might be feasible to progressively refine a plot as trials are run
				// if we make one of these per grid point...
			}

			// print out the min, mean, mode and max of errors in AER
			Character separator = ',';
			stream.append(Double.toString(target.getLatitude().getDegrees()));
			stream.append(separator);
			stream.append(Double.toString(target.getLongitude().getDegrees()));
			stream.append(separator);
			stream.append(Double.toString(target.getHeight()));
			for (int i = 0; i < 3; i++) {
				stream.append(separator);
				stream.append(Double.toString(statistics[i].getMin()));
				stream.append(separator);
				stream.append(Double.toString(statistics[i].getMean()));
				stream.append(separator);
				stream.append(Double.toString(statistics[i].getPercentile(50.0)));
				stream.append(separator);
				stream.append(Double.toString(statistics[i].getMax()));
				stream.append(separator);
				stream.append(Double.toString(statistics[i].getStandardDeviation()));
			}
			stream.append("\r\n"); // TODO should we extract more info from Solution too?
		}
	}

	public static TargetModel generateTargetGrid(long time, double lat, double lon, double height, int count,
                                                 double step) {
		TargetModel model = new TargetModel();
		for (int i = -count; i <= count; i++) {
			for (int j = -count; j <= count; j++) {
				Target target = new Target(time, lat + (i * step), lon + (j * step), height);
				model.add(model.getRowCount(), target);
			}
		}
		return model;
	}

	public static void pointPedestals(List<Pedestal> model, Target target) {
		for (Pedestal pedestal : model) {
			pedestal.pointToLocation(target.getGeocentricCoordinates());
			Polar pertubed = pedestal.getPerturbedLocal(Generator);
			pedestal.point(pertubed);
		}
	}

	// meh, we might eventually want to produce some model for these results when we
	// start passing them to a gui...
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
