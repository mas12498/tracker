package tspi.filter;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.*;
import tspi.rotation.Vector3;
import tspi.simulator.*;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;

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
			double time = observations.getTime();

			// get the true nav plot
			Vector3 t = observations.getTruth();
			Ellipsoid llh = new Ellipsoid();
			llh.setGeocentric(t);

			// tabulate the current true target position
			navs.append(Double.toString(observations.getTime()));
			navs.append(", ").append(String.valueOf(llh.getNorthLatitude()))
					.append(", ").append(String.valueOf(llh.getEastLongitude()))
					.append(", ").append(String.valueOf(llh.getEllipsoidHeight()));
			navs.println();

			// update the filter with the noisy measurements
			RealVector state = filter.filter(time, ensemble);

			// tabulate the filter results into CSV
			stream.append(Double.toString(time)); // time
			for (double d : state.toArray())
				stream.append(", ").append(String.valueOf(d));// state delta

			// Do we want to include residuals?
//			RealVector residuals = filter.getResiduals();
//			RealVector innovations = filter.getInnovations();

			stream.println();
		}
	}
	/* TODO Do we want to adapt this into a test? How can we do that?
	 *  - make sure tracker state is within some epsilon of the truth? */

	/** loads a constellation of pedestals and their associated observations as they track a target.
	 * Their measurements are combined in a Kalman filter, whose state is written out to a file as observations are added.
	 * <pre> Tracker <pedestal input> <observations input> <state output> <truth output> </pre> */
	public static void main(String[] args) {

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
			// this example uses a observations file but you could use a simulator instead

			Pedestal pedestal = reader.getEnsemble().getOrigin();
			Ellipsoid origin = pedestal.getLocationEllipsoid();
			System.out.println("ORIGIN:" + pedestal.getLocation().toString(3));

			TestFilter test = new TestFilter(reader);
			test.track(stream, navs);

			navs.close();
			stream.close();

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
