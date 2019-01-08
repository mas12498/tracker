package tspi.filter;

import java.util.ArrayList;

import tspi.model.Pedestal;

/** This example is for applying a Filter against the pre-recorded measurements of an ensemble.
 * It also provides the ability to simulate some data using a {@link tspi.filter.Trajectory Trajectory}
 * and a Pedestal ensemble. */
public class TestMeasurements {

	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

	public static ArrayList<double[]> createMeasurements(
			Trajectory trajectory,
			Pedestal[] pedestals,
			double t0, double dt, int n
	) {
		ArrayList<double[]> measurements = new ArrayList<double[]>();
		
		
		
		return measurements;
	} // TODO this is very similar to Ensemble test.generate
}
