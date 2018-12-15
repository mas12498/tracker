package tspi.filter;

import java.util.Random;

import tspi.model.Pedestal;
import tspi.model.Polar;

/** Exercise the filter */
class TestFilter {
	public static void main(String args[]) {
		Random random = new Random(1);
		
		// TODO initialize the pedestal array
		Pedestal pedestals[] = {};
		
		// TODO create the filter
		Filter filter = new KalmanFilter( pedestals );
		
		// TODO create the Trajectory
		TrackSimulator.Trajectory model = null;
		TrackSimulator simulator = new TrackSimulator( model, pedestals, random ); 
		Polar measurements[] = {};
		
		// generate measurements over time 
		double start=0, end=10, dt=.020;
		for (double t=start; t<end; t+=dt) {
			measurements = simulator.generate(t, measurements);
			
			// update the filter
			filter.measurement(t+dt, measurements);
			
			//TODO compare prediction with next time step?
			// compare filter state with parametric model.
			// use descriptive statistics
			// tabulate into CSV output?
		}
	}
}