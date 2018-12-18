package tspi.filter;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import tspi.model.Pedestal;
import tspi.model.PedestalModel;
import tspi.model.Polar;

/** Exercise the filter */
class TestFilter {
	public static void main(String args[]) {
		
		// constant seed right now for reproducibility
		Random random = new Random(1);
		
		// initialize the pedestal array
		File file = new File("");
		PedestalModel model = new PedestalModel();
		try { model.load(file); }
		catch (Exception e) {
			e.printStackTrace();
			return;
		}
		
		// convert pedestal model a primitive array
		ArrayList<Pedestal> list = model.asList();
		Pedestal pedestals[] = new Pedestal[list.size()];
		list.toArray(pedestals);
		
		// create the Trajectory model
		double a[] = {0.0, 0.0, 0.0};
		double v[] = {0.0, 0.0, 0.0};
		double p[] = {0.0, 0.0, 0.0};
		TrackSimulator.Trajectory trajectory = new Kinematic(
				new ArrayRealVector(a),
				new ArrayRealVector(v),
				new ArrayRealVector(p) );
		
		// create the track simulator from the trajectory model
		TrackSimulator simulator = new TrackSimulator( trajectory, pedestals, random ); 
		Polar measurements[] = {};
		
		// TODO create the filter
		Filter filter = new KalmanFilter( pedestals );
		
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

/** You ain't cheating you ain't trying */
class CheatFilter implements Filter {
	TrackSimulator.Trajectory cheat;
	
	public CheatFilter(TrackSimulator.Trajectory hint) {
		this.cheat = hint;
	}
	
	@Override
	public void measurement(double time, Polar[] measurements) {}
	
	@Override
	public RealVector prediction(double time) {
		return cheat.getState(time);
	}
	
}

