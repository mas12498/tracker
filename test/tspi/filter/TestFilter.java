package tspi.filter;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
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
		
		PedestalModel model;
		try {
			// initialize output
			File out = null;
			PrintStream stream = System.out;
			if (out!=null)
				stream = new PrintStream( new FileOutputStream(out) );
			
			// initialize the pedestal array
			File in = new File("");
			model = new PedestalModel();
			model.load(in);
		}
		catch (Exception e) {
			e.printStackTrace();
			return;
		}
		
		// convert pedestal model a primitive array
		ArrayList<Pedestal> list = model.asList();
		Pedestal pedestals[] = new Pedestal[list.size()];
		list.toArray(pedestals);
		
		// create the Trajectory model
		// moving west over intersection of meridian and equator at about 700m altitude?
		// we're in meters right? TODO I should just use the EFG classes to construct this correctly...
		double a[] = {-10.0, 0.0, 0.0}; // mpss
		double v[] = {0.0, 300.0, 0.0}; // mps
		double p[] = {4500000.0, 0.0, 0.0}; // meter
		TrackSimulator.Trajectory trajectory = new Kinematic(
				new ArrayRealVector(a),
				new ArrayRealVector(v),
				new ArrayRealVector(p) );
		
		// create the track simulator from the trajectory model
		TrackSimulator simulator = new TrackSimulator( trajectory, pedestals, random ); 
		Polar measurements[] = {};
		
		// TODO create the filter
//		Filter filter = new KalmanFilter( pedestals );
		Filter filter = new CheatFilter( trajectory ); 
		
		// generate measurements over time 
		double start=0, end=100, dt=.02;
		for (double t=start; t<end; t+=dt) {
			measurements = simulator.generate(t, measurements);
			
			// update the filter
			filter.measurement(t, measurements);
			
			// TODO compare filter state with parametric model.
			filter.prediction(t); // prediction is kind of a misnomer when dt is 0...
			
			//TODO compare prediction with next time step too?
			filter.prediction(t+dt);
			
			// TODO use descriptive statistics
			
			// TODO tabulate into CSV output?
			
		}
		// TODO make sensors intermittently drop measurements
		
		//TODO extract test which compares given filter to cheating filter... 
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

