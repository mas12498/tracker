package filter;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import tspi.model.Pedestal;
import tspi.model.Polar;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class Filter {

	double time;
	RealMatrix covariance;
	RealVector state;
	Pedestal pedestals[];
	
	public Filter( Pedestal pedestals[] ) {
		this.time = 0.0;
		this.pedestals = pedestals;
		this.covariance = new Array2DRowRealMatrix(9, 9);
		this.state = new ArrayRealVector(pedestals.length*2); //3 with range?
	}
	
	/** Update the filter's state and covariance using a vector of pedestal measurements.
	 * @param measurements an array of measurements in the same index order as the corresponding pedestal array*/
	public void measurement( double time, Polar measurements[] ) {
		
	}
	// digression; we might want something more explicit and dynamic;
//	public void measurement( long time, Map<Pedestal, Polar> measurements ) { }
	
	
	/** using the current model, predict the future location of the target using the 
	 * current state. */
	public RealVector prediction( double time ) {
		//TODO
		return null;
	}
	//TODO this has to eventually be done in a thread safe way...
	// and we probably want something less contentious that a simple mutex on the state object...
}

/** Used to generate a series of test measurements of an idealized motion. */
class Trajectory {
	
	Model model;
	Pedestal pedestals[];
	
	public Trajectory( Model model, Pedestal pedestals[] ) {
		this.model = model;
		this.pedestals = pedestals;
	}
	
	interface Model { 
		public RealVector getState( double time );
	}
	// TODO implement a linear model, then a second order kinematic model
	// then possibly some circle with a constant acceleration... 
	
	/** Obtain a ideal point from the model then generate a measurement vector 
	 * by pointing each pedestal at the ideal point then perturbing it by the
	 * pedestals' error model. */
	Polar[] generate( double time, Polar measurments[] ) {
		return null;
	}
}

/** Exercise the filter */
class TestFilter {
	public static void main(String args[]) {
		
		// TODO initialize the pedestal array
		Pedestal pedestals[] = {};
		
		// TODO create the filter
		Filter filter = new Filter( pedestals );
		
		// TODO create the Trajectory
		Trajectory.Model model = null;
		Trajectory trajectory = new Trajectory( model, pedestals ); 
		Polar measurements[] = {};
		
		// generate measurements over time 
		double start=0, end=10, dt=.020;
		for (double t=start; t<end; t+=dt) {
			measurements = trajectory.generate(t, measurements);
			
			// update the filter
			filter.measurement(t+dt, measurements);
			
			//TODO compare prediction with next time step?
			// Calculate delta between state and kinematic terms?
			// tabulate into CSV output?
		}
	}
}