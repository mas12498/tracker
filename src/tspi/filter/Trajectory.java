package tspi.filter;

import java.util.Random;

import org.apache.commons.math3.linear.RealVector;

import rotation.Vector3;
import tspi.model.Pedestal;
import tspi.model.Polar;

/** A parametric model of the simulated trajectory. */
interface Trajectory {
	
	public RealVector getState( double time );
	
	/** Obtain a ideal point from the trajectory model then generate a measurement vector 
	 * by pointing each pedestal at the ideal point then perturbing it by the
	 * pedestals' error model. */
	default Polar[] generate( double time, Pedestal pedestals[], Random random ) {
		
		// find the value of the parametric model at the given time
		RealVector p = getState(time);
		Vector3 efg = new Vector3( p.getEntry(0), p.getEntry(1), p.getEntry(2) );
		
		// have each pedestal take a measurement, including error
		Polar measurements[] = new Polar[pedestals.length];
		for (int n=0; n<pedestals.length; n++) {
			Pedestal pedestal = pedestals[n];
			pedestal.pointToLocation( efg );
			measurements[n] = pedestal.getPerturbedLocal(random);
		}
		
		return measurements;
	}
}

// TODO implement a linear model, then a second order kinematic model
// then possibly some circle with a constant acceleration...
// maybe a piecewise spline so we can test jerk and jounce...