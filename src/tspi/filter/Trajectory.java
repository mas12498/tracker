package tspi.filter;

import java.util.Random;

import org.apache.commons.math3.linear.RealVector;

import rotation.Vector3;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.util.TVector;

/** A parametric model of the simulated trajectory. */
interface Trajectory {
	
	public RealVector getPosition( double time );
	
	public RealVector getVelocity( double time );
	
	public RealVector getAcceleration( double time );
	
	public RealVector getState( double time );
		
	/** Obtain a ideal point from the trajectory model then generate a measurement vector 
	 * by pointing each pedestal at the ideal point then perturbing it by the
	 * pedestals' error model.
	 * @param time over segment...
	 * @param trajectory the truth source for the tracked object's movement
	 * @param pedestals the sensors which will take measurements of the moving object, including error
	 * @return an array containing all the pedestals' measurements of the moving object. It's 
	 * indices correspond to the indices of the pedestal array. */
	default Polar[] track( double time, Pedestal pedestals[], Random random ) {
		
		// find the value of the parametric model at the given time
		RealVector p = this.getPosition(time);
		Vector3 efg = new TVector( p );
		
		// have each pedestal take a measurement, including error
		Polar measurements[] = new Polar[pedestals.length];
		for (int n=0; n<pedestals.length; n++) {
			pedestals[n].pointToLocation( efg );
			measurements[n] = pedestals[n].getPerturbedLocal(random);
		}
		
		return measurements;
	} // TODO make sensors intermittently drop measurements
	
	default void simulateTrack(double time, Pedestal pedestals[], Random random) {
		// find the value of the parametric model at the given time
		RealVector p = this.getPosition(time);
		Vector3 efg = new TVector( p );
		Polar measurements[] = new Polar[pedestals.length];
		// clear H matrix AND z vector here!!!!
		for (int n=0; n<pedestals.length; n++) {
//			transportReliable = bernouliDeviate(); // simulate leaky measurement transport...
//			if (pedestals[n].getMapAZ() && pedestals[n].getMapEL() && transportReliable) {
			if (pedestals[n].getMapAZ() && pedestals[n].getMapEL()) {
				pedestals[n].pointToLocation(efg);
				measurements[n] = pedestals[n].getPerturbedLocal(random);
				pedestals[n].pointDirection(measurements[n].getUnsignedAzimuth(), measurements[n].getElevation());
				// add rows to H matrix and z vector HERE (pairs for this case)!!!!
			} else {
				pedestals[n].clearPedestalVector();
			}
		}

	}
	
	
}

// TODO implement a linear model, then a second order kinematic model
// then possibly some circle with a constant acceleration...
// maybe a piecewise spline so we can test jerk and jounce...