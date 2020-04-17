package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Vector3;
import tspi.util.TVector;

import java.util.Random;

/** A parametric model of the simulated trajectory. */
public interface Trajectory {
	
	RealVector getPosition(double time);

	RealVector getVelocity(double time);

	RealVector getAcceleration(double time);

	RealVector getState(double time);

	/** Obtain a ideal point from the trajectory model then generate a measurement vector
	 * by pointing each pedestal at the ideal point then perturbing it by the
	 * pedestals' error model.
	 * @param time over segment...
	 * @param pedestals the sensors which will take measurements of the moving object, including error
	 * @return an array containing all the pedestals' measurements of the moving object. It's
	 * indices correspond to the indices of the pedestal array. */
	default Polar[] track(double time, Pedestal[] pedestals, Random random) {
		
		// find the value of the parametric model at the given time
		RealVector p = this.getPosition(time);
		Vector3 efg = new TVector( p );
		
		// have each pedestal take a measurement, including error
		Polar[] measurements = new Polar[pedestals.length];
		for (int n=0; n<pedestals.length; n++) {
			pedestals[n].pointToLocation( efg );
			measurements[n] = pedestals[n].getPerturbedLocal(random);
		}
		
		return measurements;
	} // TODO make sensors intermittently drop measurements

	// This version actually leaves the pedestal pointed at the perturbed location?
	default void simulateTrack(double time, Pedestal[] pedestals, Random random) {
		// find the value of the parametric model at the given time
		RealVector p = this.getPosition(time);
		Vector3 efg = new TVector( p );
		Polar[] measurements = new Polar[pedestals.length];

		for (int n=0; n<pedestals.length; n++) {
			if (pedestals[n].getMapAZ() && pedestals[n].getMapEL()) { // CDS: why is this case here again? it will break all simulation of partial sensors...
				pedestals[n].pointToLocation(efg);
				measurements[n] = pedestals[n].getPerturbedLocal(random);
				pedestals[n].pointDirection(measurements[n].getUnsignedAzimuth(), measurements[n].getElevation());
			} else {
				pedestals[n].clearPedestalVector();
			}
		}
	}
}
