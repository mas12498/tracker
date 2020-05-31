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

}
