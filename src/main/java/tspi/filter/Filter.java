package tspi.filter;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;

// TODO will there be multiple implementations for the sake of comparison?

/**  */
public interface Filter {

	/** Update the filter's state and covariance using a vector of pedestal measurements.
	 * @param ensemble
	 * @return position, velocity and acceleration at the given time. */
	RealVector filter(double time, Ensemble ensemble);

	// TODO is this still needed?
	/* using the current model, predict the future location of the target using the
	 * current state.
	 * @return The state of the filter */
//	RealVector prediction( double time );

	RealVector getState();

	RealMatrix getCovariance();

	RealVector getResiduals();

	RealVector getInnovations();

}
