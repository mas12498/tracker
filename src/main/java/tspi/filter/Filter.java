package tspi.filter;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import tspi.model.Pedestal;
import tspi.model.Polar;

public interface Filter {

	/** Update the filter's state and covariance using a vector of pedestal measurements.
	* @param measurements an array of measurements in the same index order as the corresponding pedestal array
	* @return position, velocity and acceleration at the given time. */
	RealVector filter(double time, Pedestal measurements[]);
	// digression; we might want something more explicit and dynamic;
	//	public void measurement( long time, Map<Pedestal, Polar> measurements ) { }

//	/** using the current model, predict the future location of the target using the
//	 * current state.
//	 * @return The state of the filter */
//	RealVector prediction( double time );
//	//TODO this has to eventually be done in a thread safe way...
//	// and we probably want something less contentious that a simple mutex on the state object...

	Polar[] getResidualsPrediction(double dt);
	Polar[] getResidualsUpdate(double dt);
	RealVector getState();
	RealMatrix getCovariance();
	RealVector getResiduals();
	RealVector getInnovations();
	
	
}