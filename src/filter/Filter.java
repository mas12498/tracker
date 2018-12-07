package filter;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import tspi.model.Pedestal;
import tspi.model.Polar;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class Filter {

	double time;
	RealMatrix covariance;
	RealVector state;
		
	RealVector x;
	RealMatrix P;
	
	RealMatrix x_;
	RealMatrix P_;
	
	
	double dt;
	RealVector A; //transition matrix: apriori predict from time k-1 to time at k
	RealMatrix Q; //process noise matrix: stochastic noise introduced in transition

	RealMatrix H; //observation matrix: map from state-space x to measurement space of z
	RealMatrix z; //measurement vector for updating apriori filter
	RealMatrix R; //measurement noise matrix: stochastic noise introduced in measurement
	
	RealMatrix K; //Kalman gain and blending matrix.

	/** 
	 * K: measurement update Kalman gain and blending matrix
	 * @param H,R,P_   */
	public RealMatrix updateKalmanGain( RealMatrix H, RealMatrix R, RealMatrix P_ ) {
		//RealMatrix numerator = P_.multiply(H.transpose());                        //matrix RHS
		//RealMatrix denominator =( H.multiply(P_).multiply(H.transpose())).add(R); //matrix LHS				
		//QRDecomposition factored = new QRDecomposition((H.multiply(P_).multiply(H.transpose())).add(R));		
		//DecompositionSolver K = factored.getSolver();                             //solve K
		//return 	factored.getSolver().solve(P_.multiply(H.transpose()));		
		return new QRDecomposition((H.multiply(P_).multiply(H.transpose())).add(R)).getSolver()
				.solve(P_.multiply(H.transpose()));
	}
	
	/** 
	 * state: measurement update x
	 * @param K,H,z,x_   */
	public RealMatrix updateState( RealMatrix K, RealMatrix H, RealMatrix z, RealMatrix x_ ) {
		
		return  x_.add(K.multiply( z.subtract(H.multiply(x_)))) ;
	
	}
	
	/** 
	 * covariance: measurement update P
	 * @param K,H,P_  */
	public RealMatrix updateCovariance( RealMatrix K, RealMatrix H, RealMatrix P_ ) {
		
		return  (MatrixUtils.createRealIdentityMatrix(9).subtract(K.multiply( H))).multiply(P_) ;
	
	}
	
	
	/** 
	 * state: propagate apriori x_
	 * @param A,Q,x_   */
	public RealMatrix propagateState( RealMatrix A, RealMatrix x ) {
		
		return  A.multiply(x);
	
	}
	
	/** 
	 * covariance: apriori propagate P_
	 * @param A,Q,P  */
	public RealMatrix propagateCovariance( RealMatrix A, RealMatrix Q, RealMatrix P ) {
		
		return  (A.multiply(P).multiply(A.transpose())).add(Q); 
	
	}
	
	
	Pedestal pedestals[];
	
	public Filter( Pedestal pedestals[] ) {
		this.time = 0.0;
		this.pedestals = pedestals;
		this.covariance = new Array2DRowRealMatrix(9, 9); //P matrix
		this.state = new ArrayRealVector(9); //x vector
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