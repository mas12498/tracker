package tspi.filter;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SparseRealMatrix;

import tspi.model.Pedestal;
import tspi.model.Polar;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class KalmanFilter implements Filter {

	double time;
	RealMatrix covariance;
	RealVector state;
		
	RealMatrix x = MatrixUtils.createRealMatrix(9,1);
	RealMatrix P = MatrixUtils.createRealMatrix(9,9);
	
	RealMatrix x_ = MatrixUtils.createRealMatrix(9,1);
	RealMatrix P_ = MatrixUtils.createRealMatrix(9,9);
	
	
	double dt=0.020;
	
	SparseRealMatrix A; //transition matrix: apriori predict from time k-1 to time at k
	
	double wn = 10;
	double[] qDiagnonal = {0,0,0,0,0,0,wn,wn,wn};
	SparseRealMatrix Q = (SparseRealMatrix) MatrixUtils.createRealDiagonalMatrix(qDiagnonal); //.createRealIdentityMatrix(9);  //process noise matrix: stochastic noise introduced in transition

	RealMatrix H = MatrixUtils.createRealMatrix(6,9); //observation matrix: map from state-space x to measurement space of z
	RealMatrix z = MatrixUtils.createRealMatrix(6,1); //measurement vector for updating apriori filter
	double sa = StrictMath.toRadians(.005);
	double se = StrictMath.toRadians(.005);
	double[] rDiagnonal = {sa,se,sa,se,sa,se};
	SparseRealMatrix R = (SparseRealMatrix) MatrixUtils.createRealDiagonalMatrix(rDiagnonal); //measurement noise matrix: stochastic noise introduced in measurement
	
	RealMatrix K; //Kalman gain and blending matrix.

	/** 
	 * Generate one of these for each likely propagation interval and on fly for weird ones.
	 * 
	 * A: generate sparse transition matrix from time of propagation
	 * @param sec_propagate   */
    public SparseRealMatrix createTransitionMatrix(double timePropagate) {
    	RealMatrix subIdentity = MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(timePropagate);
    	RealMatrix T = MatrixUtils.createRealIdentityMatrix(9);
    	T.setSubMatrix(subIdentity.getData(), 0, 3);
    	T.setSubMatrix(subIdentity.getData(), 6, 3);
    	subIdentity.scalarMultiply(timePropagate/2);
    	T.setSubMatrix(subIdentity.getData(), 0, 6);
    	return ( SparseRealMatrix) T; //cast to sparse!
    }
	
	
	/** 
	 * K: measurement update Kalman gain and blending matrix
	 * @param H,R,P_   */
	public RealMatrix updateKalmanGain( SparseRealMatrix H, SparseRealMatrix R, RealMatrix P_ ) {
		RealMatrix work = H.multiply(P_.transpose());
		return (new QRDecomposition((work.multiply(H.transpose())).add(R)).getSolver().solve(work)).transpose();
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
	public RealMatrix propagateState( SparseRealMatrix A, RealMatrix x ) {
		
		return  A.multiply(x);
	
	}
	
	/** 
	 * covariance: apriori propagate P_
	 * @param A,Q,P  */
	public RealMatrix propagateCovariance( SparseRealMatrix A, SparseRealMatrix Q, RealMatrix P ) {
		
		return  (A.multiply(P).multiply(A.transpose())).add(Q); 
	
	}
	
	
	Pedestal pedestals[];
	
	public KalmanFilter( Pedestal pedestals[] ) {
		this.time = 0.0;
		this.pedestals = pedestals;
		this.covariance = new Array2DRowRealMatrix(9, 9); //P matrix
		this.state = new ArrayRealVector(9); //x vector
	}
	
	
	
	
	/* (non-Javadoc)
	 * @see tspi.filter.Filter#measurement(double, tspi.model.Polar[])
	 */
	@Override
	public void measurement( double time, Polar measurements[] ) {
		
	}
	// digression; we might want something more explicit and dynamic;
//	public void measurement( long time, Map<Pedestal, Polar> measurements ) { }
	
	
	/* (non-Javadoc)
	 * @see tspi.filter.Filter#prediction(double)
	 */
	@Override
	public RealVector prediction( double time ) {
		//TODO
		return null;
	}
	//TODO this has to eventually be done in a thread safe way...
	// and we probably want something less contentious that a simple mutex on the state object...
}