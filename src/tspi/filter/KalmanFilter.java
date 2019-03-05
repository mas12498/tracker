package tspi.filter;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.apache.commons.math3.linear.SparseRealMatrix;

import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Vector3;
import tspi.util.TVector;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class KalmanFilter implements Filter {
	boolean _STEADY=false
			,_MANEUVER=false
			,_ACQUIRE=false
			,_INIT=true;
	
	RealVector _state;       //function of filter _x, dtNext
	RealMatrix _covariance;  //function of filter _P, dtNext
	
	//Filter Cycle Globals
	double _timePrev;	  // time of previous measurement frame
	long _msecAdv;		  // msec advance to next measurement frame
	double _processNoise; // @ 16 m/s/s	suggested
	final TVector _origin = new TVector(Pedestal.getOrigin()); //fusion system Geocentric coordinates {EFG}
	
	//measured (a_posteriori) filter state	place hold
	RealVector _x;
	RealMatrix _P;
	
	//predicted (a priori) filter state place hold
	RealVector _x_;
	RealMatrix _P_;
		
	//Kalman gain and working matrices...
	RealMatrix _S;  //transition state update:   [S: I+S == A]   
	RealMatrix _Q;  //process noise: 
	RealMatrix _H;  //observation H -- mapping from state-space x to measurement innovations of z
	RealMatrix _R;  //measurement noise: Modeled envelope of Gaussian measurement noise
	RealMatrix _Kt; //Kalman gain & blending.
	RealMatrix _D;  //Cov part-update matrix.
	RealMatrix _E;  //Cov part-update matrix.	

	//Measurement space structures
	RealVector _z;  //measurements: az and el projected differences from tracking Filter location in EFG
	RealVector _zm;  //measurements: az and el projected differences from tracking Filter location in EFG
	RealVector _w;  //innovations w  -- measurement residuals from a priori predictions 
	RealVector _e;  //residuals v  -- measurement residuals from a posteriori measurement updates
			
	/** 
	 * Construct Kalman Filter
	 * @param pedestals used in track fusion 
	 * @param initPosition used in track acquisition
	 * @param initVelocity used in track acquisition
	 * @param processNoise used in track steady state
	 * */
	public KalmanFilter( Pedestal pedestals[] , TVector initPosition, TVector initVelocity, double processNoise) {
					
		this._state = new ArrayRealVector(9);				//Kalman Filter exported state vector
		this._covariance = new Array2DRowRealMatrix(9, 9);  //Kalman Filter exported COV matrix
				
		this._x_ = new ArrayRealVector(9); 					//a priori filter state
		this._P_ = MatrixUtils.createRealMatrix(9,9);		//a priori filter COV		
		this._x = new ArrayRealVector(9);					//a posteriori filter state
		     _x.setSubVector(0, initPosition.realVector()); 
		     _x.setSubVector(3, initVelocity.realVector()); 	
		this._P = MatrixUtils.createRealMatrix(9,9);		//a posteriori filter COV
				
		this._S =  MatrixUtils.createRealMatrix(9,9);       	//transition update matrix S: Ax = x +Sx		
		this._Q = MatrixUtils.createRealMatrix(9,9); 			//process noise matrix Q: 	
		     _Q.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(_processNoise).getData(), 6, 6);		

		int nMeas = pedestals.length * 3; // potentially mapped rg, az and el pedestal sensors 	
		
		this._H = MatrixUtils.createRealMatrix(nMeas, 3);		//observation matrix H:
		this._R = MatrixUtils.createRealMatrix(nMeas, nMeas);	// measurement noise update R:		  
		this._D  = MatrixUtils.createRealMatrix(nMeas,9); 		//Work RHS.
		this._E  = MatrixUtils.createRealMatrix(nMeas,nMeas); 	//Work LHS.
		this._Kt = MatrixUtils.createRealMatrix(nMeas,9); 		//Kalman gain & blending matrix transposed.
		this._z = new ArrayRealVector(nMeas); 					//measurements: rg, az and el measurements	
		this._w = new ArrayRealVector(nMeas); 					//innovations w: a priori measurement differences
		this._e = new ArrayRealVector(nMeas); 					//residuals e: a posteriori measurement differences
		
		this._processNoise = processNoise;			
	}
	
	/** 
	 * Transition Update Matrix: S
	 *       S == (A-I)
	 * Generate sparse transition update matrix S for time of propagation between measurements:
	 * @param timePropagate   */
    public RealMatrix init_S(double timePropagate) {
    	RealMatrix subDiagonal = MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(timePropagate);   	   	
    	RealMatrix T = MatrixUtils.createRealMatrix(9,9);
    	//by cliques:
    	T.setSubMatrix(subDiagonal.getData(), 0, 3);
    	T.setSubMatrix(subDiagonal.getData(), 3, 6);
    	subDiagonal = subDiagonal.scalarMultiply(timePropagate/2);
    	T.setSubMatrix(subDiagonal.getData(), 0, 6);
    	return T; //transition matrix cast to sparse!
    }
    
    public RealMatrix init_P() {
    	double diag[] = {100000,100000,100000,1000,1000,1000,200,200,200};
    	return MatrixUtils.createRealDiagonalMatrix(diag);
    }
    
    public RealVector init_x() {
    	TVector position = new TVector(3135932.588, -5444754.209, 1103864.549);
    	TVector velocity = new TVector(0,0,0);
    	
    	double[] x = new double[9];
    	RealVector v = MatrixUtils.createRealVector(x);
    	v.setSubVector(0, position.realVector());
    	v.setSubVector(0, velocity.realVector());
    	return v;
    }
    
	
	/** 
	 * (D: The blas level 3 right hand side of K' computation)
	 * @return <b>D</b> = H P_
	 * @param H  Observation matrix
	 * @param P_ COV matrix (a priori)
	 * */
	public RealMatrix rightHandSideKt( RealMatrix H, RealMatrix P_ ) {
		return H.multiply(P_);                                                                             //level 3
	}
	   
	/** 
	 * (E: The blas level 3 left hand side of K' computation == H P_ H' + R)
	 * @return <b>E</b> = D H' + R 
	 * @param D Right Hand Side K'  
	 * @param H  Observation matrix
	 * @param R Measurement noise matrix
	 **/
	public RealMatrix leftHandSideKt( RealMatrix D, RealMatrix H, RealMatrix R ) { 
		return D.multiply(H.transpose()).add(R) ;	//level 3	
	}
		
	/** 
	 * (K': The QR solution of transpose of the Kalman gain and blending matrix)
	 * @return <b>K'</b> = E \ D
	 * @param E Left Hand Side K'
	 * @param D Right Hand Side of K' 
	 * */
	public RealMatrix solveTransposedKalmanGain( RealMatrix E, RealMatrix D ) {  
		return new QRDecomposition( E ).getSolver().solve(D);		//LA JAMA Style
	}
		
	/** 
	 * innovation w: The blas level 2 computation of a priori filter residual of measurement z
	 * @param H,z,x_   */
	public RealVector measurementResiduals(RealMatrix H, RealVector z, RealVector x_ ) {			
		return z.subtract( H.operate(x_));  //level 2
	}
	
	/** 
	 * state: The blas level 2 measurement update of x
	 * @param K_T,w,x_   */
	public RealVector updateState( RealMatrix K_T, RealVector w, RealVector x_ ) {
		return  x_.add( K_T.preMultiply(w) ) ;	//level 2		
	}
	
	/** 
	 * covariance: The blas level 3 measurement update of P
	 * @param Kt,G,Pt_  */
	public RealMatrix updateTransposedCovariance( RealMatrix Kt, RealMatrix D, RealMatrix P_ ) {
		return  P_.subtract(D.transpose().multiply(Kt));                //level 3			
	}
	
	/** 
	 * state: propagate apriori x_ = Ax
	 * @param S RealMatrix: S+I==A   
	 * @param x RealVector 
	 * @return  RealVector x_ = x+Sx (a priori) */
	public RealVector propagateState( RealMatrix S, RealVector x ) {
		return x.add(S.operate(x)); 		
	}
	
	/** 
	 * covariance: apriori propagate P_
	 * @param S,Q,P  */
	public RealMatrix propagateTransposedCovariance(RealMatrix S, RealMatrix q2, RealMatrix P) {
		RealMatrix P1 = P.add(P.multiply(S.transpose()));  // Sparse level 3
		return P1.add(S.multiply(P1)).add(q2);             // Sparse Level 3
	}
	
	/**
	 * This routine updates the filter state with new pedestal measurements ...
	 */
	@Override
	public RealVector filter( double time, Pedestal measurements[] ) {
		//Prediction update (a priori)		
		long msTimeAdvOld = _msecAdv;
		this._msecAdv = StrictMath.round((time - this._timePrev)*1000);
		if(_msecAdv < 10 ) _msecAdv = 20;
		if(_msecAdv!=msTimeAdvOld) {			
			_S = init_S(((double) _msecAdv)*0.001);		
			//this for filter re-Starts after a divergence, too.	
			_P = init_P();
			System.out.println("This is when initialized S:"+time);
		}
		TVector varI = new TVector(TVector.EMPTY);
		TVector varJ = new TVector(TVector.EMPTY);
		TVector varK = new TVector(TVector.EMPTY);
		//prediction update:
		_x_ = propagateState( _S, _x );
		_P_ = propagateTransposedCovariance( _S,  _Q, _P );
				
		//Compile measurement observation matrices: {(H|z),R}
		int m = 0;		
		for (int n = 0; n < measurements.length; n++) {
			// add rows to H, z vector, and diag(R) (projection ped sensor measurements)!!!!
			TVector pedLoc = new TVector(measurements[n].getLocalCoordinates()); // get this pedestal location
			if (measurements[n].getMapRG()) {
				varI.set(measurements[n].getAperture_i().unit());
				_H.setRow(m, varI.doubleArray());
				_z.setEntry(m, measurements[n].getLocal().getRange() + pedLoc.getInnerProduct(varI)); // meters track																						// error
				_R.setEntry(m, m, measurements[n].getDeviationRG());
				m += 1;
			}
			if ((measurements[n].getMapAZ()) || (measurements[n].getMapEL())) {
				double priorPedRG = new TVector(_x_.getSubVector(0, 3)).subtract(pedLoc).getAbs();
				System.out.println(measurements[n].getSystemId() + " predicted range to target: " + priorPedRG + "\n");
				if (measurements[n].getMapAZ()) {
					varJ.set(measurements[n].getAperture_j().unit().divide(priorPedRG));
					_H.setRow(m, varJ.doubleArray());
					_z.setEntry(m, pedLoc.getInnerProduct(varJ)); // @radians tracke error AZ
					_R.setEntry(m, m, measurements[n].getDeviationAZ().getRadians());
					m += 1;
				}
				if (measurements[n].getMapEL()) {
					varK.set(measurements[n].getAperture_k().unit().divide(priorPedRG));
					_H.setRow(m, varK.doubleArray());
					_z.setEntry(m, pedLoc.getInnerProduct(varK)); // @radians track error EL
					_R.setEntry(m, m, measurements[n].getDeviationEL().getRadians());
					m += 1;
				}
			}
		}
		this._timePrev = time;
		
//		Edit innovations based upon mode limits???
		_w = measurementResiduals( // (m) - (m,3)(3) == (m)
				_H.getSubMatrix(0, m - 1, 0, 2)
				, _z.getSubVector(0, m)
				, _x_.getSubVector(0, 3)
				);

		RealMatrix singlePoint = new Array2DRowRealMatrix(_H.getData());  //REFERENCE	ideal pedestal Solution:
		
//		Single Point Soln? f(H,z): Least Squares versus error in variables solution...
		SingularValueDecomposition svd = new SingularValueDecomposition(singlePoint.getSubMatrix(0,m-1,0, singlePoint.getColumnDimension()-1));
		RealVector b = new ArrayRealVector(_z);
		RealVector y = svd.getSolver().solve(b.getSubVector(0, m));				
		
		if (m > 0) { //Have new measurements to process an update:
			
			_D = rightHandSideKt( // (mx3)(3x9) == (mx9)
					_H.getSubMatrix(0, m - 1, 0, 2)
					,_P_.getSubMatrix(0,2,0,8)
					);
			_E = leftHandSideKt( // (mx3)(3xm)+(mxm) == (mxm)
					_D.getSubMatrix(0, m - 1, 0,2)
					,_H.getSubMatrix(0, m - 1, 0, 2)
					,_R.getSubMatrix(0, m - 1, 0, m - 1)
					);
			_Kt = solveTransposedKalmanGain( // (mxm) \ (mxp) == (mxp)
					_E.getSubMatrix(0, m - 1, 0, m - 1)
					, _D.getSubMatrix(0, m - 1, 0, 8)
					);
			_x = updateState( // (m) + (pxm)(m) == (m)
					_Kt
					, _w
					, _x_
					);
			_P = updateTransposedCovariance( // (9x9) - (9xm)(mx9) = (9x9)
					_Kt.getSubMatrix( 0, m - 1,0, 8) 
					,_D.getSubMatrix(0, m - 1, 0, 8)
					,_P_);
			_e = measurementResiduals( // (m) - (m,3)(3) == (m)
					//OR: -->   _e = _w + _x - _x_
					_H.getSubMatrix(0, m - 1, 0, 2)
					, _z.getSubVector(0, m)
					, _x.getSubVector(0, 3)
					);
			
			System.out.println("\n ***Innovations Norm = "+_w.getNorm()*StrictMath.sqrt(m-3));
			System.out.println("\n ***Innovations = {  ");
			for (int h=0;h<m; h++) {
				System.out.print(_w.getEntry(h)+"\n  ");
			}
			System.out.println("} \n\n");
			System.out.println("\n ***Residuals Norm = "+_e.getNorm()*StrictMath.sqrt(m-3));
			System.out.println("\n ***Residuals = {  ");
			for (int h=0;h<m; h++) {
				System.out.print(_e.getEntry(h)+"\n  ");
			}
			System.out.println("} \n");
			
		}
		
		//Export geocentric coordinate state of filter... could also be a forecast for a look-ahead...
        _state=_x.copy();
		_state.setSubVector(0,(_origin.realVector().add(_x.getSubVector(0, 3))));
		
		_covariance = _P;
		return _state;
		
	}
//	@Override
//	public RealVector filter( double time, Pedestal pedestals[] ) {
//		//TODO
//		return null;
//	}
	
	/**
	 * Return differences between filter a posteriori and rejected measurements
	 */
	public Polar[] getEditsPrediction( double dt ) {
		
		//TODO: give array of residuals of measurements from prediction a priori... 
		return null;
	}
	
	/**
	 * Return differences between filter a posteriori and rejected measurements
	 */
	public Polar[] getEditsUpdate( double dt ) {
		
		//TODO: give array of residuals of measurements from prediction a priori... 
		return null;
	}
	
	/**
	 * Return differences between filter a priori and measurements
	 */
	public Polar[] getResidualsPrediction( double dt ) {
		
		//TODO: give residuals of measurements from prediction a priori... 
		return null;
	}
	
	/**
	 * Return differences between filter a posteriori and measurements
	 */
	public Polar[] getResidualsUpdate( double dt ) {
		//TODO: give residuals of measurements from prediction a posteriori
		return null;
	}
	
	
	public RealVector getState() {
		return _state;
	}
	
	public RealMatrix getCovariance() {
		return _covariance;
	}
	
	public RealVector getResiduals(){
		return _e;
	}
	
	public RealVector getInnovations(){
		return _w;
	}

//	@Override
//	public RealVector prediction( double time ) {
//		//TODO
//		return null;
//	}
}