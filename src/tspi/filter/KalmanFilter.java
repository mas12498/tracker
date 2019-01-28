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
import tspi.rotation.Vector3;
import tspi.util.TVector;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class KalmanFilter implements Filter {
		
	RealVector state;       //function of filter _x, dtNext
	RealMatrix covariance;  //function of filter _P, dtNext
	
	//Filter Cycle Globals
	double _timePrev;	  // time of previous measurement frame
	long _msecAdv;		  // msec advance to next measurement frame
	TVector _position;	  // Cartesian local pedestal position from fusion system origin
	double filterRange;   //range to track filter point computed from fusion system origin
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
		
		this._processNoise = processNoise;	
		
		//Kalman Filter exported outputs:
		this.state = new ArrayRealVector(9);				//
		this.covariance = new Array2DRowRealMatrix(9, 9);   //P matrix
		
		//internal filter state	and COV place hold
		this._x = new ArrayRealVector(9);	
		  _x.setSubVector(0, initPosition.realVector());
		  _x.setSubVector(3, initVelocity.realVector());
		this._position	= new TVector(Vector3.EMPTY);
			  		  
		this._P = MatrixUtils.createRealMatrix(9,9);
		
		//a priori internal filter state and COV place hold
		this._x_ = new ArrayRealVector(9);
		this._P_ = MatrixUtils.createRealMatrix(9,9);
		
		//dense transition state update matrix S:  
		this._S =  MatrixUtils.createRealMatrix(9,9);
		
		//process noise matrix Q:
		this._Q = MatrixUtils.createRealMatrix(9,9);  	   	
		  _Q.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(_processNoise).getData(), 6, 6);		

		// potential measurement pedestals listed (including origin and non fusion sensors)
		int nMeas = pedestals.length * 2; // potentially both az and el sensors used for selected pedestals
			
		// observation matrix H:
		this._H = MatrixUtils.createRealMatrix(nMeas, 3);
		
		// measurement noise update R:
		this._R = MatrixUtils.createRealMatrix(nMeas, nMeas);		  
		  
		//transposed Kalman gain and partial working matrices...envelopes
		this._D  = MatrixUtils.createRealMatrix(nMeas,9); //Optimal observability.
		this._E  = MatrixUtils.createRealMatrix(nMeas,nMeas); //Optimal observability.
		this._Kt = MatrixUtils.createRealMatrix(nMeas,9); //Kalman gain and blending matrix.

		//measurements: az and el differences from tracking Filter in EFG
		this._z = new ArrayRealVector(nMeas);		
		this._w = new ArrayRealVector(nMeas); //innovations w
		this._e = new ArrayRealVector(nMeas); //residuals e
		
	}
	
	
//    public void initSparseQ(SparseRealMatrix Q, double wn) {
//    	double[] qDiagnonal = {0,0,0,0,0,0,wn,wn,wn}; //Process errors possible in kinematic acceleration
//    	Q = (SparseRealMatrix) (MatrixUtils.createRealDiagonalMatrix(qDiagnonal)); //.createRealIdentityMatrix(9);  //process noise matrix: stochastic noise introduced in transition
//    }   
//	public void initSparseR(SparseRealMatrix R, double sa, double se) {
//		double[] rDiagnonal = { sa, se, sa, se, sa, se }; // R[6,6]
//		R = (SparseRealMatrix) MatrixUtils.createRealDiagonalMatrix(rDiagnonal); // measurement noise matrix: stochastic
//																					// noise introduced in measurement
//	}

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
		return D.multiply(H.transpose()).add(R) ;		
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
	 * state: propagate apriori x_
	 * @param S+I==A, x==x_   */
	public RealVector propagateState( RealMatrix S, RealVector x ) {
		return x.add(S.operate(x)); 		
	}
	
	/** 
	 * covariance: apriori propagate P_
	 * @param S,Q,P  */
	public RealMatrix propagateTransposedCovariance(RealMatrix S, RealMatrix q2, RealMatrix P) {
		RealMatrix P1 = P.add(P.multiply(S.transpose())); // Sparse level 3
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
			
			//filterRange = 20000;
			System.out.println("This is when initialized S:"+time);
		}

		filterRange = StrictMath.hypot(StrictMath.hypot(_x.getEntry(0), _x.getEntry(1)),_x.getEntry(2));
		if(filterRange < 100) filterRange = 1000;
		
		//prediction update:
		_x_ = propagateState( _S, _x );
		_P_ = propagateTransposedCovariance( _S,  _Q, _P );
		
		
		//Compile measurement: {(H|z),R}
		int m = 0;		
		_position = new TVector(_x_.getSubVector(0, 3)); //get filter state position (local coordinates)
		for (int n=0; n<measurements.length; n++) {
			
//			transportReliable = bernouliDeviate(); // simulate leaky measurement transport...
//			if (pedestals[n].getMapAZ() && pedestals[n].getMapEL() && transportReliable) {
			//CHECK for EDITS here!!!
			if (measurements[n].getMapAZ() && measurements[n].getMapEL()) {
				
				
				//predict pedestal 'n' measurement range:
				TVector pedLoc = new TVector(measurements[n].getLocalCoordinates()); //get pedestal location from fusion origin	
				//compute range from pedestal to a priori target:
				double pedestalRange = new TVector(_position).subtract(pedLoc).getAbs();
				
				System.out.println(measurements[n].getSystemId()+" predicted range to target: "+ pedestalRange +"\n");
				
				// add rows to H, z vector, and diag(R)  (projection of pairs for this case)!!!!
				
				//Ped 'n' observations matrix
				TVector varJ = new TVector( measurements[n].getAperture_j().unit().divide(pedestalRange)); 
				TVector varK = new TVector( measurements[n].getAperture_k().unit().divide(pedestalRange)); 			

				//Do validation here...
				
				double hM[][] = { {varJ.getX(),varJ.getY(),varJ.getZ()} 
						         ,{varK.getX(),varK.getY(),varK.getZ()} };
				_H.setSubMatrix(hM,m,0 );
				
				//Ped 'n' measurements
				_z.setEntry(m,   pedLoc.getInnerProduct(varJ)); //@radians tracke error AZ
				_z.setEntry(m+1, pedLoc.getInnerProduct(varK)); //@radians track error EL
				
				//Ped 'n' measurement noise model
				////double sqrt2 = StrictMath.sqrt(2d);
				_R.setEntry(m, m, measurements[n].getDeviationAZ().getRadians() );////*sqrt2);
				_R.setEntry(m+1, m+1, measurements[n].getDeviationEL().getRadians() );////*sqrt2);
				m+=2;
				
			}			
		}
		this._timePrev = time;
		
		
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
			
			_w = measurementResiduals( // (m) - (m,3)(3) == (m)
					_H.getSubMatrix(0, m - 1, 0, 2)
					, _z.getSubVector(0, m)
					, _x_.getSubVector(0, 3)
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
			
		}
		
		//Export geocentric coordinate state of filter... could also be a forecast for a look-ahead...
        state=_x.copy();
		state.setSubVector(0,(_origin.realVector().add(_x.getSubVector(0, 3))));
		
		covariance = _P;
		return state;
		
	}
//	@Override
//	public RealVector filter( double time, Pedestal pedestals[] ) {
//		//TODO
//		return null;
//	}
	// NOTE We might want something more explicit and dynamic;
	//	public void measurement( long time, Map<Pedestal, Polar> measurements ) { }
	
	//TODO implement accessors for the state you want displayed.

	
	//Need to re-think this context...
	
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
		return state;
	}
	public RealMatrix getCovariance() {
		return covariance;
	}
	
//	@Override
//	public RealVector prediction( double time ) {
//		//TODO
//		return null;
//	}
}