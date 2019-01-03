package tspi.filter;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import rotation.Vector3;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.util.TVector;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class KalmanFilter implements Filter {
	
	
//	//Filter Globals
	double _timePrev;
	long _msecAdv;
	TVector _position;
	double filterRange;
	
	//Simple Filter process noise model for acceleration error: ~3g
	double _processNoise; // = 30;
	
	Pedestal pedestals[]; //list of pedestals 
	
	
	final TVector _origin = new TVector(Pedestal.getOrigin());
	
	//
	//Kalman Filter structures:
	//
	
	//for export
	RealMatrix covariance;
	RealVector state;
	
	//measured filter state	place hold
	RealVector _x;
	RealMatrix _P;
	
	//a priori filter state place hold (before measurements applied)
	RealVector _x_;
	RealMatrix _P_;
	
	
	//transition state update matrix:   [S: I+S == A]
	RealMatrix _S;
	
	//process noise update matrix:      
	RealMatrix _Q;
	
	//Kalman gain and working matrices...
	RealMatrix _K_T; //Kalman gain & blending matrix.
	RealMatrix _F;   //Cov part-update matrix.

	//measurements: az and el projected differences from tracking Filter location in EFG
	RealVector _z;
	
	//observation matrix H -- mapping from state-space x to measurement innovations of z
	RealMatrix _H;
		
	//innovations w  -- vector residual used for updating apriori filter
	RealVector _w;
	
	//measurement noise: Modeled envelope of Gaussian measurement noise
	RealMatrix _R;
		
	//constructor from pedestals list 
	public KalmanFilter( Pedestal pedestals[] , Kinematic cue, double processNoise) {
		this._processNoise = processNoise;	
		//potential measurement pedestals listed (including origin and non fusion sensors)
		this.pedestals = pedestals; //reference to external array list...
		int nMeas = pedestals.length*2; //potentially both az and el sensors used for selected pedestals
		
		//Kalman Filter exported outputs:
		this.state = new ArrayRealVector(9);		
		this.covariance = new Array2DRowRealMatrix(9, 9); //P matrix
		
		//internal filter state	and COV place hold
		this._x = new ArrayRealVector(9);	
		//trial initiation of cued position and velocity... 
		  _x.setSubVector(0, cue.getPosition(0));
		  _x.setSubVector(0, cue.getVelocity(0));
		  
		this._position	= 	new TVector(Vector3.EMPTY);
		  
		this._P = MatrixUtils.createRealMatrix(9,9);
		
		//a priori internal filter state and COV place hold
		this._x_ = new ArrayRealVector(9);
		this._P_ = MatrixUtils.createRealMatrix(9,9);
		
		//dense transition state update matrix S:  
		this._S =  MatrixUtils.createRealMatrix(9,9);
		
		//dense process noise update matrix Q:
		this._Q = MatrixUtils.createRealMatrix(9,9);
		_Q.setEntry(6, 6, _processNoise);
		_Q.setEntry(7, 7, _processNoise);
		_Q.setEntry(8, 8, _processNoise);
		
		
		//transposed Kalman gain and partial working matrices...envelopes
		this._K_T = MatrixUtils.createRealMatrix(nMeas,9); //Kalman gain and blending matrix.
		this._F  = MatrixUtils.createRealMatrix(9,9); //Optimal observability.

		//measurements: az and el differences from tracking Filter in EFG
		this._z = new ArrayRealVector(nMeas);
		
		//observation matrix H: 
		this._H = MatrixUtils.createRealMatrix( nMeas, 9);
			
		//innovations w:
		this._w = new ArrayRealVector(nMeas);
		
		//measurement noise update R:  
		this._R =  MatrixUtils.createRealMatrix(nMeas,nMeas);


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
    public RealMatrix initSparseUpdateS(double timePropagate) {
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
	 * first part of similarity transform...HP of HPH'
	 * Return F:= H P_
	 * @param H,P_   
	 * */
	public RealMatrix productHP( RealMatrix H, RealMatrix P_ ) {
		return H.multiply(P_);                                                                             //level 3
	}
	   
	/** 
	 * Kt: update the transpose of the Kalman gain and blending matrix K
	 * @param H,R,P_ : (F)  
	 * */
	public RealMatrix updateTransposedKalmanGain( RealMatrix H, RealMatrix R, RealMatrix P_, RealMatrix F ) {  
		//level 3
		return new QRDecomposition( F.copy().multiply(H.transpose()).add(R) ).getSolver().solve(F);		//LA JAMA Style
	}
		
	/** 
	 * state: measurement innovation w: filter residual of measurement z
	 * @param H,z,x_   */
	public RealVector measurementInnovation(RealMatrix H, RealVector z, RealVector x_ ) {		
		return z.subtract( H.operate(x_));  //level 2
	}
	
	/** 
	 * state: measurement update x
	 * @param Kt,w,x_   */
	public RealVector updateState( RealMatrix K_T, RealVector w, RealVector x_ ) {
		return  x_.add( K_T.preMultiply(w) ) ;	//level 2		
	}
	
	/** 
	 * covariance: measurement update Pt
	 * @param Kt,G,Pt_  */
	public RealMatrix updateTransposedCovariance( RealMatrix Kt, RealMatrix F, RealMatrix P_ ) {
		return  P_.subtract(F.transpose().multiply(Kt));                //level 3			
	}
	
	
	/** 
	 * state: propagate apriori x_
	 * @param s2==S, x==x_   */
	public RealVector propagateState( RealMatrix s2, RealVector x ) {
		RealVector y = x.copy();
		x =  y.add(s2.operate(x));                //Sparse level 2
		
		//monitor Filter track range
		TVector position = new TVector(x);
		filterRange = position.getAbs(); //StrictMath.hypot(x.getEntry, y)
		if (filterRange < 500) {
			filterRange = 500;
		}
		
		return x;
	}
	
	/** 
	 * covariance: apriori propagate Pt_
	 * @param S,Q,Pt  */
	public RealMatrix propagateTransposedCovariance(RealMatrix s2, RealMatrix q2, RealMatrix Pt) {
		RealMatrix Pt1 = Pt.add(Pt.multiply(s2.transpose())); // Sparse level 3
		return Pt1.add(s2.multiply(Pt1)).add(q2);             // Sparse Level 3
	}
	
	/**
	 * This routine updates the Kalman filter state with new pedestal measurements at time ...
	 */
	@Override
	public RealVector filter( double time, Pedestal measurements[] ) {
		//Prediction update (a priori)		
		long msTimeAdvOld = _msecAdv;
		this._msecAdv = StrictMath.round((time - this._timePrev)*1000);
		if(_msecAdv < 10 ) _msecAdv = 20;
		if(_msecAdv!=msTimeAdvOld) {
			
			_S = initSparseUpdateS(((double) _msecAdv)*0.001);
		
			//this for filter re-Starts after a divergence, too.	
			_P = init_P();
			
			//filterRange = 20000;
			System.out.println("This is when initialized S:"+time);
		}

		filterRange = StrictMath.hypot(StrictMath.hypot(_x.getEntry(0), _x.getEntry(1)),_x.getEntry(2));
		if(filterRange < 100) filterRange = 1000;
		
		_x_ = propagateState( _S, _x );
		_P_ = propagateTransposedCovariance( _S,  _Q, _P );
		
		
		//Compile measurement: {(H|z),R}
		int mp = 0;		
		_position = new TVector(_x_.getSubVector(0, 3)); //get filter state position (local coordinates)
		for (int n=0; n<measurements.length; n++) {			
//			transportReliable = bernouliDeviate(); // simulate leaky measurement transport...
//			if (pedestals[n].getMapAZ() && pedestals[n].getMapEL() && transportReliable) {
			if (measurements[n].getMapAZ() && measurements[n].getMapEL()) {
				
				TVector pedLoc = new TVector(measurements[n].getLocalCoordinates()); //get pedestal location from fusion origin				
				//double pedestalRange = measurements[n].getAperture_i().unit().getInnerProduct(new TVector(pedLoc).subtract(_position));
				double pedestalRange = (new TVector(_position).subtract(pedLoc)).getInnerProduct(measurements[n].getAperture_i().unit());
				
				////double pedestalRange = 1;
				System.out.println(measurements[n].getSystemId()+" predicted range to target: "+ pedestalRange +"\n");

				
				
				// add rows to H, z vector, and diag(R)  (projection of pairs for this case)!!!!
				TVector seJ = new TVector( measurements[n].getAperture_j().unit().divide(pedestalRange)); 
				TVector seK = new TVector( measurements[n].getAperture_k().unit().divide(pedestalRange));				
				double hM[][] = { {seJ.getX(),seJ.getY(),seJ.getZ()} 
						         ,{seK.getX(),seK.getY(),seK.getZ()} };
				
				_H.setSubMatrix(hM,mp,0 );
				
				_z.setEntry(mp,  new TVector(pedLoc).getInnerProduct(seJ)); //@radians measurement error AZ
				_z.setEntry(mp+1, new TVector(pedLoc).getInnerProduct(seK)); //@radians measurement error EL
				_R.setEntry(mp, mp, measurements[n].getDeviationAZ().getRadians());
				_R.setEntry(mp+1, mp+1, measurements[n].getDeviationEL().getRadians());
				mp+=2;
			}			
		}
		this._timePrev = time;
		
		//RealVector rD = MatrixUtils.createRealVector(rDiagnonal).getSubVector(0, mp-1);
		//RealMatrix R = MatrixUtils.createRealDiagonalMatrix(rD.toArray());	
		if (mp > 0) {
			_F = productHP( //F := F.getSubMatrix(0, mp - 1, 0, 8)
					_H.getSubMatrix(0, mp - 1, 0, 8)
					,_P_
					);

			_K_T = updateTransposedKalmanGain( //Kt = Kt.getSubMatrix( 0, 8,0, mp - 1)
					_H.getSubMatrix(0, mp - 1, 0, 8)
					,_R.getSubMatrix(0, mp - 1, 0, mp - 1)
					,_P_
					, _F.getSubMatrix(0, mp - 1, 0, 8)
			);
			//H.getSubMatrix(0, mp - 1, 0, 8)
			//z.getSubVector(0, mp -1);
			_w = measurementInnovation(
					_H.getSubMatrix(0, mp - 1, 0, 8)
					, _z.getSubVector(0, mp)
					, _x_
					);
			//System.out.println("And here we have a conformal matrix issue!");
			_x = updateState(
					_K_T
					, _w
					, _x_
					);
			
			
			
			
			
			_P = updateTransposedCovariance(
					_K_T.getSubMatrix( 0, mp - 1,0, 8) 
					,_F.getSubMatrix(0, mp - 1, 0, 8)
					,_P_);
//		r_OT = LocalCoordinates(T)
//		Vector3 eI = measurements[n].getAperture_i();
//		double zI = eI.getInnerProduct(pedLoc) + eI.getInnerProduct(r_OT);
		}
		
//		System.out.println("And here we have a conformal matrix issue!");
        state=_x.copy();
	    //.setSubVector(0,(state.getSubVector(0, 3).add(_origin.realVector())));
		state.setSubVector(0,(_origin.realVector().copy().add(_x.getSubVector(0, 3))));
		
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
	public Polar[] getResidualsPrediction( double dt ) {
		return null;
	}
	public Polar[] getResidualsUpdate( double dt ) {
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