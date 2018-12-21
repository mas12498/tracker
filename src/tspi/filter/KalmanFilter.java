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
import tspi.util.TVector;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class KalmanFilter implements Filter {
	final TVector origin = new TVector(Pedestal.getOrigin());
	RealVector fuseOrigin =  origin.realVector();
	double filterRange;
	double time;
	double timeLagged;
	double timeElapsed;
	long msTimeAdv;
	
	RealMatrix covariance;
	RealVector state;
	double[] stateVector;
	
	//measured filter state	place hold
	RealVector x = new ArrayRealVector(9);
	RealMatrix Pt = MatrixUtils.createRealMatrix(9,9);
	
	//a priori filter state place hold
	RealVector x_ = new ArrayRealVector(9);
	RealMatrix Pt_ = MatrixUtils.createRealMatrix(9,9);
	
	//transition state update matrix:   (9,9)
	//SparseRealMatrix S; 
	double[][] S_doub = new double[9][9]; 
	RealMatrix S =  MatrixUtils.createRealMatrix(S_doub);
	//process noise update matrix:      (9,9)
	double[][] Q_doub = new double[9][9]; 
	RealMatrix Q = MatrixUtils.createRealMatrix(Q_doub);
	
	//Kalman gain and working matrices...
	RealMatrix Kt = MatrixUtils.createRealMatrix(6,9); //Kalman gain and blending matrix.
	RealMatrix F  = MatrixUtils.createRealMatrix(9,9); //Optimal observability.
	RealMatrix G  = MatrixUtils.createRealMatrix(9,9); //Cov measurement update matrix.

	//measurements: az and el differences from tracking Filter in EFG
	RealVector z = new ArrayRealVector(6); 
	
	//observation matrix H -- map from state-space x to measurement innovations of z
	RealMatrix H = MatrixUtils.createRealMatrix(6,9); 
		
	//innovations w vector -- residuals for updating apriori filter
	RealVector w = new ArrayRealVector(6); 
	
	//measurement noise update:  (6,6)
	//SparseRealMatrix R; 
	double[][] R_doub = new double[6][6]; 
	RealMatrix R =  MatrixUtils.createRealMatrix(R_doub);
	
	//list of pedestals and positionings
	double[] measurements = new double[6];
	
	double dt=0.020;
	double wn = 30; //  m/s/s
	double sa = StrictMath.toRadians(.005);
	double se = StrictMath.toRadians(.005);
	
    public void initSparseQ(SparseRealMatrix Q, double wn) {
    	double[] qDiagnonal = {0,0,0,0,0,0,wn,wn,wn}; //Process errors possible in kinematic acceleration
    	Q = (SparseRealMatrix) (MatrixUtils.createRealDiagonalMatrix(qDiagnonal)); //.createRealIdentityMatrix(9);  //process noise matrix: stochastic noise introduced in transition
    }
    
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
    	T.setSubMatrix(subDiagonal.getData(), 6, 3);
    	subDiagonal.scalarMultiply(timePropagate/2);
    	T.setSubMatrix(subDiagonal.getData(), 0, 6);
    	return T; //transition matrix cast to sparse!
    }
	
	
	/** 
	 * Kt: update the transpose of the Kalman gain and blending matrix K
	 * @param H,R,P_   */
	public RealMatrix updateTransposedKalmanGain( RealMatrix H, RealMatrix R, RealMatrix Pt_, RealMatrix F, RealMatrix G ) {
		F = H.multiply(Pt_);              //level 3
		G = Pt_.multiply(H.transpose());  //level 3
		
		return new QRDecomposition(H.multiply(G).add(R)).getSolver().solve(F);		//LA JAMA Style
	}
	
	
	/** 
	 * state: measurement innovation w: filter residual of measurement z
	 * @param H,z,x_   */
	public RealVector measurementInnovation(RealMatrix H, RealVector z, RealVector x_ ) {		
		return z.subtract(H.operate(x_));  //level 2
	}
	
	/** 
	 * state: measurement update x
	 * @param Kt,w,x_   */
	public RealVector updateState( RealMatrix Kt, RealVector w, RealVector x_ ) {
		return  x_.add( Kt.operate(w) ) ;	//level 2
	}
	
	/** 
	 * covariance: measurement update Pt
	 * @param Kt,G,Pt_  */
	public RealMatrix updateTransposedCovariance( RealMatrix Kt, RealMatrix G, RealMatrix Pt_ ) {
		
		return  Pt_.subtract(G.multiply(Kt)); //level 3
	
	}
	
	
	/** 
	 * state: propagate apriori x_
	 * @param S,Q,x_   */
	public RealVector propagateState( RealMatrix s2, RealVector x ) {
		x =  x.copy().add(s2.operate(x));                //Sparse level 2
		
		//monitor Filter track range
		TVector position = new TVector(x,0);
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
		return Pt1.add((s2).multiply(Pt1)).add(q2); // Sparse Level 3
	}
	
	Pedestal pedestals[];
	
	public KalmanFilter( Pedestal pedestals[] ) {
			
		this.time = 0.0;
		double timeLagged = -dt;
		double timeElapsed = 0.0;
		double timeAdv = dt;
		
		
		this.pedestals = pedestals;
		this.covariance = new Array2DRowRealMatrix(9, 9); //P matrix
		
		//this.state = new ArrayRealVector(9); //x vector
		
		//?instance in stream?	
		
	}
	
	/**
	 * This routine updates the Kalman filter state with new pedestal measurements at time ...
	 */
	@Override
	public RealVector filter( double time, Pedestal measurements[] ) {
		
		//Prediction update (a priori)		
		long msTimeAdvOld = msTimeAdv;
		this.msTimeAdv = StrictMath.round((this.time - this.timeLagged)*1000);
		filterRange = StrictMath.hypot(StrictMath.hypot(x.getEntry(1), x.getEntry(2)),x.getEntry(3));
		if(filterRange < 100) filterRange = 1000;
		if(msTimeAdv < 10 ) msTimeAdv = 20;
		if(msTimeAdv!=msTimeAdvOld) {
			S = initSparseUpdateS(((double) msTimeAdv)*0.001);
		
			//this for filter re-Starts after a divergence, too.	
			Pt.setEntry(0, 0, 100000); 
			Pt.setEntry(1, 1, 100000); 
			Pt.setEntry(2, 2, 100000); 
			Pt.setEntry(3, 3, 1000); 
			Pt.setEntry(4, 4, 1000); 
			Pt.setEntry(5, 5, 1000); 
			Pt.setEntry(6, 6, 200); 
			Pt.setEntry(7, 7, 200); 
			Pt.setEntry(8, 8, 200); 
			filterRange = 20000;
		}
		
		x_ = propagateState( S, x );
		Pt_ = propagateTransposedCovariance( S,  Q, Pt );
		
		//Compile measurement: {(H|z),R}
        
		int mp = 0;		
		for (int n=0; n<measurements.length; n++) {
			System.out.println(measurements[n].getSystemId()+" "+n);
//			transportReliable = bernouliDeviate(); // simulate leaky measurement transport...
//			if (pedestals[n].getMapAZ() && pedestals[n].getMapEL() && transportReliable) {
			if (measurements[n].getMapAZ() && measurements[n].getMapEL()) {
 
				// add rows to H, z vector, and diag(R)  (pairs for this case)!!!!
				TVector pedLoc = new TVector(measurements[n].getLocalCoordinates()); // location from fusion origin
				TVector seJ = new TVector( measurements[n].getAperture_j().divide(filterRange)); 
				TVector seK = new TVector( measurements[n].getAperture_k().divide(filterRange));				
				double hM[][] = { {seJ.getX(),seJ.getY(),seJ.getZ()} 
						         ,{seJ.getX(),seJ.getY(),seJ.getZ()} };
				
				H.setSubMatrix(hM,mp,0 );
				z.setEntry(mp,  seJ.getInnerProduct(pedLoc)); //radians measurement error AZ
				z.setEntry(mp+1, seK.getInnerProduct(pedLoc)); //radians measurement error EL
				R.setEntry(mp, mp, measurements[n].getDeviationAZ().getRadians());
				R.setEntry(mp+1, mp+1, measurements[n].getDeviationEL().getRadians());
				mp+=2;

			}
		}
		
		//RealVector rD = MatrixUtils.createRealVector(rDiagnonal).getSubVector(0, mp-1);
		//RealMatrix R = MatrixUtils.createRealDiagonalMatrix(rD.toArray());	
		if (mp > 0) {
			Kt = updateTransposedKalmanGain( //Kt = Kt.getSubMatrix( 0, 8,0, mp - 1)
					H.getSubMatrix(0, mp - 1, 0, 8)
					,R.getSubMatrix(0, mp - 1, 0, mp - 1)
					,Pt_
					, F.getSubMatrix( 0, 8,0, mp - 1)
					, G.getSubMatrix( 0, 8,0, mp - 1)
			);
			w = measurementInnovation(H, z, x_);
			x = updateState(Kt.getSubMatrix( 0, 8,0, mp - 1)
					, w, x_);
			Pt = updateTransposedCovariance(
					Kt.getSubMatrix( 0, 8,0, mp - 1) 
					,H.getSubMatrix(0, mp - 1, 0, 8)
					,Pt_);
//		r_OT = LocalCoordinates(T)
//		Vector3 eI = measurements[n].getAperture_i();
//		double zI = eI.getInnerProduct(pedLoc) + eI.getInnerProduct(r_OT);
		}
		state.setSubVector(0,state.getSubVector(0, 3).add(fuseOrigin));
		covariance = Pt;
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