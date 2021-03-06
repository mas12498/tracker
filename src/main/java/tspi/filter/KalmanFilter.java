package tspi.filter;

import org.apache.commons.math3.linear.*;
import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Rotator;
import tspi.rotation.Vector3;
import tspi.util.TVector;

import java.util.ArrayList;

/** A Kalman filter for synthesizing a series of measurements into a trajectory model in near-real-time. */
public class KalmanFilter {
	
	int[] mapPed;
	int[] mapSensor;
	int _numMeas;
	
	//Track Mode filter control booleans
	boolean _STEADY     = false
			,_MANEUVER  = false
			,_ACQUIRE   = false
			,_ASSOCIATE = true;

	//Latest Z statistic formed from normalized residuals
	double _Z_NormalizedResidual;

	//Filter mode determination by critical deviations of normalized residuals
	double _Z_STEADY      = 3;
	double _Z_MANEUVER    = 6;
	double _Z_ACQUISITION = 12;
	double _Z_ASSOCIATE   = 24;

	//Filter auto-tune -- Process noise Q by filter mode selection
	double _STEADY_Q 		= 1;
	double _MANEUVER_Q		= 10;
	double _ACQUISITION_Q	= 100;
	//Process noise initialization
	double _autoTune_Q = _ACQUISITION_Q;

	//Auto-tune aided by min input solution limits...
	int _CRIT_NUMBER_CONVERGE   = 4;
	int _CRIT_NUMBER_TIGHTEN	= 4;
	int _CRIT_NUMBER_LOOSEN		= 3;
	int _ENSEMBLES_CONVERGENCE	= 30;
	int _ENSEMBLES_DIVERGENCE	= 30;

    //innovations edit thresholds for filter inputs

	double _Z_EDIT_STEADY      = 3; //NOTE: _editMeasurement read from measurement model overrides.
	double _Z_EDIT_MANEUVER    = 6;
	double _Z_EDIT_ACQUISITION = 9;
	double _Z_EDIT_ASSOCIATION = 24;


	//Measurement noise R by Cartesian measurement uncertainty (sigma^2):
	// --Sensor measurement deviations projected at filter track point!
	//   --Angle (radians) raw deviations scaled by track point range [to meters]
	//   --Range raw deviations by instrument are [already] in meters
	RealVector _sigmaMeasurement;
	// --Edits measurement innovations by instrument from _STEADY Filter stream
	RealVector _editThreshold; //Overrides _Z_EDIT_STEADY.

	ArrayList<Polar> _predictedVideoResiduals;

	//mode performance statistics support
	int cntSteady = 0;
	int cntInit = 0;
	int cntDivergence = 0;
	int cntAcquisition = 0;
	int cntManeuver = 0;
	int cntCoast = 0;
	int plotCount = 0;


	//Filter Cycle Globals
	double 	_timePrev;	  // time of previous measurement frame
	long 	_msecAdv;	  // msec advance to next measurement frame
	double filterRange;
		
	final 	TVector _origin = new TVector(Pedestal.getOrigin()); //fusion system Geocentric coordinates {EFG}
	
	TVector _position;
	
	RealVector v_point = new ArrayRealVector(3);
	RealVector p_point = new ArrayRealVector(3);
	RealVector v_point_prior = new ArrayRealVector(3);
	RealVector p_point_prior = new ArrayRealVector(3);
			
	RealMatrix _covariance;  //function of filter _P, dtNext
	RealVector _state;       //function of filter _x, dtNext
		
	//measured (a_posteriori) filter state	place hold
	RealVector _x;
	RealMatrix _P;
	
	//predicted (a priori) filter state place hold
	RealVector _x_;
	RealMatrix _P_;
		
	//Kalman gain and working matrices...
	RealMatrix _S;  //transition state update:   [S: I+S == A]
	RealMatrix _Q;  //process noise:
	RealMatrix _Kt; //Kalman gain & blending.
	RealMatrix _D;  //Cov part-update matrix.
	RealMatrix _E;  //Cov part-update matrix.

	//Measurement space matrices...
	RealVector _z;  //measurements: az and el projected differences from tracking Filter location in EFG
	//RealVector _zm;  //measurements: az and el projected differences from tracking Filter location in EFG
	RealMatrix _H;  //observation H -- mapping from state-space x to measurement innovations of z

	RealVector _w;  //innovations w  -- measurement residuals from a priori predictions
	RealVector _e;  //residuals v  -- measurement residuals from a posteriori measurement updates
	RealVector _eN;  //residuals v  -- measurement residuals from a posteriori measurement updates

	RealMatrix _R;  //measurement noise: Modeled envelope of Gaussian measurement noise
//	RealDiagonalMatrix _Rdiag;

	/** 
	 * Construct Kalman Filter
	 * @param ensemble A collection of Pedestals whose measurements are used by the KalmanFilter
	 * */
	public KalmanFilter( Ensemble ensemble ) {
		
		this._numMeas = ensemble.size() * 3; // potentially mapped rg, az and el pedestal sensors of group
		
		this._state = new ArrayRealVector(9);				//Kalman Filter exported state vector
		
		this._x = new ArrayRealVector(9);					//a posteriori filter state

		this._covariance = new Array2DRowRealMatrix(9, 9);  //Kalman Filter exported COV matrix
		this._position = new TVector(Vector3.EMPTY);
		
		this._P = MatrixUtils.createRealMatrix(9,9);		//a posteriori filter COV

		this._x_ = new ArrayRealVector(9); 					//a priori filter state
		this._P_ = MatrixUtils.createRealMatrix(9,9);		//a priori filter COV
				
		this._S =  MatrixUtils.createRealMatrix(9,9);       	//transition update matrix S: Ax = x +Sx
		
		this._Q = MatrixUtils.createRealMatrix(9,9); 			//process noise matrix Q:
		     _Q.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(_autoTune_Q).getData(), 6, 6);

		this._sigmaMeasurement = new ArrayRealVector(_numMeas);
		this._editThreshold = new ArrayRealVector(_numMeas);

		this._predictedVideoResiduals = new ArrayList<Polar>(_numMeas);

		this._Kt = MatrixUtils.createRealMatrix(_numMeas,9); 		//Kalman gain & blending matrix transposed.
		this._D  = MatrixUtils.createRealMatrix(_numMeas,9); 		//Work RHS.
		this._E  = MatrixUtils.createRealMatrix(_numMeas,_numMeas); 	//Work LHS.
	
		this._z = new ArrayRealVector(_numMeas); 					//measurements: rg, az and el measurements
		this.mapPed = new int[_numMeas];
		this.mapSensor = new int[_numMeas];
		
		this._H = MatrixUtils.createRealMatrix(_numMeas, 3);		//observation matrix H:

		this._w = new ArrayRealVector(_numMeas); 					//innovations w: a priori measurement differences
		this._e = new ArrayRealVector(_numMeas); 					//residuals e: a posteriori measurement differences
		this._eN = new ArrayRealVector(_numMeas); 					//residuals e: a posteriori measurement differences

		this._R = MatrixUtils.createRealMatrix(_numMeas, _numMeas);	// measurement noise update R:
			
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
            	
	/** 
	 * (D: The blas level 3 right hand side of K' computation)
	 * @return <b>D</b> = H P_
	 * @param H  Observation matrix
	 * @param P COV matrix (a priori)
	 * */
	public RealMatrix rightHandSideKt(RealMatrix H, RealMatrix P ) {
		return H.multiply(P);    //level 2                                                                         //level 3
	}
	   
	/** 
	 * (E: The blas level 3 left hand side of K' computation == H P_ H' + R)
	 * @return <b>E</b> = D H' + R 
	 * @param D Right Hand Side K'  
	 * @param H  Observation matrix
	 * @param R Measurement noise matrix
	 **/
	public RealMatrix leftHandSideKt(RealMatrix D, RealMatrix H, RealMatrix R ) {
		return D.multiply(H.transpose()).add(R) ;	//level 3	
	}
		
	/** 
	 * (K': The QR solution of transpose of the Kalman gain and blending matrix)
	 * @return <b>K'</b> = E \ D
	 * @param E Left Hand Side K'
	 * @param D Right Hand Side of K' 
	 * */
	public RealMatrix solveTransposedKalmanGain(RealMatrix E, RealMatrix D ) {
		return new QRDecomposition( E ).getSolver().solve(D);		//LA JAMA Style
//		RRQRDecomposition decomp = new RRQRDecomposition(E, 0.0001);
//		return decomp.getSolver().solve(D);	
	}
		
	/** 
	 * innovation w: The blas level 2 computation of a priori filter residual of measurement z
	 * @param H observation matrix
	 * @param z measurements
	 * @param x state vector 
	 * @return z - Hx: (a priori==innovation residuals; a posteriori==measurement residuals)
	 * */
	public RealVector residuals(RealMatrix H, RealVector z, RealVector x ) {
		return z.subtract( H.operate(x));  //level 2
	}

	public RealVector residualsNormalized(RealMatrix H, RealVector z, RealVector x, RealVector sigma ) {
		return (z.subtract( H.operate(x))).ebeDivide(sigma);  //level 2
	}

	/**
	 * state: The blas level 2 measurement update of x
	 * @param K_T,w,x_   */
	public RealVector updateState(RealMatrix K_T, RealVector w, RealVector x_ ) {
		return  x_.add( K_T.preMultiply(w) ) ;	//level 2		
	}
		
	/** 
	 * covariance: The blas level 3 measurement update of P
	 * @param Kt,G,P  */
	public RealMatrix updateTransposedCovariance(RealMatrix Kt, RealMatrix D, RealMatrix P ) {
		return  P.subtract(D.transpose().multiply(Kt));                //level 3			
	}
	
	/** 
	 * state: propagate apriori x_ = Ax
	 * @param S RealMatrix: S+I==A   
	 * @param x RealVector 
	 * @return  RealVector x_ = x+Sx (a priori) */
	public RealVector propagateState(RealMatrix S, RealVector x ) {
		return x.add(S.operate(x)); 		
	}
	
	/** 
	 * covariance: apriori propagate P_
	 * @param S,Q,P  */
	public RealMatrix propagateTransposedCovariance(RealMatrix S, RealMatrix q2, RealMatrix P) {
		RealMatrix P1 = P.add(P.multiply(S.transpose()));  // Sparse level 3
		return P1.add(S.multiply(P1)).add(q2);             // Sparse Level 3
	}
	


	/** Edit logic for innovations outliers...
	 * 
	 * @param m number of sensor instruments mapped to filter
	 * @return number of innovations in update
	 */
	public int thresh(int m ) {

		double threshold;

		if(_STEADY) {
			threshold =  _Z_EDIT_STEADY; //default thresh value later overridden.
		} else if(_MANEUVER) {
			threshold =  _Z_EDIT_MANEUVER;
		} else if(_ACQUIRE) {
			threshold =  _Z_EDIT_ACQUISITION;
		} else if(_ASSOCIATE) { //_ASSOCIATE
			threshold =  _Z_EDIT_ASSOCIATION;
		} else {//
			threshold = Double.NaN;
			System.out.println("****** ERR! Undefined Track Filter MODE!!! ******************************");
			System.out.println("Undetected: NOT in {_STEADY, _MANEUVER, _ACQUISITION, _ASSOCIATE}.");
		}

		//m possible measurements reduced by map to mRow measurements
		double Z_thresh;
		int mRow = 0;
		for(int g = 0; g < m; g++) { //edit z outliers from H,R,z,w
			if(_STEADY) threshold = _editThreshold.getEntry(g); //probably greater than 1, less than 3 ...
			double sigmaMeasure = _sigmaMeasurement.getEntry(g);
			if (StrictMath.abs(_w.getEntry(g)) < threshold * _sigmaMeasurement.getEntry(g)) {//* _R.getEntry(g,g)) {
				_H.setRowMatrix(mRow, (_H.getRowMatrix(g)));
				_sigmaMeasurement.setEntry(mRow, sigmaMeasure);
				_editThreshold.setEntry(mRow,_editThreshold.getEntry(g)); //
				_R.setEntry(mRow, mRow, sigmaMeasure * sigmaMeasure );// _R.getEntry(g, g));
				_z.setEntry(mRow, _z.getEntry(g));
				_w.setEntry(mRow, _w.getEntry(g));
				mapPed[mRow] = mapPed[g];
				mapSensor[mRow] = mapSensor[g];
				mRow = mRow +1;
			} else {
				System.out.println("Track measurement edited: Ped "+mapPed[g]+" Sens "+mapSensor[g] );
			}
		}
		return mRow;
	}
	
	/** form the measurement data into observation and measurements
	 * 
	 * @param ensemble
	 * @return number of instruments mapped to filter
	 */
	public int formMeasurementEnsemble(Ensemble ensemble) {
		this._H = MatrixUtils.createRealMatrix(_numMeas, 3); //force zeros all structure
		int instr = 0;		
		this._position = new TVector(_x_.getSubVector(0, 3));

		//Declare local working variables...

		TVector filterVector = new TVector(TVector.EMPTY);
		TVector pedLocation = new TVector(TVector.EMPTY);
		TVector projectI = new TVector(TVector.EMPTY);
		TVector projectJ = new TVector(TVector.EMPTY);
		TVector projectK = new TVector(TVector.EMPTY);

		Polar filterPlot = new Polar();
		Polar radar = new Polar();

		Rotator local = new Rotator();
		Polar VideoResiduals = new Polar();

		//For each pedestal in group...
		for (int n = 0; n < ensemble.size(); n++) {

			//Done for every pedestal in group...(1:1) with pedestal instruments...
			Pedestal pedestal = ensemble.get(n);
			radar.set(pedestal.getLocal()); //get this n^(th) pedestal radar coordinates for target vector plot {rae}
			local.set(pedestal.getLocationFrame().getLocal()); //local orientation rotator {NED} from {EFG}

			pedLocation.set(pedestal.getLocalCoordinates()); // pedestal surveyed-location vector from local filter origin {EFG}

			filterVector.set(_x_.getSubVector(0, 3).subtract( pedLocation.realVector() ),0);  //filter track position from pedestal {EFG}
			filterPlot.set(Polar.commandLocal(filterVector, local)); //filter track position from pedestal as Polar plot {RAE}

			//Done for every pedestal in group...(1:1) with pedestal instruments...

			VideoResiduals.set(new Polar(     //Pedestal aperture video residuals
					radar.getRange() - filterPlot.getRange()
					,radar.getSignedAzimuth().subtract(filterPlot.getSignedAzimuth()).signedPrinciple().multiply(StrictMath.cos(filterPlot.getElevation().getRadians()))
					,radar.getElevation().subtract(filterPlot.getElevation()).signedPrinciple()
			));

			_predictedVideoResiduals.add( n, VideoResiduals ); //add pedestal video residuals to prediction list

			System.out.println( "**** Pedestal Sensor Predicted Video Residuals **** " + pedestal.getSystemId()
					+ "\t dRng " + _predictedVideoResiduals.get(n).getRange()
					+ "\t dHrz " + _predictedVideoResiduals.get(n).getSignedAzimuth().getDegrees()
					+ "\t dVrt " + _predictedVideoResiduals.get(n).getElevation().getDegrees()
					+ "\t (a priori) RAE: "	+ filterPlot.getRange() + "  " + filterPlot.getUnsignedAzimuth().getDegrees()+ "  " + filterPlot.getElevation().getDegrees() );	//Process pedestals sensor[instr]: (those mapped to filter update...)

			//INSTRUMENTS:
			// add rows to H, z vector, and diag(R) (projection ped sensor measurements)!!!!

			if (pedestal.getMapRG()) {

				projectI.set(pedestal.getAperture_i().unit());

				_H.setRow(instr, projectI.doubleArray());
				_z.setEntry(instr, radar.getRange() + pedLocation.getInnerProduct(projectI)); // meters track projection measured
				_sigmaMeasurement.setEntry(instr, pedestal.getDeviationRG()); // meters I error model
				_editThreshold.setEntry(instr,2.0); // thresh should be read with measurement model like measurements[n].getDeviationAZ()!!!!
				mapPed[instr] = n; //look up pedestal by instrument in filter solution
				mapSensor[instr] = 0;
				instr += 1;
			}

			if ((pedestal.getMapAZ()) || (pedestal.getMapEL())) {

				if (pedestal.getMapAZ()) {

					projectJ.set(pedestal.getAperture_j().unit());

					_H.setRow(instr, projectJ.doubleArray());
					_z.setEntry(instr, pedLocation.getInnerProduct(projectJ)); // meters J error model
//					_sigmaMeasurement.setEntry(instr, pedestal.getDeviationAZ().getRadians() * filterPlot.getRange()); ////@MAS: Need '* Cos[EL]' multiplier! // innovation meters
					_sigmaMeasurement.setEntry(instr, pedestal.getDeviationAZ().getRadians()
							* filterPlot.getRange() * StrictMath.cos(filterPlot.getElevation().getRadians()) ); ////@MAS: Need '* Cos[EL]' multiplier! // innovation meters
					_editThreshold.setEntry(instr,3.0); // thresh should be read with measurement model like measurements[n].getDeviationAZ()!!!!
					mapPed[instr] = n;
					mapSensor[instr] = 1;
					instr += 1;
				}

				if (pedestal.getMapEL()) {

					projectK.set(pedestal.getAperture_k().unit());

					_H.setRow(instr, projectK.doubleArray());
					_z.setEntry(instr, pedLocation.getInnerProduct(projectK)); // meters K error model
					_sigmaMeasurement.setEntry(instr, pedestal.getDeviationEL().getRadians() * filterPlot.getRange()); // innovation meters
					_editThreshold.setEntry(instr,3.0); // thresh should be read with measurement model like measurements[n].getDeviationAZ()!!!!
					mapPed[instr] = n;
					mapSensor[instr] = 2;
					instr += 1;
				}
			}
		}
		return instr;
	}
	
	
	/**
	 * This routine updates the filter state with new pedestal measurements ...
	 */
	public RealVector filter(double time, Ensemble ensemble) {
						
//		//Compile mapped measurements: {(H|z),R}
//		int instr = formMeasurementEnsemble(ensemble);
		int instr = 0;




		//Prediction update (a priori)		
		long msTimeAdvOld = _msecAdv;
		this._msecAdv = StrictMath.round((time - this._timePrev) * 1000);
		if(_msecAdv < 10 ) {
			_msecAdv = 20; //assume dup frame time
		}
		if( ( _msecAdv!=msTimeAdvOld ) && ( _msecAdv == 20 ) ) {			
			_S = init_S(((double) _msecAdv) * 0.001);		
			System.out.println("This is when initialized S:"+time);
		}		
		
		this._timePrev = time;

		if(_ASSOCIATE) System.out.println("\n ***Associate Track*** " + time);
		if(_ACQUIRE) System.out.println("\n ***Acquire Track*** " + time);
		if(_MANEUVER) System.out.println("\n ***Maneuver Track*** " + time);
		if(_STEADY) System.out.println("\n ***Steady Track*** " + time);


		if (_ASSOCIATE) { //measurements track associate...
			//Compile mapped measurements: {(H|z),R}
			instr = formMeasurementEnsemble(ensemble);

			RealMatrix a = _H.getSubMatrix(0, instr - 1, 0, 2);//copy();
			SingularValueDecomposition svd = new SingularValueDecomposition(a.getSubMatrix(0, instr - 1,0,2));
			p_point = svd.getSolver().solve(_z.getSubVector(0, instr)); //proxy plot

			_w = _z.subtract(_H.operate(p_point));

			//edited measurement set
			//instr = thresh(instr, _Z_EDIT_ASSOCIATION);
			instr = thresh(instr); //requires _w

			if(instr>=_CRIT_NUMBER_CONVERGE) { //ensemble plot convergence...[independent crit number?]
				svd = new SingularValueDecomposition(_H.getSubMatrix(0, instr - 1,0,2));
				p_point = svd.getSolver().solve(_z.getSubVector(0, instr)); 
				_w = _z.subtract(_H.operate(p_point));				
			}
			
			if(_w.getNorm()/StrictMath.sqrt(instr) > _Z_ACQUISITION) { //ensemble plot divergence...
				plotCount = 0;
				v_point_prior = new ArrayRealVector(3);
			}

			if (plotCount > 0) { //improve average velocity over expanding plot associations
				v_point.setSubVector(0,
						v_point_prior.add(((p_point.subtract(p_point_prior).mapMultiply(50)).subtract(v_point_prior))
								.mapMultiply(1.0 / (plotCount))));
			}
			
			if (plotCount > _ENSEMBLES_CONVERGENCE) { //track acquired!!!!
				_ASSOCIATE = false;
				_ACQUIRE = true;
				System.out.println("\n *** to Acquire Track*** " + time);

			}
			_x.setSubVector(0, p_point);
			_x.setSubVector(3, v_point);
			_x.setEntry(6, 0.0);
			_x.setEntry(7, 0.0);
			_x.setEntry(8, 0.0);
			
			//store reduced prior plot-track associations
			p_point_prior.setSubVector(0, p_point);
			v_point_prior.setSubVector(0, v_point);	
			
			plotCount = plotCount + 1;
			cntInit = cntInit +1;	
			
		} else { //measurements track plot...
			
			//prediction updates:
			_x_ = propagateState( _S, _x );


			//Compile mapped measurements: {(H|z),R}
			instr = formMeasurementEnsemble(ensemble);


			_P_ = propagateTransposedCovariance( _S,  _Q, _P );	
			_P_ = _P_.transpose().add(_P_).scalarMultiply(1/2.0); //enforce symmetry
			_w = residuals(_H.getSubMatrix(0, instr - 1, 0, 2), _z.getSubVector(0, instr),
					_x_.getSubVector(0, 3));

			//get threshed instr count....
//			if(_STEADY) {
//				instr = thresh(instr, _Z_EDIT_STEADY);
//			} else if(_MANEUVER) {
//				instr = thresh(instr, _Z_EDIT_MANEUVER);
//			} else if(_ACQUIRE) {
//				instr = thresh(instr, _Z_EDIT_ACQUISITION);
//			} else {
//
//			}
			instr = thresh(instr); //requires _w
			if (instr > 0) { // Have new measurements to process an update:

				_D = rightHandSideKt( // (mx3)(3x9) == (mx9)
						_H.getSubMatrix(0, instr - 1, 0, 2), _P_.getSubMatrix(0, 2, 0, 8));
				_E = leftHandSideKt( // (mx3)(3xm)+(mxm) == (mxm)
						_D.getSubMatrix(0, instr - 1, 0, 2), _H.getSubMatrix(0, instr - 1, 0, 2),
						_R.getSubMatrix(0, instr - 1, 0, instr - 1));
				_Kt = solveTransposedKalmanGain( // (mxm) \ (mxp) == (mxp)
						_E.getSubMatrix(0, instr - 1, 0, instr - 1), _D.getSubMatrix(0, instr - 1, 0, 8));
				_x = updateState( // (m) + (pxm)(m) == (m)
						_Kt.getSubMatrix(0,instr-1,0,8), _w.getSubVector(0,instr), _x_);
				_P = updateTransposedCovariance( // (9x9) - (9xm)(mx9) = (9x9)
						//_Kt.getSubMatrix(0, instr - 1, 0, 8), _D.getSubMatrix(0, instr - 1, 0, 8), _P_);
				        _Kt.getSubMatrix(0, instr - 1, 0, 8), _D.getSubMatrix(0, instr - 1, 0, 8), _P_);
				_e = residuals( // (m) - (m,3)(3) == (m) OR: --> _e = _w + _x - _x_
						_H.getSubMatrix(0, instr - 1, 0, 2)
						, _z.getSubVector(0, instr)
						, _x.getSubVector(0, 3) );

				_eN = residualsNormalized( // (m) - (m,3)(3) == (m) OR: --> _e = _w + _x - _x_
						_H.getSubMatrix(0, instr - 1, 0, 2)
						, _z.getSubVector(0, instr)
						, _x.getSubVector(0, 3)
						, _sigmaMeasurement.getSubVector(0,instr)      );

//				averageResidual = _e.getSubVector(0, instr).getNorm();

				double sumNormalizedResiduals = 0;
				for (int h = 0; h< instr; h++){
					sumNormalizedResiduals += _eN.getEntry(h);
				}
				_Z_NormalizedResidual = sumNormalizedResiduals / instr; /// StrictMath.sqrt((double) instr);
				System.out.println("\n *** Z from ave Normalized residuals  = " + _Z_NormalizedResidual);

//				double averageNormalizedResidual = sumNormalizedResiduals/instr;
//				System.out.println(" *** ave Norm res.    = " + averageNormalizedResidual);
//
//				double _rmsNormalizedResidual = _eN.getSubVector(0, instr).getNorm() / StrictMath.sqrt((double) instr);
//				System.out.println(" *** rms Norm res.    = " + _rmsNormalizedResidual);

//				//System.out.println("\n ***Innovations Norm = " + _w.getNorm() * StrictMath.sqrt(instr - 3));
//				System.out.println("***Ped.Sensor Normalized Residuals & Residuals & Innovations & sigmas = { eN, e, w, sigma");
//				for (int h = 0; h < instr; h++) {
//					System.out.print("\tP"+ mapPed[h] +":S" + mapSensor[h] + "\t"+_eN.getEntry(h)+", \t"+_e.getEntry(h)+", \t"+_w.getEntry(h) + ", \t" +_sigmaMeasurement.getEntry(h)+ "\n  ");
//				}
//				System.out.print("   } \n");

				System.out.println("\t ped:sensor  \tresiduals (m)\t           innov (m)\t        sigmas(m)\t           truncZ(resid)\t            truncZ(innov)\t");
				for (int h = 0; h < instr; h++) {
					System.out.print("\t "+ mapPed[h] +":S" + mapSensor[h]
							+ "    \t" + _e.getEntry(h) + "    \t"+_w.getEntry(h) + "    \t" +  _sigmaMeasurement.getEntry(h)
							+ "    \t" + _e.getEntry(h) / _sigmaMeasurement.getEntry(h) +"    \t" + _w.getEntry(h) / _sigmaMeasurement.getEntry(h) + "\n  ");
				}


			} else { // Have no new measurements to process an update:
				_x = _x_.copy();
				_P = _P_.copy();
			}
			
		}

		//Auto-tuning of normalized residuals logic

		System.out.println("Z from normalized residuals = " + _Z_NormalizedResidual);

		if(_STEADY) {
			if((_Z_NormalizedResidual <= _Z_STEADY)&&(instr>_CRIT_NUMBER_LOOSEN)) {
				cntSteady = cntSteady+1;
			} else {
				_MANEUVER = true;
				_STEADY = false;
				System.out.println("\n *** to Maneuver Track*** " + time);
				_autoTune_Q = _MANEUVER_Q;
				cntManeuver = cntManeuver + 1;
			}
		} else if(_MANEUVER) {
			if((_Z_NormalizedResidual <= _Z_STEADY)&&(instr>=_CRIT_NUMBER_TIGHTEN)) {
				_STEADY = true;
				_MANEUVER = false;
				System.out.println("\n *** to STEADY Track*** " + time);
				_autoTune_Q = _STEADY_Q;
				cntSteady = cntSteady + 1;
			} else if((_Z_NormalizedResidual > _Z_MANEUVER)||(instr<=_CRIT_NUMBER_LOOSEN)) {
				_ACQUIRE = true;
				_MANEUVER = false;
				System.out.println("\n *** to Acquire Track*** " + time);
				_autoTune_Q = _MANEUVER_Q;
				cntAcquisition = cntAcquisition + 1;
			} else { //(_MANEUVER)
				cntManeuver = cntManeuver + 1;
			}
		} else if(_ACQUIRE) {
			if((_Z_NormalizedResidual <= _Z_MANEUVER)&&(instr>=_CRIT_NUMBER_TIGHTEN)) {
				_MANEUVER = true;
				_ACQUIRE = false;
				System.out.println("\n *** to Maneuver Track*** " + time);
				_autoTune_Q = _MANEUVER_Q;
				cntManeuver = cntManeuver + 1;
			} else if((_Z_NormalizedResidual > _Z_ACQUISITION)||(instr<=_CRIT_NUMBER_LOOSEN)) {
				cntDivergence = 0;
				cntAcquisition = cntAcquisition +1;
			} else {
				cntDivergence = cntDivergence + 1;
				if(cntDivergence > _ENSEMBLES_DIVERGENCE) {
					_ACQUIRE = false;
					_ASSOCIATE = true;
					System.out.println("\n *** to Acquire Track*** " + time);
					cntDivergence = 0;
				}
				cntAcquisition = cntAcquisition +1;				
			}
		}
		
		// Export geocentric coordinate state of filter... could also be a forecast for
		// a look-ahead...
		_state = _x.copy();
		_state.setSubVector(0, (_origin.realVector().add(_x.getSubVector(0, 3))));

		_covariance = _P;
		return _state;

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

}