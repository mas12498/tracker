package tspi.model;

import tspi.rotation.*;

/**
 * Stores local geodetic [WGS84] definition:<p>
 * <p> Topocentric coordinate frame (local geodetic)
 * <br>-- (Operator) Geocentric rotator to local orientation of geodetic navigation {N,E,D}
 * <br>-- (double) Helmert height above ellipsoid from which to calculate gravity potentials and elevations 
 *
 * <p> Class includes methods for generating: 
 * <br>-- (Vector3) Cartesian geocentric coordinates {E,F,G}
 * <br>-- (Ellipsoid) geodetic coordinates {latitude,longitude,height}
 *                          
 * @author mike
 *
 */
public class T_EFG_NED {	
	
	//Definition WGS84 ellipsoid.
//	public static final double _a = Ellipsoid._a; //WGS84 semi-major axis radius
//	public static final double _f = Ellipsoid._f; //WGS84 flattening

	//Conversion algorithm constants.
	protected static final double ZERO         = 0d;
	private static final double NEGATIVE_ZERO  = -ZERO;
	
	protected static final double ONE          = 1d;
	private static final double NEGATIVE_ONE   = -ONE;
	
	protected static final double TWO          = 2d;
	protected static final double THREE        = 3d;
	protected static final double ONE_THIRD    = ONE/THREE;
	protected static final double FOUR_THIRDS  = 4d/THREE;
	
	private static final double MIN_RATIO      = ONE - Ellipsoid._f; // = _b/_a 
    protected static final double MIN_RATIO_SQ = (MIN_RATIO)*(MIN_RATIO); // = (_b/_a)*(_b/_a)
    protected static final double DIFF_RATIOS  = (ONE - MIN_RATIO_SQ)/MIN_RATIO; //= (_a/_b) - (_b/_a);	

 //   protected static final double _b = Ellipsoid._b; //WGS84 semi-minor axis radius
    	
	//minimalist geodetic OP holding:
	protected final Rotator _local; //rotater
	protected double _height;  //translater		

	/**
	 * Geodetic local copy-constructor:
	 */
	public T_EFG_NED(Rotator local, double height){
		_height = height;
		_local = local;
	}

	
	/**
	 * Constructor: initialize 'Empty'.
	 */
	public T_EFG_NED(){
		_height = Double.NaN;
		_local = new Rotator(Rotator.EMPTY);
	}	
	
	/**
	 * Geodetic local copy-constructor:
	 */
	public T_EFG_NED(T_EFG_NED wgs84){
		_height = wgs84._height;
		_local = new Rotator(wgs84._local);
	}


	/**
	 * Factory: Rotator defines local Cartesian frame orientation from horizontal geodetic coordinates of local origin. 
	 * @return Rotator local
	 */
	
	public static Rotator local(Angle northGeodeticLatitude, Angle eastGeodeticLongitude)
	{
		Angle theta = new Angle(northGeodeticLatitude).add(Angle.RIGHT).negate();
		return QuaternionMath.eulerRotate_kj(eastGeodeticLongitude.codedPhase(),theta.codedPhase());
	}


	public static Rotator local(CodedPhase northGeodeticLatitude, CodedPhase eastGeodeticLongitude){
		CodedPhase theta = new CodedPhase(northGeodeticLatitude).addRight().negate();
		return QuaternionMath.eulerRotate_kj(eastGeodeticLongitude,theta);
	}


	/**
		 * Factory: Rotator from local navigation coordinate frame {N,E,D}
		 * to geocentric frame {E,F,G}. 
		 * 
		 * @return Operator quaternion {w,xI,yJ,zK}
		 */
		public Rotator getLocal(){
	//		//System.out.println("inputs axial OP: "+_latitude.getDegrees()+"  "+_longitude.getDegrees());
	//		Angle theta = new Angle(_latitude).add(Angle.QUARTER_REVOLUTION).negate();
	//		return QuaternionMath.eulerRotate_kj(_longitude.getPrinciple(),theta.getPrinciple());
			
			return new Rotator(_local);
			
		}


	/**
	 * @return meters vertical (Helmert height) above WGS 84 ellipsoid
	 */
	public double getLocalHeight() {
		return _height;
	}


	/**
		 * [Recovery...]
		 * Sets geodetic north latitude and geodetic east longitude as extracted from q_NG.
		 * @param q_NG rotator defining local geodetic frame alignment
		 */
	//	public void setEllipsoidHorizontal(Operator q_NG) { 
		public Ellipsoid getEllipsoid(){ //Operator q_NG) { 
			double dump = _local.getEuler_i_kji().tanHalf();
			CodedPhase pLat = _local.getEuler_j_kj().addRight().negate();
			CodedPhase pLon = _local.getEuler_k_kj();
			Ellipsoid ellipsoid = new Ellipsoid();
			ellipsoid.setNorthLatitude(pLat.angle().signedPrinciple());
			ellipsoid.setEastLongitude(pLon.angle().unsignedPrinciple());			ellipsoid.setEllipsoidHeight(_height);
			ellipsoid.setEllipsoidHeight(_height);	
			return ellipsoid;
		}


	public void set(T_EFG_NED frame){
		_local.set(frame.getLocal());
		_height = frame._height;
	}
	
	public void setLocal(Rotator geocentric_to_topocentric){
		_local.set(geocentric_to_topocentric);
	}

	public void setLocalHeight(double heightGeodetic){
		_height = heightGeodetic;
	}

	/**
	 * Setter: by Ellipsoid coordinates.
	 * @param coordinates
	 */
	public void set(Ellipsoid coordinates){
		_height = coordinates._height; //pass thru...
		_local.set(local(coordinates._latitude,coordinates._longitude));
	}

	/** 
	 * @param geocentricEFG earth-centered, earth-fixed Cartesian position
	 */
	public void set(Vector3 geocentricEFG){	
		
	    double x= geocentricEFG.getX(); //E
	    double y= geocentricEFG.getY(); //F
	    double z= geocentricEFG.getZ(); //G   
	
	    /* 2.0 compute intermediate values for latitude */
	    double r= StrictMath.hypot(x, y); //.sqrt( x*x + y*y );
	    double e = (StrictMath.abs(z) / Ellipsoid._a - DIFF_RATIOS) / (r / Ellipsoid._b);
		double f = (StrictMath.abs(z) / Ellipsoid._a + DIFF_RATIOS) / (r / Ellipsoid._b);
	    
	    /* 3.0 Find solution to: t^4 + 2*E*t^3 + 2*F*t - 1 = 0  */
	    double p= FOUR_THIRDS * (e*f + ONE);
	    double q= TWO * (e*e - f*f);
	    
	    double d = p*p*p + q*q;
	    double v;
	    if( d >= ZERO ) {
	            v= StrictMath.pow( (StrictMath.sqrt( d ) - q), ONE_THIRD )
	             - StrictMath.pow( (StrictMath.sqrt( d ) + q), ONE_THIRD );
	    } else {
	            v= TWO * StrictMath.sqrt( -p )
	             * StrictMath.cos( StrictMath.acos( q/(p * StrictMath.sqrt( -p )) ) / THREE );
	    }
	    
	    /* 4.0 Improve v. NOTE: not really necessary unless point is near pole */
	    if( v*v < StrictMath.abs(p) ) {
	            v= -(v*v*v + TWO*q) / (THREE*p);
	    }
	    double g = (StrictMath.sqrt( e*e + v ) + e) / TWO;
	    double t = StrictMath.sqrt( g*g  + (f - v*g)/(TWO*g - e) ) - g;
			
		double kSinLat = ONE - t*t;
		double kCosLat = TWO*t*StrictMath.copySign(MIN_RATIO,z);
		double k = StrictMath.hypot(kSinLat, kCosLat);
		
		/* 5.0 Calculate ellipsoid height WGS84 */	    
		double B = (z<ZERO)?-Ellipsoid._b:Ellipsoid._b;	    
		_height = ((r - Ellipsoid._a * t) * kCosLat + (z - B) * kSinLat)/k;

		
		/* 6.0 Perform direct Principle computations for ellipsoid WGS84 coordinates */
		
		double cosLon = x/r; // /r always positive.
		CodedPhase plon = CodedPhase.encodes(StrictMath.copySign(StrictMath.sqrt((ONE-cosLon)/(ONE+cosLon)),y));
		Angle theta = Angle.inRadians(StrictMath.atan(kSinLat/kCosLat)).add(Angle.RIGHT).negate(); //codes as cotangent of half-latitude
		CodedPhase ptheta = theta.codedPhase();

		//_localHorizontal 	
		if (plon.isAcute()) {
			if (ptheta.isAcute()) { //acute lon, acute theta
				if (ptheta.isZero()) {
					_local.set(ONE, ZERO, ZERO, plon.tanHalf());
					//_localHorizontal.unit();
					return;
				}
				_local.set(ONE, -ptheta.tanHalf() * plon.tanHalf(), ptheta.tanHalf(), plon.tanHalf());
				//_localHorizontal.unit();
				return;
			} //acute lon, obtuse theta
			if (ptheta.isStraight()) {
				_local.set(0, -plon.tanHalf(), ONE, ZERO);
				//_localHorizontal.unit();
				return;
			}
			_local.set(ONE / ptheta.tanHalf(), -plon.tanHalf(), ONE, plon.tanHalf() / ptheta.tanHalf());
			//_localHorizontal.unit();
			return;
		} //obtuse lon, acute theta
		if (ptheta.isAcute()) {
			if (ptheta.isZero()) {
				_local.set(ONE / plon.tanHalf(), ZERO, ZERO, ONE);
				//_localHorizontal.unit();
				return;
			}
			_local.set(ONE / plon.tanHalf(), -ptheta.tanHalf(), ptheta.tanHalf() / plon.tanHalf(), ONE);
			//_localHorizontal.unit();
			return;
		} //obtuse lon, obtuse theta
		if (ptheta.isStraight()) {
			_local.set(ZERO, NEGATIVE_ONE, ONE / plon.tanHalf(), ZERO);
			//_localHorizontal.unit();
			return;
		}
		double cotHalfTheta = ONE / ptheta.tanHalf();
		_local.set(cotHalfTheta / plon.tanHalf(), NEGATIVE_ONE, ONE / plon.tanHalf(), cotHalfTheta);
		//_localHorizontal.unit();
		return;
	}

	

//	/**
//	 * @param meters above WGS reference ellipsoid.
//	 */
//	public void setEllipsoidHeight(double meters) {
//		this._height = meters;
//	}
//
//	/** 
//	 * @param latitude defined positive in northern hemisphere
//	 */
//	public void setNorthLatitude(Angle latitude) {
//		_latitude.set(latitude);
//		
//	}

		
//		/** Setter:
//		 * @param Location East longitude
//		 */
//		public void setEastLongitude(Angle eastLongitude) {
//			_longitude.set(eastLongitude);
//		}

			
		/**
	 * Factory: Cartesian position coordinates of this GeodeticLocation, earth-centered and earth-fixed. 
	 * @return Vector3 geocentric {E,F,G}
	 */
	public Vector3 getGeocentric(){
		
		//trig unnecessary[?]
		
		//double sin_lat = getEast
		Ellipsoid local = getEllipsoid();
		return local.getGeocentric();
//		double latitudeRadians = local.getNorthLatitude().getRadians();
//		
//		
//		double sin_lat = StrictMath.sin(latitudeRadians);
//		
//		double rPE = _a / StrictMath.sqrt(ONE - FLATFN * sin_lat * sin_lat);
//		double x = (rPE + local.getEllipsoidHeight()) * StrictMath.cos(latitudeRadians);
//		double longitudeRadians = local.getEastLongitude().getRadians();
//		return new Vector3(
//				x * StrictMath.cos(longitudeRadians), 
//				x * StrictMath.sin(longitudeRadians),
//				(rPE * FUNSQ + local.getEllipsoidHeight()) * sin_lat
//		);
		
	}

		/** 
		 * Clears this Geodetic location -- re-initializes as empty.
		 */
		public void clear(){			
			_height = Double.NaN;
			_local.set(Rotator.EMPTY);
		}
	
}
