package tspi.model;

import tspi.rotation.Angle;
import tspi.rotation.Vector3;
/**
 * Transfer Object for ellipsoid coordinates.
 * 
 * @author mike
 *
 */
public class Ellipsoid {
	/* Reference ellipsoid coordinates: Defined per WGS84 */  
	public static final double _a = 6378137.0d; //WGS84 semi-major ellipsoid radius meters
	public static final double _f = 0.00335281068118d; //WGS84 flattening (unitless)
    public static final double _b = _a * (1 - _f); //WGS84 semi-minor axis radius
    protected static final double FLAT_FN = (2 - _f)*_f; //== 1-((1-f)^2) == 1 - RATIO_SQ

	protected final Angle _latitude;
	protected final Angle _longitude;
	protected double _height;
	
	protected Ellipsoid(Angle latitude, Angle longitude, double height){
		_latitude = new Angle(latitude);
		_longitude = new Angle (longitude);
		_height = height;
	}

	public Ellipsoid(){
		this(Angle.EMPTY, Angle.EMPTY, Double.NaN);
	}

	public Ellipsoid(Ellipsoid coordinates) {
		this(coordinates._latitude, coordinates._longitude, coordinates._height);
	}

	public void set(Angle latitude, Angle longitude, double height){
		_latitude.set(latitude);
		_longitude.set(longitude);
		_height = height;
	}
	
	public void set(Ellipsoid local){
		_latitude.set(local._latitude);
		_longitude.set(local._longitude);
		_height = local._height;
	}
	
	/**
	 * @return the North latitude
	 */
	public Angle getNorthLatitude() {
		return _latitude;
	}

	/**
	 * @param set North latitude.
	 */
	public void setNorthLatitude(Angle latitude) {
		_latitude.set(latitude);
	}

	/**
	 * @return the East longitude
	 */
	public Angle getEastLongitude() {
		return _longitude;
	}
	
	/**
	 * @param set East longitude
	 */
	public void setEastLongitude(Angle longitude) {
		this._longitude.set(longitude);
	}

	/**
	 * @return the _height
	 */
	public double getEllipsoidHeight() {
		return _height;
	}

	/**
	 * @param _height the _height to set
	 */
	public void setEllipsoidHeight(double height) {
		this._height = height;
	}
	
//	public void set(Angle latitude,Angle longitude, double metersHeight){
//		_latitude.set(latitude);
//		_longitude.set(longitude);
//		_height = metersHeight;
//	}
	
	public Vector3 getGeocentric(){
		double ellipsoidLatitudeRadians = this.getNorthLatitude().getRadians();
		double sinEllipsoidLatitude = StrictMath.sin(ellipsoidLatitudeRadians);
		double radiusInflatedEllipsoid = Ellipsoid._a 
				/ StrictMath.sqrt(T_EFG_NED.ONE - FLAT_FN * sinEllipsoidLatitude * sinEllipsoidLatitude);
		double rCosEllipsoidLatitiude = (radiusInflatedEllipsoid + this.getEllipsoidHeight())
				* StrictMath.cos(ellipsoidLatitudeRadians);
		double ellipsoidLongitudeRadians = this.getEastLongitude().getRadians();
		return new Vector3( //Geocentric EFG
				rCosEllipsoidLatitiude * StrictMath.cos(ellipsoidLongitudeRadians),
				rCosEllipsoidLatitiude * StrictMath.sin(ellipsoidLongitudeRadians),
				sinEllipsoidLatitude * (radiusInflatedEllipsoid * T_EFG_NED.MIN_RATIO_SQ + this.getEllipsoidHeight()));
	}	

	public void setGeocentric(Vector3 geocentricEFG){	
	
	    double x= geocentricEFG.getX(); //E
	    double y= geocentricEFG.getY(); //F
	    double z= geocentricEFG.getZ(); //G   
	
	    /* 2.0 compute intermediate values for latitude */
		double r = StrictMath.hypot(x, y);
		double s = r / _b;
		double e = (StrictMath.abs(z) / _a - T_EFG_NED.DIFF_RATIOS) / s;
		double f = (StrictMath.abs(z) / _a + T_EFG_NED.DIFF_RATIOS) / s;
	    
	    /* 3.0 Find solution to: t^4 + 2*E*t^3 + 2*F*t - 1 = 0  */
	    double p= T_EFG_NED.FOUR_THIRDS * (e*f + T_EFG_NED.ONE);
	    double q= T_EFG_NED.TWO * (e*e - f*f);
	    
	    double d = p*p*p + q*q;
	    double v;
	    if( d >= T_EFG_NED.ZERO ) {
	            v= StrictMath.pow( (StrictMath.sqrt( d ) - q), T_EFG_NED.ONE_THIRD )
	             - StrictMath.pow( (StrictMath.sqrt( d ) + q), T_EFG_NED.ONE_THIRD );
	    } else {
	            v= T_EFG_NED.TWO * StrictMath.sqrt( -p )
	             * StrictMath.cos( StrictMath.acos( q/(p * StrictMath.sqrt( -p )) ) / T_EFG_NED.THREE );
	    }
	    
	    /* 4.0 Improve v. NOTE: not really necessary unless point is near pole */
	    if( v*v < StrictMath.abs(p) ) {
	            v= -(v*v*v + T_EFG_NED.TWO*q) / (T_EFG_NED.THREE*p);
	    }
	    double g = (StrictMath.sqrt( e*e + v ) + e) / T_EFG_NED.TWO;
	    double t = StrictMath.sqrt( g*g  + (f - v*g)/(T_EFG_NED.TWO*g - e) ) - g;
	
	    /* 5.0 Set B sign to get sign of latitude and height correct */
	    double B = (z<T_EFG_NED.ZERO)?-_b:_b;
		    
	    _longitude.setRadians(StrictMath.atan2( y, x ));
	    
	    _latitude.setRadians(StrictMath.atan( (_a*(T_EFG_NED.ONE - t*t)) / (T_EFG_NED.TWO*B*t) ));
	    
	    _height = (r - _a*t)*StrictMath.cos(_latitude.getRadians()) + (z - B)*StrictMath.sin(_latitude.getRadians());
	
	}





}
