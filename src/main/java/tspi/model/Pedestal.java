package tspi.model;

import tspi.rotation.Angle;
import tspi.rotation.Rotator;
import tspi.rotation.Vector3;

import java.util.Random;

/**  
 * Every thing native in EFG  WGS84 Cartesian frame
 * 
 * */
public class Pedestal {
	
	String _systemId; // system identifier
	
	/** Pedestal instruments available for TSPI: boolean values */
	final Boolean[] _mapSensors = new Boolean[3]; // == {hasRG,hasAZ,hasEL}

	/** Angular bias of the pedestal's measurements. Supplies mean of the distribution in Angles and Meters. */
	final Polar _bias = new Polar();
	
	/** Angular deviation of the pedestal's local.  Supplies deviation of the distribution in Angles and Meters. */
	final Polar _deviation = new Polar();
	// TODO just slapping stuff together; feel free to revise this error model!

	/** PEDESTAL GEODETIC LOCATION WGS 84 ELLIPSOID [LLh] in Angles and Meters.**/
	final Ellipsoid _geodeticLocation = new Ellipsoid();

	/** PEDESTAL LOCAL VECTOR [RAE: Range, Azimuth, Elevation -- NO ATMOSPHERE] **/
	final Polar _local = new Polar();

	/** LOCAL SITE TRANSFORMATION defined from geocentric XYZ. **/
	final T_EFG_NED _localFrame = new T_EFG_NED();

	/** LOCAL APERTURE TRANSFORMATION defined from geocentric XYZ. **/
	final T_EFG_FRD _apertureFrame = new T_EFG_FRD();

	/** PEDESTAL LOCATION WGS 84 GEOCENTRIC [XYZ] **/
	final Vector3 _location = new Vector3(Vector3.EMPTY); // geocentric vector: EFG

	/** PEDESTAL LOCATION WGS 84 GEOCENTRIC [XYZ] **/
	final Vector3 _localCoordinates = new Vector3(Vector3.EMPTY); // geocentric vector: EFG

	
	/** PEDESTAL VECTOR WGS 84 GEOCENTRIC ORIENTED [XYZ] **/
	final Vector3 _vector = new Vector3(Vector3.EMPTY); //local vector offset: EFG

	/** LOCAL SITE 'NORTH' DIRECTION in GEOCENTRIC [XYZ] frame. **/
	final Vector3 _unitNorth = new Vector3(Vector3.EMPTY); 
	
	/** LOCAL SITE 'EAST' DIRECTION in GEOCENTRIC [XYZ] frame. **/
	final Vector3 _unitEast = new Vector3(Vector3.EMPTY); 
	
	/** LOCAL SITE 'UP' DIRECTION in GEOCENTRIC [XYZ] frame. **/
	final Vector3 _unitUp = new Vector3(Vector3.EMPTY);
	
	// Static final...
	
	/** Geocentric origin of solution **/
	private static final Vector3 _origin = new Vector3(Vector3.EMPTY);
	
	/** LOCAL SITE TRANSFORMATION defined from geocentric XYZ. **/
	private static final T_EFG_NED _originFrame = new T_EFG_NED();

	private static final int DIGITS = 14;
	
	//Constructor
	public Pedestal( String id, Boolean hasRG, Boolean hasAZ, Boolean hasEL, Angle lat, Angle lon, double h) {
		//_wgs84: from Pedestal's geodetic-ellipsoid coordinates Latitude, Longitude, height
		this._systemId = id;
		this._geodeticLocation.set(lat,lon, h); //Ellipsoid
		this._localFrame.set(this._geodeticLocation);	//T_EFG_NED
		this._unitNorth.set(this._localFrame._local.getImage_i());
		this._unitEast.set(this._localFrame._local.getImage_j());
		this._unitUp.set(this._localFrame._local.getImage_k().negate());
		this._location.set(this._geodeticLocation.getGeocentric()); //Cartesian	location for next to make sense...	
		this.setMapSensors(hasRG, hasAZ, hasEL);
		this.setLocalOriginCoordinates();		//zeros for the first pedestal in ensemble defined to be origin... 
		this.clearPedestalVector();
		
	}	

	public static Vector3 getOrigin() { return new Vector3( _origin ); 	}
	public static T_EFG_NED getOriginFrame() { return new T_EFG_NED( _originFrame ); 	}

	public static void setOrigin(Vector3 geocentricOrigin) {
		_origin.set(geocentricOrigin);
		_originFrame.set(_origin);
	}
	
//	/**
//	 * set current pedestal's origin and location from local origin in geocentric coordinates.
//	 */
//	public void setLocalCoordinates(Vector3 originXYZ) { 
//		_origin.set(originXYZ);
//		_originFrame.set(_origin);
//		this._localCoordinates.set(new Vector3(this._location).subtract(_origin) );
//	}	
//
//	/**
//	 * set pedestal's origin and location from local origin in ellipsoid coordinates.
//	 */
//	public void setLocalCoordinates(Ellipsoid _originLLh) { 
//		_origin.set(_originLLh.getGeocentric());
//		_originFrame.set(_origin);
//		this._localCoordinates.set(new Vector3(this._location).subtract(_origin) );
//	}	
	
	
	/**
	 * set current Pedestal's instance location from local _origin.
	 */
	public void setLocalOriginCoordinates() { 
		this._localCoordinates.set(new Vector3(this._location).subtract(_origin) );
	}		

	//Clone Pedestal objects...
	public String getSystemId() { return this._systemId; }
	
	/**
	 * @return pedestal's location in geocentric coordinate frame.
	 */
	
	public Boolean[] getMapSensors() { return _mapSensors; }
	
	public void setMapSensors(Boolean hasRG, Boolean hasAZ, Boolean hasEL) { 
		this._mapSensors[0] = hasRG; 
		this._mapSensors[1] = hasAZ;
		this._mapSensors[2] = hasEL;
	}	
	

	public Boolean getMapRG() { return (this._mapSensors[0]); }
	public Boolean getMapAZ() { return (this._mapSensors[1]); }
	public Boolean getMapEL() { return (this._mapSensors[2]); }	
	public void setMapRG(Boolean hasRG) { this._mapSensors[0] = hasRG; }
	public void setMapAZ(Boolean hasAZ) { this._mapSensors[1] = hasAZ; }
	public void setMapEL(Boolean hasEL) { this._mapSensors[2] = hasEL; }
	
	

	/**
	 * @return pedestal's location in geocentric coordinate frame.
	 */
	public Vector3 getLocation() { return new Vector3(this._location); }


	/**
	 * @return pedestal's location from local origin given in geocentric oriented coordinate frame.
	 */
	public Vector3 getLocalCoordinates() { return new Vector3( _localCoordinates ); }

	
	public Polar getBias() { return _bias; }
	public void setBias(Polar bias) { this._bias.set(bias); }
	
	public double getBiasRG() { return this._bias.getRange(); }
	public Angle getBiasAZ() { return this._bias.getSignedAzimuth(); }
	public Angle getBiasEL() { return this._bias.getElevation(); }
	public void setBiasRG(double DRG) { this._bias.setRange(DRG); }
	public void setBiasAZ(Angle DAZ) { this._bias.setAzimuth(DAZ); }
	public void setBiasEL(Angle DEL) { this._bias.setElevation(DEL); }
	
	public Polar getDeviation() { return _deviation; }
	public void setDeviation(Polar deviation) { this._deviation.set(deviation); }
	
	public double getDeviationRG() { return this._deviation.getRange(); }
	public Angle getDeviationAZ() { return this._deviation.getSignedAzimuth(); }
	public Angle getDeviationEL() { return this._deviation.getElevation(); }
	public void setDeviationRG(Double sdRG) { this._deviation.setRange(sdRG); }
	public void setDeviationAZ(Angle sdAZ) { this._deviation.setAzimuth(sdAZ); }
	public void setDeviationEL(Angle sdEL) { this._deviation.setElevation(sdEL); }

	
	public Polar getPerturbedLocal(Random random) {
		// add a normally distributed error to each of the polar coordinates 
		// TODO There is another daz correcting term that has to be handled as elevation increases!!!	
		Polar perturbedLocal = new Polar(
				_local.getRange() + _bias.getRange() + _deviation.getRange() * random.nextGaussian(),
				Angle.inPiRadians(_local.getUnsignedAzimuth().getPiRadians() + _bias.getSignedAzimuth().getPiRadians()
						+ _deviation.getSignedAzimuth().getPiRadians() * random.nextGaussian()).unsignedPrinciple(),
				Angle.inPiRadians(_local.getElevation().getPiRadians() + _bias.getElevation().getPiRadians()
						+ _deviation.getElevation().getPiRadians() * random.nextGaussian()));
		return perturbedLocal;  
	}
	
	public Polar getBiasedLocal() {
		// add a normally distributed error to each of the polar coordinates 
		// TODO There is another daz correcting term that has to be handled as elevation increases!!!
		double biasRange = _bias.getRange();
		double biasAz = _bias.getSignedAzimuth().getPiRadians();
		double biasEl = _bias.getElevation().getPiRadians();
		
		if(Double.isNaN(biasRange)) biasRange = 0d;
		if(Double.isNaN(biasAz)) biasAz = 0d;
		if(Double.isNaN(biasEl)) biasEl = 0d;

		Polar biasedLocal = new Polar(
				_local.getRange() + biasRange,
				Angle.inPiRadians(_local.getSignedAzimuth().getPiRadians() + biasAz).unsignedPrinciple(),
				Angle.inPiRadians(_local.getElevation().getPiRadians() + biasEl)
		);
		return biasedLocal;  
	}
	
	/**
	 * @return pedestal's vector in geocentric-oriented coordinate frame.
	 */
	public Vector3 getVector() { return new Vector3(this._vector); }
		
	/**
	 * @return pedestal's vector in local geodetic topocentric frame (polar form): range, azimuth elevation.
	 */
	public Polar getLocal() { return new Polar(this._local); }

	/**
	 * @return model transform from geocentric frame coordinates to pedestal's aperture frame coordinates.
	 */
	public T_EFG_FRD getApertureFrame() { return new T_EFG_FRD(this._apertureFrame); }
	
	/**
	 * @return Vector3 unit-vector i direction ('F' is line-of-sight of {FRD}).
	 */
	public Vector3 getAperture_i() {
		return this._apertureFrame._orientation.getImage_i().unit();
	}
	
	/**
	 * @return Vector3 unit-vector j direction ('R' is normal to 'F' in level plane {FRD}).
	 */
	public Vector3 getAperture_j() {
		return this._apertureFrame._orientation.getImage_j().unit();
	}

	/**
	 * @return Vector3 unit-vector k direction ('D' is normal to 'F' in vertical plane {FRD}).
	 */
	public Vector3 getAperture_k() {
		return this._apertureFrame._orientation.getImage_k().unit();
	}
		
	/**
	 * @return pedestal's location in WGS 84's ellipsoid coordinates.
	 */
	public Ellipsoid getLocationEllipsoid() {return new Ellipsoid(this._geodeticLocation); }

	/**
	 * @return model transform from geocentric frame coordinates to pedestal's local topocentric frame coordinates.
	 */
	public T_EFG_NED getLocationFrame() { return new T_EFG_NED(this._localFrame); }
	

	//Mutators
	
	public void clearOrigin() {
		_origin.set( Vector3.EMPTY );
		_localCoordinates.set(Vector3.EMPTY);
	}


	public void clearPedestalLocation() {
		_geodeticLocation.set(Angle.EMPTY, Angle.EMPTY, Double.NaN);
		_location.set(Vector3.EMPTY);
		_localCoordinates.set(Vector3.EMPTY);
		_localFrame.clear();
		clearPedestalVector();
	};
	
	public void clearPedestalVector(){
		_local.set(Double.NaN, Angle.EMPTY, Angle.EMPTY);
		_apertureFrame.clear();
	};

	public void clearPedestalPerturbation(){
		_bias.set(Double.NaN, Angle.EMPTY, Angle.EMPTY);
		_deviation.set(Double.NaN, Angle.EMPTY, Angle.EMPTY);
    //@MAS //any thing downstream, too 
	};
	
	
	
	public void setSystemId(String id) { this._systemId = id; }
	

	public void locate(T_EFG_NED locationFrame){	
		this._localFrame.set(locationFrame);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 		
		this._geodeticLocation.set(_localFrame.getEllipsoid());		
		
		this._location.set(_geodeticLocation.getGeocentric());
		this.setLocalOriginCoordinates();		

	}
	
	public void locateHorizontal(Rotator horizontal){		
		this._localFrame.setLocal(horizontal);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
		this._geodeticLocation.set(_localFrame.getEllipsoid());
		
		this._location.set(_geodeticLocation.getGeocentric());
		this.setLocalOriginCoordinates();		

	}
	
	public void locateVertical(double vertical){		
		this._localFrame.setLocalHeight(vertical);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 		
		this._geodeticLocation.set(_localFrame.getEllipsoid());
		
		this._location.set(_geodeticLocation.getGeocentric());
		this.setLocalOriginCoordinates();		

	}
	
	
	
	public void locate(Ellipsoid wgs84) {
		this._geodeticLocation.set(wgs84);
		
		this._location.set(_geodeticLocation.getGeocentric());
		this.setLocalOriginCoordinates();		

		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 	
	}
	
	public void locateLatitude(Angle lat) {
		this._geodeticLocation.setNorthLatitude(lat);
		
		this._location.set(_geodeticLocation.getGeocentric());
		this.setLocalOriginCoordinates();		

		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}
	
	public void locateLongitude(Angle lon) {
		this._geodeticLocation.setEastLongitude(lon);
		
		this._location.set(_geodeticLocation.getGeocentric());
		this.setLocalOriginCoordinates();		

		this._localFrame.set(_geodeticLocation);	
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}
		
	//lossy mutator...
	
	public void locateEllipsoidHeight(double meters) {
		this._geodeticLocation.setEllipsoidHeight(meters);
		
		this._location.set(_geodeticLocation.getGeocentric());	
		this.setLocalOriginCoordinates();		
	
		this._localFrame.set(_geodeticLocation); 
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}


	public void locate(Vector3 geocentric){
		this._location.set(geocentric);
		this.setLocalOriginCoordinates();		

		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}
		
	public void locateX(double E) {
		this._location.setX(E);
		this.setLocalOriginCoordinates();		

		// this._wgs84.set(_geocentric); //with moving pedestal -- could avoid need for Ellipsoid calculations below...
		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}

	public void locateY(double F) {
		this._location.setY(F);
		this.setLocalOriginCoordinates();		

		// this._wgs84.set(_geocentric); //with moving pedestal -- could avoid need for Ellipsoid calculations below...
		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}

	public void locateZ(double G) {
		this._location.setZ(G);
		this.setLocalOriginCoordinates();		

		// this._wgs84.set(_geocentric);//with moving pedestal -- could avoid need for Ellipsoid calculations below...
		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}	
	
	


	// After location is set... deal with pedestal rotator and range positioning updates...
	
	public void point(Polar position) {
		this._local.set(position);
		this._apertureFrame.set(position, this._localFrame._local);
		this._vector.set(_apertureFrame.getVector());
	}

	public void pointRange( double meters ) {
		this._local.setRange(meters);		
		this._apertureFrame.setRange(meters);
		double rescale = meters/_vector.getAbs();
		this._vector.multiply(rescale);
//		this._vector.set(_apertureFrame.getVector());
	}
	
	// leaves vector range unchanged...
	public void pointDirection(Angle azimuth, Angle elevation) {
		this._local.setAzimuth(azimuth);
		this._local.setElevation( elevation);	
		//do nothing about _vectorPolar._range
		this._apertureFrame.set(azimuth, elevation, this._localFrame._local);
		//do nothing about _apertureFrame._range
		this._vector.set(_apertureFrame.getVector());
	}
	
	public void point(double range, Angle azimuth, Angle elevation) {
		this._local.set(range, azimuth, elevation);
		this._apertureFrame.set(range, azimuth, elevation, this._localFrame._local);
		this._vector.set(_apertureFrame.getVector());
	}
	
	public void pointAzimuth(Angle azimuth) {
		this._local.setAzimuth(azimuth);
		this._apertureFrame.set(azimuth, _local._elevation, this._localFrame._local);
		this._vector.set(_apertureFrame.getVector());
	}
	
	public void pointElevation(Angle elevation) {
		this._local.setElevation(elevation);
		this._apertureFrame.set(_local._azimuth, elevation, this._localFrame._local);
		this._vector.set(_apertureFrame.getVector());
	}
	
	/** Updates pedestal's vector to point to WGS84 Cartesian location.
	 * @param geocentricEFG Position of target.  */
	public void pointToLocation(Vector3 geocentricEFG) {
		//Define geocentric vector from ped to target -- obtain pedestal.aperture range to target.
		Vector3 r_PT_G = new Vector3(geocentricEFG).subtract(_location);
		pointToLocationOffset(r_PT_G); //updates RAE _position and Rotator _positionGeodetic
	}
		
	
	/** Updates pedestal's vector to point to WGS84 Cartesian location 
	 *  defined as Cartesian coordinate offset from pedestal's location
	 * @param locationOffsetEFG Position of target.  */
	public void pointToLocationOffset(Vector3 locationOffsetEFG){		
		this._local.set( Polar.commandLocal(locationOffsetEFG,this._localFrame._local));
		this._apertureFrame.set(this._local, this._localFrame._local);		
		this._apertureFrame._range = _local.getRange();		
	}
	
	public String toString() { 
		return this._systemId 
				+ "("+ this._geodeticLocation.getNorthLatitude().toDegreesString(DIGITS) +", "+ this._geodeticLocation.getEastLongitude().toDegreesString(DIGITS)+", "+this.getLocationEllipsoid().getEllipsoidHeight()+")"
				+"("+_local.getUnsignedAzimuth().toDegreesString(DIGITS)+", "+_local.getElevation().toDegreesString(DIGITS)+")";
	}

}
