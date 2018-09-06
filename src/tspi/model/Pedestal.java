package tspi.model;

import java.util.Random;

import rotation.Angle;
import rotation.Rotator;
import rotation.Vector3;

/**  
 * Every thing native in EFG  WGS84 Cartesian frame
 * 
 * */
public class Pedestal {
	
	String _systemId; // system identifier
	
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

	/** PEDESTAL VECTOR WGS 84 GEOCENTRIC ORIENTED [XYZ] **/
	final Vector3 _vector = new Vector3(Vector3.EMPTY); //local vector offset: EFG

	/** LOCAL SITE 'NORTH' DIRECTION in geocentric XYZ frame. **/
	final Vector3 _unitNorth = new Vector3(Vector3.EMPTY); 
	
	/** LOCAL SITE 'EAST' DIRECTION in geocentric XYZ frame. **/
	final Vector3 _unitEast = new Vector3(Vector3.EMPTY); 
	
	/** LOCAL SITE 'UP' DIRECTION in geocentric XYZ frame. **/
	final Vector3 _unitUp = new Vector3(Vector3.EMPTY); 
	
	private static final int DIGITS = 14;
	
	//Constructor
	public Pedestal( String id, Angle lat, Angle lon, double h) {
		//_wgs84: from Pedestal's geodetic-ellipsoid coordinates Latitude, Longitude, height
		this._systemId = id;	
		this._geodeticLocation.set(lat,lon, h); //Ellipsoid
		this._localFrame.set(this._geodeticLocation);	//T_EFG_NED
		this._unitNorth.set(this._localFrame._local.getImage_i());
		this._unitEast.set(this._localFrame._local.getImage_j());
		this._unitUp.set(this._localFrame._local.getImage_k().negate());
		this._location.set(this._geodeticLocation.getGeocentric()); //Cartesian
		this.clearPedestalVector();
	}
	
	
	//Clone Pedestal objects...
	
	public String getSystemId() { return this._systemId; }
	
	public Polar getBias() { return _bias; }

	public Polar getDeviation() { return _deviation; }
	
	public Polar getPerturbedLocal(Random random) {
		// add a normally distributed error to each of the polar coordinates 
		double range =
				_local.getRange() + _bias.getRange()
				+ random.nextGaussian() * _deviation.getRange();
		
		double azimuth = 
				_local.getAzimuth().getPiRadians() + _bias.getAzimuth().getPiRadians()
				+ random.nextGaussian() * _deviation.getAzimuth().getPiRadians();
		// TODO There is another correcting term that has to be added as elevation increases!!!
		
		double elevation = 
				_local.getElevation().getPiRadians() + _bias.getElevation().getPiRadians()
				+ random.nextGaussian() * _deviation.getElevation().getPiRadians();
		
		return new Polar (range, Angle.inPiRadians(azimuth), Angle.inPiRadians(elevation));
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
	 * @return pedestal's location in geocentric oriented coordinate frame.
	 */
	public Vector3 getLocation() { return new Vector3(this._location); }	 
	
	/**
	 * @return pedestal's location in WGS 84's ellipsoid coordinates.
	 */
	public Ellipsoid getLocationEllipsoid() {return new Ellipsoid(this._geodeticLocation); }

	/**
	 * @return model transform from geocentric frame coordinates to pedestal's local topocentric frame coordinates.
	 */
	public T_EFG_NED getLocationFrame() { return new T_EFG_NED(this._localFrame); }
	

	//Mutators
	
	public void clearPedestalLocation() {
		_geodeticLocation.set(Angle.EMPTY, Angle.EMPTY, Double.NaN);
		_location.set(Vector3.EMPTY);
		_localFrame.clear();
		clearPedestalVector();
	};
	
	public void clearPedestalVector(){
		_local.set(Double.NaN, Angle.EMPTY, Angle.EMPTY);
		_apertureFrame.clear();
	};
	
	public void setSystemId(String id) { this._systemId = id; }
	
	public void setBias(Polar bias) { this._bias.set(bias); }
	
	public void setDeviation(Polar deviation) { this._deviation.set(deviation); }
	
	
	public void locate(T_EFG_NED locationFrame){	
		this._localFrame.set(locationFrame);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 		
		this._geodeticLocation.set(_localFrame.getEllipsoid());		
		
		this._location.set(_geodeticLocation.getGeocentric());
	}
	
	public void locateHorizontal(Rotator horizontal){		
		this._localFrame.setLocal(horizontal);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
		this._geodeticLocation.set(_localFrame.getEllipsoid());
		
		this._location.set(_geodeticLocation.getGeocentric());
	}
	
	public void locateVertical(double vertical){		
		this._localFrame.setLocalHeight(vertical);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 		
		this._geodeticLocation.set(_localFrame.getEllipsoid());
		
		this._location.set(_geodeticLocation.getGeocentric());
	}
	
	
	
	public void locate(Ellipsoid wgs84) {
		this._geodeticLocation.set(wgs84);
		
		this._location.set(_geodeticLocation.getGeocentric());
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 	
	}
	
	public void locateLatitude(Angle lat) {
		this._geodeticLocation.setNorthLatitude(lat);
		
		this._location.set(_geodeticLocation.getGeocentric());
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}
	
	public void locateLongitude(Angle lon) {
		this._geodeticLocation.setEastLongitude(lon);
		
		this._location.set(_geodeticLocation.getGeocentric());
		this._localFrame.set(_geodeticLocation);	
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}
		
	public void locateEllipsoidHeight(double meters) {
		this._geodeticLocation.setEllipsoidHeight(meters);
		
		this._location.set(_geodeticLocation.getGeocentric());				
		this._localFrame.set(_geodeticLocation); 
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}

	//lossy mutator...
	
	public void locate(Vector3 geocentric){
		this._location.set(geocentric);
		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}
		
	public void locateX(double E) {
		this._location.setX(E);
		// this._wgs84.set(_geocentric); //with moving pedestal -- could avoid need for Ellipsoid calculations below...
		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}

	public void locateY(double F) {
		this._location.setY(F);
		// this._wgs84.set(_geocentric); //with moving pedestal -- could avoid need for Ellipsoid calculations below...
		
		this._geodeticLocation.setGeocentric(_location);
		
		this._localFrame.set(_geodeticLocation);
		
		this._unitNorth.set(this._localFrame._local.getImage_i()); 
		this._unitEast.set(this._localFrame._local.getImage_j()); 
		this._unitUp.set(this._localFrame._local.getImage_k().negate()); 
	}

	public void locateZ(double G) {
		this._location.setZ(G);
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
				+"("+_local.getAzimuth().toDegreesString(DIGITS)+", "+_local.getElevation().toDegreesString(DIGITS)+")";
	}

}
