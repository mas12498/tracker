package tspi.model;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;

public class Target {
	double time;
	
	/** TARGET GEODETIC LOCATION WGS 84 ELLIPSOID [LLh] **/
	Ellipsoid _geodeticLocation;
	
	/** TARGET TRANSFORMATION LOCAL NAVIGATION defined from geocentric XYZ. **/
	T_EFG_NED _localNavFrame;
	
	/** TARGET LOCATION WGS 84 GEOCENTRIC [XYZ] **/
	Vector3 _location;
	
	Solution solution;
	
	public Target(double time, double lat, double lon, double h) {
		this.time = time;
		this._localNavFrame = new T_EFG_NED();
		this._geodeticLocation = new Ellipsoid(Angle.inDegrees(lat), Angle.inDegrees(lon), h);
		this._localNavFrame.set(_geodeticLocation);
		
		this._location = _localNavFrame.getGeocentric();
		this.solution = null;
	}

	public T_EFG_NED getEllipsoidalCoordinates() { return this._localNavFrame; }
	public Vector3 getGeocentricCoordinates() { return this._location; }
	public Double getTime() { return this.time; }
	public Angle getLatitude() { return _geodeticLocation.getNorthLatitude(); }
	public Angle getLongitude() { return _geodeticLocation.getEastLongitude().signedPrinciple(); }
	public double getHeight() { return this._geodeticLocation.getEllipsoidHeight(); }
	public double getE() { return this._location.getX(); }
	public double getF() { return this._location.getY(); }
	public double getG() { return this._location.getZ(); }
	
	private static final int DIGITS = 14;
	
	public Double getError() {
		if(solution==null) return null;
		else return solution._error;
	}
	public Double getConditionNumber() {
		if(solution==null) return null;
		else return solution._condition;
	}

	public void setTime(long time) { this.time = time; }

	public void setLatitude(double lat) {
		if(_localNavFrame.equals(null)){
			_localNavFrame = new T_EFG_NED();
		}
		this._geodeticLocation.setNorthLatitude(Angle.inDegrees(lat));
		_localNavFrame.set(_geodeticLocation);
		this._location = _localNavFrame.getGeocentric();
	}

	public void setLongitude(double lon) {
		if(_localNavFrame.equals(null)){
			_localNavFrame = new T_EFG_NED();
		}
		this._geodeticLocation.setEastLongitude(Angle.inDegrees(lon).signedPrinciple());
		this._localNavFrame.set(_geodeticLocation);
		this._location = _localNavFrame.getGeocentric();
	}

	public void setHeight(double h) {
		if(_localNavFrame.equals(null)){
			_localNavFrame = new T_EFG_NED();
		}
		this._geodeticLocation.setEllipsoidHeight(h);
		this._localNavFrame.set(_geodeticLocation); 
		this._location = _localNavFrame.getGeocentric();
	}

	public void setE(double E) {
		this._location.setX(E);
		if(_localNavFrame.equals(null)){
			_localNavFrame = new T_EFG_NED();
		}
		this._localNavFrame.set( _location );
	}

	public void setF(double F) {
		this._location.setY(F);
		if(_localNavFrame.equals(null)){
			_localNavFrame = new T_EFG_NED();
		}
		this._localNavFrame.set( _location );
	}

	public void setG(double G) {
		this._location.setZ(G);
		if(_localNavFrame.equals(null)){
			_localNavFrame = new T_EFG_NED();
		}
		this._localNavFrame.set( _location );
	}

	public void setSolution(Solution solution) {
		this.solution = solution;
	}

	public String toString() { 
		return this.time 
				+ "(" + this.getLatitude().toDegreesString(DIGITS) + ", " + this.getLongitude().signedPrinciple().toDegreesString(DIGITS)
				+ ", " + this.getHeight() + ")";
	}
}


