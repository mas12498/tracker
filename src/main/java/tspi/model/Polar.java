package tspi.model;

import tspi.rotation.Angle;
import tspi.rotation.Rotator;
import tspi.rotation.Vector3;

public class Polar {
	protected Double _range;
	protected final Angle _azimuth;
	protected final Angle _elevation;

	public Polar() {
		_range = Double.NaN;
		_azimuth = new Angle();
		_elevation = new Angle();
	}
	
	public  Polar(double range, Angle azimuth, Angle elevation) {
		_range = range;
		_azimuth = azimuth;
		_elevation = elevation;
	}

	public Polar(Polar template) {
		_azimuth = new Angle(template._azimuth);
		_elevation = new Angle(template._elevation);
		_range = template._range;
	}

	/**
	 * Azimuth Angle factory.
	 * @return the unsigned principle azimuth Angle
	 */
	public Angle getUnsignedAzimuth() {
		return _azimuth.unsignedPrinciple();
	}

	/**
	 * Azimuth signed Angle factory.
	 * @return the signed principle azimuth Angle
	 */
	public Angle getSignedAzimuth() {
		return _azimuth.signedPrinciple();
	}

	
	/**
	 * @return the coded signed principle elevation
	 */
	public Angle getElevation() {
		return _elevation.signedPrinciple();
	}

	/**
	 * @return the range
	 */
	public double getRange() {
		return _range;
	}

	public Vector3 getTopocentric() { // NED of navigation
		double RcosE = _range * StrictMath.cos(_elevation.getRadians());
		return new Vector3(
				RcosE * StrictMath.cos(_azimuth.getRadians())
				, RcosE * StrictMath.sin(_azimuth.getRadians()),
				-_range * StrictMath.sin(_elevation.getRadians())
				);
	}

	public void setTopocentric(Vector3 r_NED) { // NED of navigation
		
		_range = r_NED.getAbs(); //always positive in this construction!!!
		
		_azimuth.set( Angle.inRadians(StrictMath.atan2(r_NED.getY(), r_NED.getX())));
		
		double rg = StrictMath.hypot(r_NED.getX(), r_NED.getY());
		
		_elevation.set(Angle.inRadians(StrictMath.atan2(-r_NED.getZ(), rg)));
		
	}

	public void set(Polar position) {
		_range = position._range;
		_azimuth.set(position._azimuth);
		_elevation.set(position._elevation);
	}

	public void set(double range, Angle azimuth, Angle elevation) {
		_range = range;
		_azimuth.set(azimuth);
		_elevation.set(elevation);
	}

	/**
	 * @param azimuth
	 *            the azimuth to set
	 */
	public void setAzimuth(Angle azimuth) {
		this._azimuth.set(azimuth);
	}

	/**
	 * @param elevation
	 *            the elevation to set
	 */
	public void setElevation(Angle elevation) {
		this._elevation.set(elevation);
	}

	/**
	 * @param range
	 *            the range to set
	 */
	public void setRange(Double range) {
		_range = range;
	}

	public void clear(double missingValue) {
		_azimuth.set(Angle.EMPTY);
		_elevation.set(Angle.EMPTY);
		_range = missingValue;
	}
	public void clear() {
		_azimuth.set(Angle.EMPTY);
		_elevation.set(Angle.EMPTY);
		_range = Double.NaN;
	}


	protected  static Polar commandLocal(Vector3 r_EFG, Rotator pedestalLocal){
		Rotator los = (Rotator) new Rotator(pedestalLocal).preTilt_i(r_EFG).conjugate();
		return new Polar(r_EFG.getAbs(), los.getEuler_k_kji().angle().signedPrinciple(), los.getEuler_j_kji().angle().signedPrinciple());
	}

//	public Rotator positionLocal() {
//		// @MAS: Maybe this method should be removed from inside this class...
//		return QuaternionMath.eulerRotate_kj(_azimuth.getPrinciple(), _elevation.getPrinciple());
//	}

}
