package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ellipsoid;
import tspi.model.T_EFG_NED;
import tspi.rotation.QuaternionMath;
import tspi.rotation.Rotator;
import tspi.rotation.Vector3;
import tspi.util.TVector;

import static java.lang.StrictMath.hypot;

/** Makes a racetrack circuit consisting of two straight segments, and two turn segments all in a plane
 * defined w.r.t to local topocentric origin. */
public class Racetrack implements Trajectory {

	double radius, velocity, startTime, perimeter, straight, turn;
	Vector3 c1, c2, r1, r2;
	Ellipsoid origin;
	Rotator rotateLocal;
	Vector3 racetrackEFG;

	/**
	 * racetrack constructor:
	 * @param startTime	start time seconds for initializing race track plots
	 * @param origin	Locate topocentric local origin ellipsoid coordinates
	 * @param c1		racetrack turn 1 center topocentric coordinates (3-D)
	 * @param c2		racetrack turn 2 center topocentric coordinates (3-D)
	 * @param radius 	racetrack turn radius
	 * @param velocity racetrack plots target velocity (positive is CCW?: negative is CW?)
	 */
	public Racetrack(double startTime, Ellipsoid origin, Vector3 c1, Vector3 c2, double radius, double velocity) {
		super();
		this.origin = origin;
		this.c1 = c1;
		this.c2 = c2;
		this.radius = radius;
		this.velocity = velocity;
		this.startTime = startTime;

		// construct some geometry
		Vector3 d = new Vector3(c2).subtract(c1);
		straight = d.getAbs();

		//Vector3 up = new Vector3(0.0,0.0,1.0);
		//r1 = new Vector3(up).crossProduct(d).unit().multiply(radius);
		//r2 = new Vector3(d).unit().multiply(radius);

		double h = radius/hypot(d.getX(),d.getY());
		r1 = new Vector3(-d.getY()*h, d.getX()*h,0); //sparse (k X d)
		r2 = new Vector3(d).multiply(radius/straight);

		turn = Math.PI * radius;
		perimeter = 2 * turn + 2 * straight;

		// construct coordinate rotation
		T_EFG_NED local = new T_EFG_NED();
		local.set(origin); //Locates racetrack topocentric coordinates origin.
		rotateLocal = new Rotator( local.getLocal().unit() ); //unit quaternion to racetrack alignment.
		racetrackEFG = origin.getGeocentric();
	}

	public double getPerimeter() {return perimeter;}
	public double getRadius() {return radius;}
	public double getVelocity() {return velocity;}
	public Vector3 getCenter1() {return c1;}
	public Vector3 getCenter2() {return c2;}

	/** rotate the racetrack plot coordinates from local topocentric orientation
	 * to Geocentric orientation. */
	Vector3 rotate(Vector3 p) {
		Vector3 s = new Vector3(p.getX(), p.getY(), -p.getZ());
		return QuaternionMath.multiply( rotateLocal,
				QuaternionMath.multiply( s,
						QuaternionMath.conjugate(rotateLocal))).getV();
	}

	/** rotate and translate the racetrack plot coordinates from local topocentric
	 * to Geocentric plots (from center of earth). */
	Vector3 rotateAndTranslate(Vector3 p) {
		Vector3 s = new Vector3(p.getX(), p.getY(), -p.getZ());
		return QuaternionMath.multiply( rotateLocal,
				QuaternionMath.multiply( s,
						QuaternionMath.conjugate(rotateLocal))).getV()
				.add(racetrackEFG);
	}

	/** The target moves along a racetrack constructed around two points at a constant velocity */
	public RealVector getPosition(double currentTime ) {
		Vector3 xyz = new Vector3( getPositionVector(currentTime) );
		Vector3 efg = new Vector3( rotateAndTranslate( xyz ) );
		return (new TVector(efg)).arrayRealVector();
	}
	Vector3 getPositionVector(double currentTime) {
		// figure out how far along the track we are
		double distance = ((currentTime - startTime) * velocity) % perimeter;

		// linearly interpolate if on first straightaway
		if (distance < straight) {
			double k = distance/straight;
			Vector3 p = new Vector3(c1).multiply(1.0-k)
					.add( new Vector3(c2).multiply(k) );
			return p.add(r1);
		} else distance -= straight;

		// rotate along first turn if it's there
		if (distance < turn) {
			double angle = Math.PI * distance / turn;
			Vector3 p = new Vector3(r1).multiply( Math.cos(angle) )
					.add( new Vector3(r2).multiply( Math.sin(angle) ) );
			return new Vector3(c2).add( p ); // TODO I could just use Vector3.slerp()...
		} else distance -= turn;

		// linearly interpolate if on second straightaway
		if (distance < straight) {
			double k = distance/straight;
			Vector3 p = new Vector3(c1).multiply(k)
					.add( new Vector3(c2).multiply(1.0-k) );
			return p.subtract(r1);
		} else distance -= straight;

		// otherwise rotate along last turn
		double angle = Math.PI * distance / turn;
		Vector3 p = new Vector3(r1).multiply( -Math.cos(angle) )
				.add( new Vector3(r2).multiply( -Math.sin(angle) ) );
		return new Vector3(c1).add( p );
	}
	
	/** v(t) = v0 + a0*t */
	public RealVector getVelocity(double currentTime ) {
		Vector3 xyz = getVelocityVector(currentTime);
		Vector3 efg = rotate(xyz);
		return (new TVector(efg)).arrayRealVector();
	}
	public Vector3 getVelocityVector(double currentTime) {
		// figure out how far along the track we are
		double distance = ((currentTime - startTime) * velocity) % perimeter;

		// velocity constant along first straight-away
		if (distance < straight)
			return new Vector3(r2).unit().multiply(velocity);
		else distance -= straight;

		// rotate along first turn
		if (distance < turn) {
			double angle = Math.PI * distance / turn;
			Vector3 v = new Vector3(r1).multiply( -Math.sin(angle) )
					.add( new Vector3(r2).multiply( Math.cos(angle) ) );
			return v.unit().multiply(velocity);
		} else distance -= turn;

		// velocity constant along second straight-away
		if (distance < straight)
			return new Vector3(r2).unit().multiply(-velocity);
		else distance -= straight;

		// otherwise rotate along last turn
		double angle = Math.PI * distance / turn;
		Vector3 v = new Vector3(r1).multiply( Math.sin(angle) )
				.add( new Vector3(r2).multiply( -Math.cos(angle) ) );
		return v.unit().multiply(velocity);
	}
	
	/** a(t) = a0 */
	public RealVector getAcceleration(double currentTime )  {
		Vector3 xyz = getAccelerationVector(currentTime);
		Vector3 efg = rotate(xyz);
		return (new TVector(efg)).arrayRealVector();
	}
	public Vector3 getAccelerationVector(double currentTime) {
		// figure out how far along the track we are
		double distance = ((currentTime - startTime) * velocity) % perimeter;

		// straight aways have zero acceleration
		if (distance < straight) {
			return new Vector3(0,0,0);
		} else distance -= straight;

		// the target accelerates centripetally during uniform circular motion
		if (distance < turn) {
			double angle = Math.PI * distance / turn;
			Vector3 v = new Vector3(r1).multiply( -Math.cos(angle) )
					.add( new Vector3(r2).multiply( -Math.sin(angle) ) );
			return v.unit().multiply(velocity*velocity/radius);
		} else distance -= turn;

		// straight aways have zero acceleration
		if (distance < straight) {
			return new Vector3(0,0,0);
		} else distance -= straight;

		// the target accelerates centripetally during uniform circular motion
		double angle = Math.PI * distance / turn;
		Vector3 v = new Vector3(r1).multiply( Math.cos(angle) )
				.add( new Vector3(r2).multiply( Math.sin(angle) ) );
		return v.unit().multiply(velocity*velocity/radius);
	}
	
	@Override
	public RealVector getState(double currentTime) {
		
		RealVector p = getPosition(currentTime);
		RealVector v = getVelocity(currentTime);
		RealVector a = getAcceleration(currentTime);

		return p.append(v.append(a));
	}
}