package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ellipsoid;
import tspi.model.T_EFG_NED;
import tspi.rotation.QuaternionMath;
import tspi.rotation.Rotator;
import tspi.rotation.Vector3;
import tspi.util.TVector;

/** Makes a circuit consisting of two straight aways, and two turns all in a plane. */
public class Racetrack implements Trajectory {

	double radius, velocity, startTime, perimeter, straight, turn;
	Vector3 c1, c2, r1, r2;
	Ellipsoid origin;
	Rotator rotateLocal;
	Vector3 racetrackEFG;

	public Racetrack(double startTime, Ellipsoid origin, Vector3 c1, Vector3 c2, double radius, double velocity) {
		super();
		this.origin = origin;
		this.c1 = c1;
		this.c2 = c2;
		this.radius = radius;
		this.velocity = velocity;
		this.startTime = startTime;

		// construct some geometry
		Vector3 up = new Vector3(0.0,0.0,1.0);
		Vector3 d = new Vector3(c2).subtract(c1);
		r1 = new Vector3(up).crossProduct(d).unit().multiply(radius);
		r2 = new Vector3(d).unit().multiply(radius);
		straight = d.getAbs();
		turn = Math.PI * radius;
		perimeter = 2 * turn + 2 * straight;

		// construct coordinate rotation
		T_EFG_NED local = new T_EFG_NED();
		local.set(origin);
		rotateLocal = new Rotator( local.getLocal().unit() );
		racetrackEFG = origin.getGeocentric();
	}

	public double getPerimeter() {return perimeter;}
	public double getRadius() {return radius;}
	public double getVelocity() {return velocity;}
	public Vector3 getCenter1() {return c1;}
	public Vector3 getCenter2() {return c2;}

	/** rotate the racetrack coordinates to an orientation tangent to a point on the surface of the ellipsoid. */
	Vector3 rotate(Vector3 p) {
		Vector3 s = new Vector3(p.getX(), p.getY(), -p.getZ());
		return QuaternionMath.multiply( rotateLocal,
				QuaternionMath.multiply( s,
						QuaternionMath.conjugate(rotateLocal))).getV();
	}
	/** rotate and translate the racetrack coordinates to an orientation tangent to a point on the surface of the ellipsoid. */
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