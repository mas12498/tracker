package tspi.filter;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import tspi.rotation.Vector3;
import tspi.util.TVector;

/** Makes a circuit consisting of two straight aways, and two turns all in a plane. */
public class Racetrack implements Trajectory {

	double startTime, radius, velocity, perimeter, straight, turn;
	Vector3 c1, c2, r1, r2;

	public Racetrack(double startTime, Vector3 c1, Vector3 c2, double radius, double velocity) {
		super();
		this.startTime = startTime;
		this.c1 = c1;
		this.c2 = c2;
		this.radius = radius;
		this.velocity = velocity;

		// construct some geometry
		Vector3 up = new Vector3(0.0,0.0,1.0);
		Vector3 d = new Vector3(c2).subtract(c1);
		r1 = new Vector3(d).multiply(-1).crossProduct(up).unit().multiply(radius);
		r2 = new Vector3(d).unit().multiply(radius);
		straight = d.getAbs();
		turn = Math.PI * radius;
		perimeter = 2 * turn + 2 * straight;
	}

	public double getPerimeter() {return perimeter;}
	public double getRadius() {return radius;}
	public Vector3 getCenter1() {return c1;}
	public Vector3 getCenter2() {return c2;}

	/** The target moves along a racetrack constructed around two points at a constant velocity */
	public RealVector getPosition(double currentTime ) {
		return (new TVector( getPositionVector(currentTime) )).arrayRealVector();
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
		return new ArrayRealVector(new TVector(getVelocityVector(currentTime)).arrayRealVector());
	}
	public Vector3 getVelocityVector(double currentTime) {
		// figure out how far along the track we are
		double distance = ((currentTime - startTime) * velocity) % perimeter;

		// linearly interpolate if on first straightaway
		if (distance < straight)
			return new Vector3(r2).unit().multiply(velocity);
		else distance -= straight;

		// rotate along first turn if it's there
		if (distance < turn) {
			double angle = Math.PI * distance / turn;
			Vector3 v = new Vector3(r1).multiply( -Math.sin(angle) )
					.add( new Vector3(r2).multiply( Math.cos(angle) ) );
			return v.unit().multiply(velocity);
		} else distance -= turn;

		// linearly interpolate if on second straightaway
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
		return new ArrayRealVector();
	}
	
	@Override
	public RealVector getState(double currentTime) {
		
		RealVector p = getPosition(currentTime);
		RealVector v = getVelocity(currentTime);
		RealVector a = getAcceleration(currentTime);

		return p.append(v.append(a));
	}
}