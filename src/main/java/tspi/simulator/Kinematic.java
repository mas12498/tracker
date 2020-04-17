package tspi.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

/** Parametric model of 2nd order kinematic motion. */
public class Kinematic implements Trajectory {

	double t0;
	ArrayRealVector a0;
	ArrayRealVector v0;
	ArrayRealVector p0;
	
	public Kinematic( double t0,
			double p00, double p01, double p02,
			double v00, double v01, double v02,
			double a00, double a01, double a02 ) {
		this.t0 = t0;
		double p[] = {p00, p01, p02};
		this.p0 = new ArrayRealVector(p);
		double v[] = {v00, v01, v02};
		this.v0 = new ArrayRealVector(v);
		double a[] = {a00, a01, a02};
		this.a0 = new ArrayRealVector(a);
	}
	
	public Kinematic(double t0, ArrayRealVector p0, ArrayRealVector v0, ArrayRealVector a0) {
		super();
		this.t0 = t0;
		this.p0 = p0;
		this.v0 = v0;
		this.a0 = a0;
	}
	
	//integrate kinematic trajectory assumming constant acceleration and initial state {p;v;a} at t0:

	/** p(t) = p0 + v0*t + 0.5*a0*t^2 */
	public RealVector getPosition(double currentTime ) {
		double elapsedTime = currentTime - t0;
		ArrayRealVector position = p0.add( v0.mapMultiply(elapsedTime) ).add( a0.mapMultiply(elapsedTime*elapsedTime*0.5) );
		return position;
	}
	
	/** v(t) = v0 + a0*t */
	public RealVector getVelocity(double currentTime ) {
		double elapsedTime = currentTime - t0;
		ArrayRealVector velocity = v0.add( a0.mapMultiply(elapsedTime) );
		return velocity;
	}
	
	/** a(t) = a0 */
	public RealVector getAcceleration(double currentTime ) {
		ArrayRealVector acceleration = new ArrayRealVector(a0);
		return acceleration;
	}
	
	@Override
	public RealVector getState(double currentTime) {
		
		RealVector p = getPosition(currentTime);
		RealVector v = getVelocity(currentTime);
		RealVector a = getAcceleration(currentTime);

		return p.append(v.append(a));
	} // TODO improve efficiency 
}