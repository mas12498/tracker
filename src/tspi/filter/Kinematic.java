package tspi.filter;

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

	/** p(t) = p0 + v*t + 0.5*a*t^2 */
	public RealVector getPosition( double time ) {
		double dt = time - t0;
		ArrayRealVector p1 = p0.add( v0.mapMultiply(dt) ).add( a0.mapMultiply(dt*dt*0.5) );
		return p1;
	}
	
	/** Compute velocity linearly from acceleration */
	public RealVector getVelocity( double time ) {
		double dt = time - t0;
		ArrayRealVector v1 = v0.add( a0.mapMultiply(dt) );
		return v1;
	}
	
	/** Acceleration is constant */
	public RealVector getAcceleration( double time ) {
		ArrayRealVector a1 = new ArrayRealVector(a0);
		return a1;
	}
	
	@Override
	public RealVector getState(double time) {
		
		RealVector p = getPosition(time);
		RealVector v = getVelocity(time);
		RealVector a = getAcceleration(time);

		return p.append(v.append(a));
	} // TODO improve efficiency 
}