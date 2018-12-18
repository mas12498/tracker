package tspi.filter;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

/** Parametric model of 2nd order kinematic motion. */
public class Kinematic implements TrackSimulator.Trajectory {

	public ArrayRealVector a;
	public ArrayRealVector v;
	public ArrayRealVector p;
		
	public Kinematic(ArrayRealVector a, ArrayRealVector v, ArrayRealVector p) {
		super();
		this.a = a;
		this.v = v;
		this.p = p;
	}

	@Override
	public RealVector getState(double time) {
		return p.add( v.mapMultiply(time) ).add( a.mapMultiply(time*time*0.5) );
	} 
}

//public class Kinematic implements Model {
//
//	public Vector3 a;
//	public Vector3 v;
//	public Vector3 p;
//		
//	public Kinematic(Vector3 a, Vector3 v, Vector3 p) {
//		super();
//		this.a = a;
//		this.v = v;
//		this.p = p;
//	}
//
//	@Override
//	public RealVector getState(double time) {
//		//return p+ t*v + t*0.5*a^2
//		Vector3 V = new Vector3(v).multiply(time);
//		Vector3 A = new Vector3(a).multiply(time*time*0.5);
//		Vector3 P = new Vector3(p).add(V).add(A);
//
//		return P;
//	}
//
//}