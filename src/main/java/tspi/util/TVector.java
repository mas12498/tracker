/**
 * 
 */
package tspi.util;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import tspi.rotation.Vector3;

/**
 * TVector extends package rotation.Vector3 
 * 
 * <p> Adaptor: 
 * <br>-- to/from Apache.math3 'RealVector'
 * <br>-- to/from 'double[]'
 * @author mshields
 *
 */
public class TVector extends Vector3 {

	//Class Constructors:
	
	public TVector(double vi, double vj, double vk) {
		super(vi, vj, vk);
	}
	public TVector(Vector3 v) {
		super(v);
	}
	
	//Adapted constructors:
	
	//double array
	public TVector(double[] v) {
		super(v[0],v[1],v[2]);
	}
	
	//double v(n:n+2) sub-array
	public TVector(double[] v,int n) {
		super(v[n],v[n+1],v[n+2]);
	}
	
	//Apache: RealVector 
	public TVector(ArrayRealVector v) {
		super(v.getEntry(0),v.getEntry(1),v.getEntry(2));
	}
	
	//Apache: ArrayRealVector(n:n+2) sub-array
	public TVector(ArrayRealVector v, int n) {
		super(v.getEntry(n),v.getEntry(n+1),v.getEntry(n+2));
	}
	
	//gsl: RealVector 
	public TVector(RealVector v) {
		super(v.getEntry(0),v.getEntry(1),v.getEntry(2));
	}
	
	//gsl: RealVector(n:n+2) sub-array
	public TVector(RealVector v, int n) {
		super(v.getEntry(n),v.getEntry(n+1),v.getEntry(n+2));
	}
	
	
	
	
	//Factories:
	
	//Apache: ArrayRealVector
	public ArrayRealVector arrayRealVector() {
		return new ArrayRealVector(this.doubleArray());
	}

	//gsl: RealVector
	public RealVector realVector() {
		return MatrixUtils.createRealVector(this.doubleArray());
	}

	
	//double[]
	public double[] doubleArray() {
		return new double[]  { this.getX(),this.getY(),this.getZ() };
	}

	
    //Writes:
	
	//Apache: ArrayRealVector(n:n+2) sub-array
	public ArrayRealVector toArray(ArrayRealVector r, int n) {
		r.setEntry(n, this.getX());
		r.setEntry(n+1, this.getY());
		r.setEntry(n+2, this.getZ());
		return r;
	}	
	
	//gsl: RealVector(n:n+2) sub-array
	public RealVector toArray(RealVector r, int n) {
		r.setEntry(n, this.getX());
		r.setEntry(n+1, this.getY());
		r.setEntry(n+2, this.getZ());
		return r;
	}
	
	//double(n:n+2) sub-array
	public double[] toArray(double v[],int n) {
		v[n] = this.getX();
		v[n+1] = this.getY();
		v[n+2] = this.getZ();
		return v;
	}
	
	//More Setters:

	//ArrayRealVector(n:n+2) sub-array
	public void set(ArrayRealVector r, int n) {
		this.setX(r.getEntry(n));
		this.setY(r.getEntry(n+1));
		this.setZ(r.getEntry(n+2));
	}	
	
		
	//RealVector(n:n+2) sub-array
	public void set(RealVector r, int n) {
		this.setX(r.getEntry(n));
		this.setY(r.getEntry(n+1));
		this.setZ(r.getEntry(n+2));
	}
	
	//double(n:n+2) sub-array
	public void set(double v[],int n) {
		this.setX(v[n]);
		this.setY(v[n+1]);
		this.setZ(v[n+2]);
	}
	
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		TVector v = new TVector(1.0,2.0,3.0);
		double[] d = v.doubleArray();
		RealVector r = v.realVector();
		TVector u = new TVector(r,0);
		r = r.add(r);
		v.set(r,0);
		TVector w = new TVector(d);
		w.multiply(10);
		System.out.println(v.toTupleString());
		System.out.println(w.toTupleString());
		u.set(d,0);
		System.out.println(u.toTupleString());

	}

}
