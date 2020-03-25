package tspi.rotation;

/**
 * @author mike
 * @version 0.0.2
 * -- argument of zero quaternion is defined null.
 */
public class Quaternion {

	public static final Quaternion EMPTY = new Quaternion(Double.NaN, Double.NaN,
			Double.NaN, Double.NaN);
	public static final Quaternion IDENTITY = new Quaternion(1d, 0d, 0d, 0d);
	public static final Quaternion UNIT_I = new Quaternion(0d, 1d, 0d, 0d);
	public static final Quaternion UNIT_J = new Quaternion(0d, 0d, 1d, 0d);
	
	public static final Quaternion UNIT_K = new Quaternion(0d, 0d, 0d, 1d);

	public static final Quaternion ZERO = new Quaternion(0d, 0d, 0d, 0d);
	public static final double INVERSE_LN10 = 1 / StrictMath.log(10d);
	private double _w;
	private double _x;
	private double _y;
	
	private double _z;

	// Constructors:

	/**
	 * Quaternion constructor: [ w(<b>1</b>) + x(<b>i</b>)+ y(<b>j</b>)+
	 * z(<b>k</b>)]
	 * 
	 * @param w
	 *            -- Coefficient {<b>1</b>} real.
	 * @param x
	 *            -- Coefficient {<b>i</b>} first imaginary.
	 * @param y
	 *            -- Coefficient {<b>j</b>} second imaginary.
	 * @param z
	 *            -- Coefficient {<b>k</b>} third imaginary.
	 */
	public Quaternion(double w, double x, double y, double z) {
		_w = w;
		_x = x;
		_y = y;
		_z = z;
	}

	/**
	 * Quaternion constructor:
	 * 
	 * [ w(<b>1</b>) + x(<b>i</b>)+ y(<b>j</b>)+ z(<b>k</b>)]
	 * <p>
	 * Real component: [w(<b>1</b>)] <br>
	 * Imaginary vector component: [ x(<b>i</b>)+ y(<b>j</b>)+ z(<b>k</b>)]
	 * 
	 * @param w
	 *            : double - Real coefficient {<b>1</b>}.
	 * @param v
	 *            : Vector3 - Imaginary coefficients
	 *            {<b>i</b>,<b>j</b>,<b>k</b>}.
	 */
	public Quaternion(double w, final Vector3 v) {
		_w = w;
		_x = v.getX();
		_y = v.getY();
		_z = v.getZ();
	}

	/**
	 * Quaternion copy constructor: [ w(<b>1</b>) + x(<b>i</b>)+ y(<b>j</b>)+
	 * z(<b>k</b>)]
	 * 
	 * @param t
	 *            : Quaternion --> template for copy.
	 */
	public Quaternion(final Quaternion t) {
		_w = t._w;
		_x = t._x;
		_y = t._y;
		_z = t._z;
	}

	/**
	 * Mutator.
	 * 
	 * @param addend
	 *            Quaternion
	 * @return sum --> this plus the addend.
	 */
	public final Quaternion add(final Quaternion addend) {
		_w += addend._w;
		_x += addend._x;
		_y += addend._y;
		_z += addend._z;
		return this;
	}

	/** 
	 * Mutator. 
	 * @return this.add(new Quaternion(q).leftMultiply(UNIT_I));
	 * */
	protected final Quaternion addLeftMultiplyI(final Quaternion q) {
		_w -= q._x;
		_x += q._w;
		_y -= q._z;
		_z += q._y;
		return this;
	}

	/** 
	 * Mutator. 
	 * @return this.add(new Quaternion(q).leftMultiply(UNIT_J));
	 * */
	protected final Quaternion addLeftMultiplyJ(final Quaternion q) {
		_w -= q._y;
		_x += q._z;
		_y += q._w;
		_z -= q._x;
		return this;
	}

	// Getters:

	/** 
	 * Mutator. 
	 * @return this.add(new Quaternion(q).leftMultiply(UNIT_K));
	 * */
	protected final Quaternion addLeftMultiplyK(final Quaternion q) {
		_w -= q._z;
		_x -= q._y;
		_y += q._x;
		_z += q._w;
		return this;
	}

	/**
	 * mutator: this.add(new Quaternion(q).multiply(scalar));
	 * 
	 * @param scalar
	 *            double
	 * 
	 * @param q
	 *            Quaternion
	 * 
	 */
	protected final Quaternion addMultiply(final double scalar, final Quaternion q) {
		_w += (scalar * q._w);
		_x += (scalar * q._x);
		_y += (scalar * q._y);
		_z += (scalar * q._z);
		return this;
	}

	/** 
	 * Mutator. 
	 * @param q Quaternion
	 * @param a double
	 * @return this.add(new Qaternion(q).multiply(scalar).leftMultiply(UNIT_I));
	 * */
	protected final Quaternion addMultiplyLeftI(final double a, final Quaternion q) {
		_w -= (a * q._x);
		_x += (a * q._w);
		_y -= (a * q._z);
		_z += (a * q._y);
		return this;
	}

	/** Add left multiply right Quaternion factor by UNIT_J and scalar. Mutator. */
	/** 
	 * Mutator. 
	 * @param q Quaternion
	 * @param a double
	 * @return this.add(new Qaternion(q).multiply(scalar).leftMultiply(UNIT_J));
	 * */
	protected final Quaternion addMultiplyLeftJ(final double a, final Quaternion q) {
		_w -= (a * q._y);
		_x += (a * q._z);
		_y += (a * q._w);
		_z -= (a * q._x);
		return this;
	}

	/** 
	 * Mutator. 
	 * @param q Quaternion
	 * @param a double
	 * @return this.add(new Qaternion(q).multiply(scalar).leftMultiply(UNIT_K));
	 * */
	protected final Quaternion addMultiplyLeftK(final double a, final Quaternion q) {
		_w -= (a * q._z);
		_x -= (a * q._y);
		_y += (a * q._x);
		_z += (a * q._w);
		return this;
	}

// Member functions
	
	/** 
	 * Mutator. 
	 * @param p Quaternion
	 * @param a double
	 * @return this.add(new Qaternion(p).multiply(scalar).rightMultiply(UNIT_I));
	 * */
	protected final Quaternion addMultiplyRightI(final double a, final Quaternion p) {
		_w -= (a * p._x);
		_x += (a * p._w);
		_y += (a * p._z);
		_z -= (a * p._y);
		return this;
	}

	/** 
	 * Mutator. 
	 * @param p Quaternion
	 * @param a double
	 * @return this.add(new Qaternion(p).multiply(scalar).rightMultiply(UNIT_J));
	 * */
	protected final Quaternion addMultiplyRightJ(final double a, final Quaternion p) {
		_w -= (a * p._y);
		_x -= (a * p._z);
		_y += (a * p._w);
		_z += (a * p._x);
		return this;
	}

	/** 
	 * Mutator. 
	 * @param p Quaternion
	 * @param a double
	 * @return this.add(new Qaternion(p).multiply(scalar).rightMultiply(UNIT_K));
	 * */
	protected final Quaternion addMultiplyRightK(final double a, final Quaternion p) {
		_w -= (a * p._z);
		_x += (a * p._y);
		_y -= (a * p._x);
		_z += (a * p._w);
		return this;
	}

	/** 
	 * Mutator. 
	 * @return this.add(new Quaternion(p).rightMultiply(UNIT_I));
	 * */
	protected final Quaternion addRightMultiplyI(final Quaternion p) {
		_w -= p._x;
		_x += p._w;
		_y += p._z;
		_z -= p._y;
		return this;
	}

 	/** 
	 * Mutator. 
	 * @return this.add(new Quaternion(p).rightMultiply(UNIT_J));
	 * */
	protected final Quaternion addRightMultiplyJ(final Quaternion p) {
		_w -= p._y;
		_x -= p._z;
		_y += p._w;
		_z += p._x;
		return this;
	}

	/** 
	 * Mutator. 
	 * @return this.add(new Quaternion(p).rightMultiply(UNIT_K));
	 * */
	protected final Quaternion addRightMultiplyK(final Quaternion p) {
		_w -= p._z;
		_x += p._y;
		_y -= p._x;
		_z += p._w;
		return this;
	}

	/**
	 * Mutator.
	 * 
	 * @return --> this conjugated.
	 */
	public final Quaternion conjugate() {
		_x = -_x;
		_y = -_y;
		_z = -_z;
		return this;
	}
	/**
	 * Mutator.
	 * 
	 * @param scalarDivisor
	 * @return quotient --> this divided by scalarDivisor.
	 * 
	 */
	public final Quaternion divide(final double scalarDivisor) {
		if (scalarDivisor == 0) {
			set(Quaternion.EMPTY);
			return this;
		}
		_w /= scalarDivisor;
		_x /= scalarDivisor;
		_y /= scalarDivisor;
		_z /= scalarDivisor;
		return this;
	}
	
	/**
	 * (non-Javadoc)
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (obj == null) {
			return false;
		}
		if (!((obj instanceof Quaternion) || (obj instanceof Rotator))) {
			return false;
		}
		final Quaternion other = (Quaternion) obj;
		if (Double.doubleToLongBits(_w) != Double.doubleToLongBits(other._w)) {
			return false;
		}
		if (Double.doubleToLongBits(_x) != Double.doubleToLongBits(other._x)) {
			return false;
		}
		if (Double.doubleToLongBits(_y) != Double.doubleToLongBits(other._y)) {
			return false;
		}
		if (Double.doubleToLongBits(_z) != Double.doubleToLongBits(other._z)) {
			return false;
		}
		return true;
	}


	// Setters:

	/**
	 * Mutator.
	 * <p>
	 * Exponentiate: Put <b><i>e</i></b> raised to this Quaternion power <br>
	 * (e to the t cosine absolute value of V, e to the t sine of the absolute
	 * value of V times V normalized to V )
	 *
	 */
	public final Quaternion exp() {
		final double vectorMagnitude = getV().getAbs();
		//int k = (int) StrictMath.round(vectorMagnitude/StrictMath.PI);
		_w = StrictMath.exp(_w);
		if (vectorMagnitude == 0) {
			// scalar case...
			return this;
		}
		final double tan = StrictMath.tan(StrictMath.IEEEremainder(vectorMagnitude, Angle.STRAIGHT.getRadians()));
		if (StrictMath.abs(tan) > 1) {
			// obtuse angle
			final double csc = StrictMath.hypot(1, 1 / tan);
			multiplyVectorPart(_w/(vectorMagnitude * csc));
			_w /= (tan * csc);
			return this;
		}
		//  accute angle
		final double sec = StrictMath.hypot(1, tan);
		multiplyVectorPart(_w/(StrictMath.abs(vectorMagnitude / tan) * sec));
		_w /= StrictMath.copySign(sec, tan);
		return this;
	}

	// Mutators

	/**
	 * @return Modulus --> Quaternion's Euclidian norm (magnitude).
	 */
	public double getAbs() {
		final double testNorm = StrictMath.scalb(getNorm1(),-1);
		if (testNorm == 0) {
			// ..zero Quaternion.
			return 0d;
		}
		if (testNorm > 2 || testNorm < 1) {
			// ..numerically scaled.
			//return new Quaternion(this).divide(testNorm).getNorm2() * testNorm;
			double w = _w / testNorm;
			double x = _x / testNorm;
			double y = _y / testNorm;
			double z = _z / testNorm;
			return testNorm * StrictMath.sqrt(w * w + x * x + y * y + z * z);
		}
		return getNorm2(); //StrictMath.sqrt(getDeterminant());
	}

	/**
	 * @return Argument --> Principle angle of rotation by Quaternion's
	 *         multiplication (qtq*).
	 */
	public final CodedPhase getArg() {
		final double v = getV().getAbs();
		if (_w == 0) {
			if (v == 0) {
				// ZERO quaternion has argument defined as empty.
				return new CodedPhase(CodedPhase.EMPTY);
				//return null;
			}
			// right-angle quaternion defines argument as positive infinity.
			return new CodedPhase(CodedPhase.RIGHT);
		}
		return CodedPhase.encodes(v / _w); //half rotation angle tangent
	}

	/**
	 * @return Determinant --> Sum of Quaternion's basis element coefficients
	 *         squared
	 */
	public double getDeterminant() {
		return  _w * _w + _x * _x + _y * _y + _z * _z;
	}


	/**
	 * Quaternion (City Block) 1-Norm:
	 *
	 * @return double 1-Norm
	 */
	public double getNorm1() {
		return StrictMath.abs(_w) + StrictMath.abs(_x) + StrictMath.abs(_y)
				+ StrictMath.abs(_z);
	}

	/**
	 * Quaternion (Euclidean) 2-Norm:
	 *
	 * @return double 2-Norm
	 * <p>Note: Fast abs() calculation without scaling.
	 */
	public double getNorm2() {
		return StrictMath.sqrt(getDeterminant());
	}

	/**
	 * Quaternion Inf-Norm:
	 *
	 * @return double max absolute element
	 */
	public double getNormInf() {
		return StrictMath.max(
				StrictMath.max(StrictMath.abs(_w), StrictMath.abs(_x)),
				StrictMath.max(StrictMath.abs(_y), StrictMath.abs(_z)));
	}

	/**
	 * @return Vector of the imaginary part of Quaternion basis.
	 */
	public final Vector3 getV() {
		return new Vector3(_x, _y, _z);
	}

	/**
	 * @return real part of Quaternion basis.
	 */
	public double getW() {
		return _w;
	}

	/**
	 * @return First imaginary part basis coefficient.
	 */
	public double getX() {
		return _x;
	}



	/**
	 * @return Second imaginary part basis coefficient.
	 */
	public double getY() {
		return _y;
	}


	/**
	 * @return Third imaginary part basis coefficient.
	 */
	public double getZ() {
		return _z;
	}



	/**
	 * (non-Javadoc)
	 *
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(_w);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(_x);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(_y);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(_z);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	/**
	 * Check if undefined:
	 *
	 * @return boolean [TRUE:] contains NaN.
	 */
	public boolean hasNan() {
		// assert(Double.isInfinite(10d + Double.POSITIVE_INFINITY));
		return Double.isNaN(_w + _x + _y + _z);
	}

	/**
	 * (non-Javadoc)
	 *
	 * @see Object#equals(Object)
	 */
	public boolean isEquivalent(Object obj, double tolerance) {
		if (this == obj)
			return true;
		
		if (obj == null) return false; //Should not allow nulls..
//		if (obj == null) {
//			if (this == null)
//				return true;
//			return false;
//		}
		
//		 if (getClass() != obj.getClass())
//		 return false;
		
		if (!( (obj instanceof Quaternion) || (obj instanceof Rotator) ))
			return false;
		
		final Quaternion other = ((Quaternion) obj);
		if (other.hasNan())
			return false;
		
		//Double delta = new Quaternion(other).subtract(this).norm1();
		//Double delta = new Quaternion(other.subtract(this)).normInf();
		Double delta = new Quaternion(other).subtract(this).getAbs();
		if (delta < tolerance)
			return true;
		
		//delta = new Quaternion(other).add(this).norm1();
		//delta = new Quaternion(other.add(this)).normInf();
		delta = new Quaternion(other).add(this).getAbs();
		if (delta < tolerance)
			return true;

		return false;
		
	}

	

	

	//public static functions:
	
	/**
	 * Mutator.
	 * 
	 * @param p
	 * @return product --> this pre-multiplied by left factor p.
	 */
	public final Quaternion leftMultiply(final Quaternion p) {
//		final Quaternion t = new Quaternion(this);	
//		return this.multiply(leftFactor._w).addMultiplyLeftI(leftFactor._x, t)
//				.addMultiplyLeftJ(leftFactor._y, t).addMultiplyLeftK(leftFactor._z, t);
		return this.setProduct(p._w, p._x, p._y, p._z, _w, _x, _y, _z);
	}

	/**
	 * Mutator.
	 * 
	 * @param vp Vector3
	 * @return product --> this pre-multiplied by vp.
	 */
	public final Quaternion leftMultiply(final Vector3 vp){
		//double px, double py, double pz) {
		double qx = _x;
		double qy = _y;
		double qz = _z;
		_x = _w * vp._x - qy * vp._z + qz * vp._y;
		_y = _w * vp._y + qx * vp._z - qz * vp._x;
		_z = _w * vp._z - qx * vp._y + qy * vp._x;
		_w = -(vp._x * qx + vp._y * qy + vp._z * qz);
		return this;
//		double qw = _w;
//		double qx = _x;
//		double qy = _y;
//		double qz = _z;
//		_w = -(px * qx + py * qy + pz * qz);
//		_x = px * qw + py * qz - pz * qy;
//		_y = -px * qz + py * qw + pz * qx;
//		_z = px * qy - py * qx + pz * qw;		
//		return this;
		
//		final Quaternion t = new Quaternion(this);
//		return this.putLeftI(t).multiply(vp._x)
//				.addMultiplyLeftJ(vp._y, t)
//				.addMultiplyLeftK(vp._z, t);

		
	}

	/**
	 * Mutator.
	 * 
	 * Put natural ln(this)
	 */
	public final Quaternion ln() {
		final double w = _w;
		final double v = StrictMath.sqrt(_x*_x+_y*_y+_z*_z);
		_w = StrictMath.scalb(StrictMath.log(w * w + v * v), -1);
		if (v == 0) {
			return this;
		}
		multiplyVectorPart(StrictMath.atan2(v, w) / v);
		return this;
	}

	/**
	 * Mutator.
	 * 
	 * Put log10(this)
	 */
	public final Quaternion log10() {
		final double w = getW();
		final double v = getV().getAbs();
		_w = StrictMath.scalb(StrictMath.log(v * v + w * w), -1) * INVERSE_LN10;
		if (v == 0) {
			return this;
		}
		multiplyVectorPart(INVERSE_LN10 * StrictMath.atan2(v, w) / v);
		return this;
	}
	
	/**
	 * Mutator.
	 * 
	 * @param scalarFactor
	 * @return product --> this multiplied by scalarFactor.
	 */
	public final Quaternion multiply(final double scalarFactor) {
		_w *= scalarFactor;
		_x *= scalarFactor;
		_y *= scalarFactor;
		_z *= scalarFactor;
		return this;
	}
	
	
	/**
	 * Scalar product mutator.
	 * 
	 * @param scalarFactor
	 *            double
	 */
	private final void multiplyVectorPart(final double scalarFactor) {
		_x *= scalarFactor;
		_y *= scalarFactor;
		_z *= scalarFactor;
	}

	/**
	 * Mutator:
	 * 
	 * @return --> this negated (Doppleganger).
	 */
	public final Quaternion negate() {
		_w = -_w;
		_x = -_x;
		_y = -_y;
		_z = -_z;
		return this;
	}

	/**
	 * Mutator.
	 * 
	 * Put this^(scalar) mutator:
	 */
	public final Quaternion power(double s) {
		return (ln().multiply(s)).exp();
	}
	
	/**
	 * Mutator.
	 * 
	 * Put this^(q) mutator:
	 */
	public final Quaternion power(Quaternion q) {
		set( QuaternionMath.multiply(ln(), q));
		return this.exp();
	}

	/**
	 * Mutator.
	 * 
	 * Put this^(vector) mutator:
	 */
	public final Quaternion power(Vector3 v) {
        set(QuaternionMath.multiply(ln(), v));
        return this.exp();
        
	}	
	
	/**
	 * Mutator.
	 * 
	 * @return reciprocal --> this inverted.
	 */
	public final Quaternion reciprocal() {
		// return conjugate().divide(determinant());
		final double d = -(_w*_w + _x*_x + _y*_y + _z*_z);//-getDeterminant();
		if (d == 0) {
			set(Quaternion.EMPTY);
			return this;
		}
		_x /= d;
		_y /= d;
		_z /= d;
		_w /= (-d);
		return this;
	}

	public final Quaternion rexp() {
		
		double qMagnitude = getAbs();
		
		if(qMagnitude==0) return Quaternion.EMPTY;
		
		if (_w<0){qMagnitude=-qMagnitude;}
		
		divide(qMagnitude);	
		
		//unit Quaternion phase...
		final double vectorMagnitude = getV().getAbs(); //sine magnitude
		
		_w = StrictMath.exp(_w);
		if (vectorMagnitude == 0) { 
			// scalar case...
			return this;
		}
		final double tan = StrictMath.tan(vectorMagnitude);
		if (StrictMath.abs(tan) > 1) { 
			// obtuse angle
			final double csc = StrictMath.hypot(1, 1 / tan);
			multiplyVectorPart(_w/(vectorMagnitude * csc));
			_w /= (tan * csc);
			return this;
		} 
		//  accute angle
		final double sec = StrictMath.hypot(1, tan);
		multiplyVectorPart(_w/(StrictMath.abs(vectorMagnitude / tan) * sec));
		_w /= StrictMath.copySign(sec, tan);
		return this;
	}
	
	//public static functions:
		
	/**
	 * Mutator.
	 * 
	 * @param q Quaternion
	 * @return product --> this (post) multiplied by right factor q.
	 */
	public final Quaternion rightMultiply(final Quaternion q) {
//		final Quaternion t = new Quaternion(this);		
//		return this.multiply(rightFactor._w).addMultiplyRightI(rightFactor._x, t)
//				.addMultiplyRightJ(rightFactor._y, t).addMultiplyRightK(rightFactor._z, t);
		return this.setProduct(_w, _x, _y, _z, q._w, q._x, q._y, q._z);		
		
	}

	/**
	 * Mutator.
	 * 
	 * @param vq Vector3
	 * 
	 * @return product --> this [post] multiplied by vq.
	 */
	public final Quaternion rightMultiply(final Vector3 vq){

		double px = _x;
		double py = _y;
		double pz = _z;
		_x = _w * vq._x + py * vq._z - pz * vq._y;
		_y = _w * vq._y - px * vq._z + pz * vq._x;
		_z = _w * vq._z + px * vq._y - py * vq._x;
		_w = -(px * vq._x + py * vq._y + pz * vq._z);
		return this;		
//		double pw = _w;
//		double px = _x;
//		double py = _y;
//		double pz = _z;
//		_w = -(px * q_x + py * q_y + pz * q_z);
//		_x = pw * q_x + py * q_z - pz * q_y;
//		_y = pw * q_y - px * q_z + pz * q_x;
//		_z = pw * q_z + px * q_y - py * q_x;		
//		return this;
		
//		final Quaternion t = new Quaternion(this);				
//		return this.putRightI(t).multiply(vq._x)
//				.addMultiplyRightJ(vq._y, t)
//				.addMultiplyRightK(vq._z, t);		

		
	}

	/**
	 * Mutator.
	 * 
	 * @param w
	 *            : Real coefficient.
	 * @param x
	 *            : First imaginary coefficient.
	 * @param y
	 *            : Second imaginary coefficient.
	 * @param z
	 *            : Third imaginary coefficient.
	 */
	public final void set(double w, double x, double y, double z) {
		_w = w;
		_x = x;
		_y = y;
		_z = z;
	}

//	/**
//	 * Mutator.
//	 * 
//	 * SLERP mutator:
//	 */
//	public Quaternion slerp(final Quaternion p, final double t) {
//		set(QuaternionMath.slerp(this, p, t));
//		return this;
//	}
//

	/**
	 * Mutator.
	 * 
	 * @param w
	 *            : Real coefficient.
	 * @param v
	 *            : Imaginary coefficients.
	 */
	public final void set(double w, Vector3 v) {
		_w = w;
		_x = v._x; //getX();
		_y = v._y; //getY();
		_z = v._z; //getZ();
	}


	/**
	 * Mutator.
	 * 
	 * @param t
	 *            : template.
	 */
	public final void set(final Quaternion t) {
		if (this != t) {
			_w = t._w;
			_x = t._x;
			_y = t._y;
			_z = t._z;
		} // do nothing ...else throw?
	}

	
	
	
	
	/** 
	 * Mutator:<p>
	 * @param q 
	 * @return this.set(new Quaternion(q).leftMultiply(Quaternion.UNIT_I));
	 * */
	public final void setLeftMultiplyI(final Quaternion q) {
		//make sure this does not equal q !
		_w = -q._x;
		_x = q._w;
		_y = -q._z;
		_z = q._y;
	}

	/** 
	 * Mutator:<p>
	 * @param q 
	 * @return this.set(new Quaternion(q).leftMultiply(Quaternion.UNIT_J));
	 * */
	public final void setLeftMultiplyJ(final Quaternion q) {
		//make sure this does not equal q !
		_w = -q._y;
		_x = q._z;
		_y = q._w;
		_z = -q._x;
	}

	/** 
	 * Mutator: Must not use q self-referent.<p>
	 * @param q 
	 * @return this.set(new Quaternion(q).leftMultiply(Quaternion.UNIT_K));
	 * */
	public final void setLeftMultiplyK(final Quaternion q) {
		//make sure this does not equal q !
		_w = -q._z;
		_x = -q._y;
		_y = q._x;
		_z = q._w;
	}

	private final Quaternion setProduct(final double pw, final double px, final double py, final double pz,
                                        final double qw, final double qx, final double qy, final double qz) {
		_x = pw * qx + px * qw + py * qz - pz * qy;
		_y = pw * qy - px * qz + py * qw + pz * qx;
		_z = pw * qz + px * qy - py * qx + pz * qw;
		_w = pw * qw - (px * qx + py * qy + pz * qz);
		return this;
	}

	/** 
	 * Mutator:<p>
	 * @param p 
	 * @return this.set(new Quaternion(p).rightMultiply(Quaternion.UNIT_I));
	 * */
	public final void setRightMultiplyI(final Quaternion p) {
		//make sure this does not equal p !
		_w = -p._x;
		_x = p._w;
		_y = p._z;
		_z = -p._y;
	}

	/** 
	 * Mutator:<p>
	 * @param p 
	 * @return this.set(new Quaternion(p).rightMultiply(Quaternion.UNIT_J));
	 * */
	public final void setRightMultiplyJ(final Quaternion p) {
		//make sure this does not equal p !
		_w = -p._y;
		_x = -p._z;
		_y = p._w;
		_z = p._x;
	}



	/** 
	 * Mutator:<p>
	 * @param p 
	 * @return this.set(new Quaternion(p).rightMultiply(Quaternion.UNIT_K));
	 * */
	public final void setRightMultiplyK(final Quaternion p) {
		//make sure this does not equal p !
		_w = -p._z;
		_x = p._y;
		_y = -p._x;
		_z = p._w;
	}
	
	/**
	 * Mutator.
	 * 
	 * @param subtrahend
	 *            Quaternion
	 * @return difference --> this minus the subtrahend
	 */
	public final Quaternion subtract(final Quaternion subtrahend) {
		_w -= subtrahend._w;
		_x -= subtrahend._x;
		_y -= subtrahend._y;
		_z -= subtrahend._z;
		return this;
	}

	@Override
//	public String toString() {
//		return "< " + _w + "  + " + _x + "i  + " + _y + "j  + " + _z + "k >";
//	}
	public String toString() {
		int decimalDigits = 4;
		return toString(decimalDigits);
	}	

	public String toString(int decimals) {
		String fmt = "< %."+decimals+"f + %."+decimals+"fi + %."+decimals+"fj + %."+decimals + "fk >";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, _w, _x, _y, _z).replace("+ -", "- ");
	}

	public String toTupleString() {
		return "{ " + _w + ",  " + _x + ",  " + _y + ",  " + _z + " }";
	}

	/**
	 * Mutator.
	 * 
	 * @return versor --> this normalized to unit magnitude.
	 * */
	public final Quaternion unit() {
		final double abs = getAbs();
		return (abs == 0 || Double.isNaN(abs)) ? Quaternion.EMPTY : this
				.divide(abs);
	}
	
	
/* 
 * 
 * rank 2 axial transformations 
 * 
 */	
	
//	// Mutator.
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param direction
//	 *            Vector3
//	 * @return this left multiplied by Quaternion defined by tilt of basis unit
//	 *         <b>I</b> to align with <b>direction</b> vector3.
//	 */
//	public final Quaternion leftMultiplyTiltI(final Vector3 direction) {
//		// I==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//        final double w = direction.getAbs() + direction.getX();
//		final double yAbs = StrictMath.abs(direction.getY());
//		if (w > yAbs) {
//			if (w > StrictMath.abs(direction.getZ())) { // I
//				return this.addMultiplyLeftJ(direction.getZ() / w, t)
//						.addMultiplyLeftK(-direction.getY() / w, t);
//			}
//		} else {
//			if (yAbs > StrictMath.abs(direction.getZ())) { // J
//				return this.multiply(-w / direction.getY()).addLeftMultiplyK(t)
//						.addMultiplyLeftJ(-direction.getZ() / direction.getY(),t);
////				return this.multiplyAddLeftK(-w / direction.getY(), t)
////						.addMultiplyLeftJ(-direction.getZ() / direction.getY(),t);
//			}
//		}
//		// K
//		return this.multiply(w / direction.getZ()).addLeftMultiplyJ(t).addMultiplyLeftK(
//				-direction.getY() / direction.getZ(), t);
////		return this.multiplyAddLeftJ(w / direction.getZ(), t).addMultiplyLeftK(
////				-direction.getY() / direction.getZ(), t);
//	}
//	
	
	
	
//	public final Quaternion putRightTiltI(final Vector3 direction) {
//		//I==0...
//		final double w = direction.getAbs() + direction.getX();
//		final double absY = StrictMath.abs(direction.getY());
//		if (w > absY) {
//			if (w > StrictMath.abs(direction.getZ())) {
//				//I
//				_w=1;
//				_x=0;
//				_y=direction.getZ() / w;
//				_z=-direction.getY() / w;					
//				return this;
//			}
//		} else {
//			if (absY > StrictMath.abs(direction.getZ())) {
//				//J
//				_w=-w / direction.getY();
//				_x=0;
//				_y=-direction.getZ() / direction.getY();
//				_z=1;					
//				return this;
//			}
//		}
//		//K
//		_w=-w / direction.getZ();
//		_x=0;
//		_y=1;
//		_z=-direction.getY() / direction.getZ();					
//		return this;
//	}
//
//public final Quaternion putRightTiltJ(final Vector3 direction) {
////J==0...
//
//final double w = direction.getAbs() + direction.getY();
//final double xAbs = StrictMath.abs(direction.getX());
//
//if (w > xAbs) {
//	if (w > StrictMath.abs(direction.getZ())) {
//		//J
//		//return new Operator(1, -direction.getZ() / w, 0, direction.getX() / w);
//		_w=1;
//		_x=-direction.getZ() / w;
//		_y=0;
//		_z=direction.getX() / w;					
//		return this;
//	}
//
//} else {
//	if (xAbs > StrictMath.abs(direction.getZ())) {
//		//I
//		//return new Operator(w / direction.getX(), -direction.getZ() / direction.getX(), 0, 1);
//		_w=w / direction.getX();
//		_x=-direction.getZ() / direction.getX();
//		_y=0;
//		_z=1;					
//		return this;
//	}
//}
////K
////return new Operator(-w / direction.getZ(), 1, 0, -direction.getX() / direction.getZ());
//_w=-w / direction.getZ();
//_x=1;
//_y=0;
//_z=-direction.getX() / direction.getZ();					
//return this;
//}
//
//public final Quaternion putRightTiltK(final Vector3 direction) {
////K==0...
//final double w = direction.getAbs() + direction.getZ();
//final double xAbs = StrictMath.abs(direction.getX());
//
//if (w > xAbs) {
//	if (w > direction.getY()) {
//		//K
//		//return new Operator(1, direction.getY() / w, -direction.getX() / w, 0);
//		_w=1;
//		_x=direction.getY() / w;
//		_y=-direction.getX() / w;
//		_z=0;					
//		return this;
//	}
//
//} else {
//	if (direction.getY() > xAbs) {
//		//J
//		//return new Operator(w / direction.getY(), 1, -direction.getX() / direction.getY(), 0);
//		_w=w / direction.getY();
//		_x=1;
//		_y=-direction.getX() / direction.getY();
//		_z=0;					
//		return this;
//	}
//}
////I
////return new Operator(-w / direction.getX(), -direction.getY() / direction.getX(), 1 , 0 );
//_w=-w / direction.getX();
//_x= -direction.getY() / direction.getX();
//_y=1;
//_z=0;					
//return this;
//}
	
	
	
	
	

//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param direction
//	 *            Vector3
//	 * @return this left multiplied by Quaternion defined by tilt of basis unit
//	 *         <b>J</b> to align with <b>direction</b> vector3.
//	 */
//	public final Quaternion leftMultiplyTiltJ(final Vector3 direction) {
//		// J==0...
//		//t.set(this);
//        final Quaternion t = new Quaternion(this);
//		final double w = direction.getAbs() + direction.getY();
//		final double xAbs = StrictMath.abs(direction.getX());
//
//		if (w > xAbs) {
//			if (w > StrictMath.abs(direction.getZ())) { // J
//				return this.addMultiplyLeftI(-direction.getZ() / w, t)
//						.addMultiplyLeftK(direction.getX() / w, t);
//			}
//		} else {
//			if (xAbs > StrictMath.abs(direction.getZ())) { // I
//				return this.multiply(w / direction.getX()).addLeftMultiplyK(t)
//						.addMultiplyLeftI(-direction.getZ() / direction.getX(),t);
////				return this.multiplyAddLeftK(w / direction.getX(), t)
////						.addMultiplyLeftI(-direction.getZ() / direction.getX(),t);
//			}
//		}
//		// K
//		return this.multiply(-w / direction.getZ()).addLeftMultiplyI(t)
//				.addMultiplyLeftK(-direction.getX() / direction.getZ(), t);
////		return this.multiplyAddLeftI(-w / direction.getZ(), t)
////				.addMultiplyLeftK(-direction.getX() / direction.getZ(), t);
//	}
//
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param direction
//	 *            Vector3
//	 * @return this left multiplied by Quaternion defined by tilt of basis unit
//	 *         <b>K</b> to align with <b>direction</b> vector3.
//	 */
//	public final Quaternion leftMultiplyTiltK(final Vector3 direction) {
//		// K==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//
//		final double w = direction.getAbs() + direction.getZ();
//		final double xAbs = StrictMath.abs(direction.getX());
//
//		if (w > xAbs) {
//			if (w > direction.getY()) { // K
//				return this.addMultiplyLeftI(direction.getY() / w, t)
//						.addMultiplyLeftJ(-direction.getX() / w, t);
//			}
//		} else {
//			if (direction.getY() > xAbs) { // J
//				return this.multiply(w / direction.getY()).addLeftMultiplyI(t)
//						.addMultiplyLeftJ(-direction.getX() / direction.getY(),t);
////				return this.multiplyAddLeftI(w / direction.getY(), t)
////						.addMultiplyLeftJ(-direction.getX() / direction.getY(),t);
//			}
//		}
//		// I
//		return this.multiply(-w / direction.getX()).addLeftMultiplyJ(t)
//				.addMultiplyLeftI(-direction.getY() / direction.getX(), t);
////		return this.multiplyAddLeftJ(-w / direction.getX(), t)
////				.addMultiplyLeftI(-direction.getY() / direction.getX(), t);
//	}
//	
//	
//	//Mutator.
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param direction
//	 *            Vector3
//	 * @return this right multiplied by Quaternion defined by tilt of basis unit
//	 *         <b>I</b> to align with <b>direction</b> vector3.
//	 */
//	public final Quaternion rightMultiplyTiltI(final Vector3 direction) {
//		//I==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//		final double w = direction.getAbs() + direction.getX();
//		final double yAbs = StrictMath.abs(direction.getY());
//		if (w > yAbs) {
//			if (w > StrictMath.abs(direction.getZ())) { //I
//				return this.addMultiplyRightJ(direction.getZ() / w, t).addMultiplyRightK(
//						-direction.getY() / w, t);
//			}
//		} else {
//			if (yAbs > StrictMath.abs(direction.getZ())) { //J
//				return this.multiply(-w / direction.getY()).addRightMultiplyK(t)
//						.addMultiplyRightJ(-direction.getZ() / direction.getY(), t);
////				return this.multiplyAddRightK(-w / direction.getY(),t)
////						.addMultiplyRightJ(-direction.getZ() / direction.getY(), t);
//			}
//		}
//		//K
//		return this.multiply(w / direction.getZ()).addRightMultiplyJ(t)
//				.addMultiplyRightK(-direction.getY() / direction.getZ(), t);
////		return this.multiplyAddRightJ(w / direction.getZ(), t)
////				.addMultiplyRightK(-direction.getY() / direction.getZ(), t);
//	}
//	
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param direction
//	 *            Vector3
//	 * @return this right multiplied by Quaternion defined by tilt of basis unit
//	 *         <b>J</b> to align with <b>direction</b> vector3.
//	 */
//	public final Quaternion rightMultiplyTiltJ(final Vector3 direction) { // J==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//
//		final double w = direction.getAbs() + direction.getY();
//		final double xAbs = StrictMath.abs(direction.getX());
//		
//		if (w > xAbs) {
//			if (w > StrictMath.abs(direction.getZ())) { //J
//				return this.addMultiplyRightI(-direction.getZ() / w, t).addMultiplyRightK(
//						direction.getX() / w, t);
//			}
//		} else {
//			if (xAbs > StrictMath.abs(direction.getZ())) { //I
//				return this.multiply(w / direction.getX()).addRightMultiplyK(t)
//						.addMultiplyRightI(-direction.getZ() / direction.getX(), t);
////				return this.multiplyAddRightK(w / direction.getX(), t)
////						.addMultiplyRightI(-direction.getZ() / direction.getX(), t);
//			}
//		}
//		//K
//		return this.multiply(-w / direction.getZ()).addRightMultiplyI(t)
//				.addMultiplyRightK(-direction.getX() / direction.getZ(), t);
////		return this.multiplyAddRightI(-w / direction.getZ(), t)
////				.addMultiplyRightK(-direction.getX() / direction.getZ(), t);
//	}	
//
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param direction
//	 *            Vector3
//	 * @return this right multiplied by Quaternion defined by tilt of basis unit
//	 *         <b>K</b> to align with <b>direction</b> vector3.
//	 */
//	public final Quaternion rightMultiplyTiltK(final Vector3 direction) {
//		//K==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//		final double w = direction.getAbs() + direction.getZ();
//		final double xAbs = StrictMath.abs(direction.getX());
//	
//		if (w > xAbs) {
//			if (w > direction.getY()) { //K
//				return this.addMultiplyRightI(direction.getY() / w, t).addMultiplyRightJ(
//						-direction.getX() / w, t);
//			}
//	
//		} else {
//			if (direction.getY() > xAbs) { //J
//				return this.multiply(w / direction.getY()).addRightMultiplyI(t)
//						.addMultiplyRightJ(-direction.getX() / direction.getY(), t);
////				return this.multiplyAddRightI(w / direction.getY(), t)
////						.addMultiplyRightJ(-direction.getX() / direction.getY(), t);
//			}
//		}
//		//I
//		return this.multiply(-w / direction.getX()).addRightMultiplyJ(t)
//				.addMultiplyRightI(-direction.getY() / direction.getX(), t);
////		return this.multiplyAddRightJ(-w / direction.getX(), t)
////				.addMultiplyRightI(-direction.getY() / direction.getX(), t);
//	}
//
//
//	// Mutator.
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param omegaJ
//	 *            defines w_j <-- omegaJ.signedAngle()
//	 * @param omegaK
//	 *            defines w_k <-- omegaK.signedAngle()
//	 * 
//	 * @return this right multiplied by Quaternion defined by exp[
//	 *         <b>j</b>(w_j)/2 +<b>k</b>(w_k)/2 ]
//	 */
//	public final Quaternion rightMultiplyExpJK(final CodedPrinciple omegaJ, final CodedPrinciple omegaK) {
//		// I==0...
//		//t.set(this);
//
//		final Quaternion t = new Quaternion(this);
//		if (omegaJ.isAcute()) {
//			if (omegaK.isAcute()) {
//				return this.addMultiplyRightJ(omegaJ.tanHalf(), t)
//						.addMultiplyRightK(omegaK.tanHalf(), t);
//			}
//			return this.multiply(omegaK.cotHalf()).addRightMultiplyK(t)
//					.addMultiplyRightJ(omegaJ.tanHalf() / omegaK.tanHalf(), t);
////			return this.multiplyAddRightK(thetaK.cotHalf(), t)
////					.addMultiplyRightJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
//		}
//		if (omegaK.isGreaterAbs(omegaJ)) {
//			return this.multiply(omegaK.cotHalf()).addRightMultiplyK(t)
//					.addMultiplyRightJ(omegaJ.tanHalf() / omegaK.tanHalf(), t);
////			return this.multiplyAddRightK(thetaK.cotHalf(), t)
////					.addMultiplyRightJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
//		}
//		return this.multiply(omegaJ.cotHalf()).addRightMultiplyJ(t).addMultiplyRightK(
//				omegaK.tanHalf() / omegaJ.tanHalf(), t);
////		return this.multiplyAddRightJ(thetaJ.cotHalf(), t).addMultiplyRightK(
////				thetaK.tanHalf() / thetaJ.tanHalf(), t);
//
//	}
//
//	/**
//	 * Rank 2 Mutator.
//	 * 
//	 * @param omegaK
//	 *            defines w_k <-- omegaK.signedAngle()
//	 * @param omegaI
//	 *            defines w_i <-- omegaI.signedAngle()
//	 * 
//	 * @return this right multiplied by Quaternion defined by exp[
//	 *         <b>j</b>(w_i)/2 +<b>k</b>(w_k)/2 ]
//	 */
//	public final Quaternion rightMultiplyExpKI(final CodedPrinciple omegaK, final CodedPrinciple omegaI) {
//		// I==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//        if (omegaK.isAcute()) {
//			if (omegaI.isAcute()) {
//				return this.addMultiplyRightK(omegaK.tanHalf(), t)
//						.addMultiplyRightI(omegaI.tanHalf(), t);
//			}
//			return this.multiply(omegaI.cotHalf()).addRightMultiplyI(t)
//					.addMultiplyRightK(omegaK.tanHalf() / omegaI.tanHalf(), t);
////			return this.multiplyAddRightI(thetaI.cotHalf(), t)
////					.addMultiplyRightK(thetaK.tanHalf() / thetaI.tanHalf(), t);
//		}
//		if (omegaI.isGreaterAbs(omegaK)) {
//			return this.multiply(omegaI.cotHalf()).addRightMultiplyI(t)
//					.addMultiplyRightK(omegaK.tanHalf() / omegaI.tanHalf(), t);
////			return this.multiplyAddRightI(thetaI.cotHalf(), t)
////					.addMultiplyRightK(thetaK.tanHalf() / thetaI.tanHalf(), t);
//		}
//		return this.multiply(omegaK.cotHalf()).addRightMultiplyK(t).addMultiplyRightI(
//				omegaI.tanHalf() / omegaK.tanHalf(), t);
////		return this.multiplyAddRightK(thetaK.cotHalf(), t).addMultiplyRightI(
////				thetaI.tanHalf() / thetaK.tanHalf(), t);
//
//	}
//	
//	// Mutator.
//	public final Quaternion rightMultiplyExpIJ(final CodedPrinciple thetaI, final CodedPrinciple thetaJ) {
//		// I==0...
//		
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//
//		if (thetaI.isAcute()) {
//			if (thetaJ.isAcute()) {
//				return this.addMultiplyRightI(thetaI.tanHalf(), t)
//						.addMultiplyRightJ(thetaJ.tanHalf(), t);
//			}
//			return this.multiply(thetaJ.cotHalf()).addRightMultiplyJ(t)
//					.addMultiplyRightI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
////			return this.multiplyAddRightJ(thetaJ.cotHalf(), t)
////					.addMultiplyRightI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
//		}
//		if (thetaJ.isGreaterAbs(thetaI)) {
//			return this.multiply(thetaJ.cotHalf()).addRightMultiplyJ(t)
//					.addMultiplyRightI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
////			return this.multiplyAddRightJ(thetaJ.cotHalf(), t)
////					.addMultiplyRightI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
//		}
//		return this.multiply(thetaI.cotHalf()).addRightMultiplyI(t).addMultiplyRightJ(
//				thetaJ.tanHalf() / thetaI.tanHalf(), t);
////		return this.multiplyAddRightI(thetaI.cotHalf(), t).addMultiplyRightJ(
////				thetaJ.tanHalf() / thetaI.tanHalf(), t);
//
//	}
//		
//	// Mutator.
//	public final Quaternion leftMultiplyExpJK(final CodedPrinciple thetaJ, final CodedPrinciple thetaK) {
//		// I==0...
//		Quaternion t = new Quaternion(this);	//t anonymous with gc.	
////		if (t==null||t==this){
////			t = new Quaternion(this);	//t anonymous with gc.	
////		} else {
////			t.set(this);				//re-uses external t...class needs be unique per instance.
////		}
////		t.set(this);
//
//		if (thetaJ.isAcute()) {
//			if (thetaK.isAcute()) {
//				return this.addMultiplyLeftJ(thetaJ.tanHalf(), t)
//						.addMultiplyLeftK(thetaK.tanHalf(), t);
//			}
//			return this.multiply(thetaK.cotHalf()).addLeftMultiplyK(t)
//					.addMultiplyLeftJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
////			return this.multiplyAddLeftK(thetaK.cotHalf(), t)
////					.addMultiplyLeftJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
//		}
//		if (thetaK.isGreaterAbs(thetaJ)) {
//			return this.multiply(thetaK.cotHalf()).addLeftMultiplyK(t)
//					.addMultiplyLeftJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
////			return this.multiplyAddLeftK(thetaK.cotHalf(), t)
////					.addMultiplyLeftJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
//		}
//		return this.multiply(thetaJ.cotHalf()).addLeftMultiplyJ(t).addMultiplyLeftK(
//				thetaK.tanHalf() / thetaJ.tanHalf(), t);
////		return this.multiplyAddLeftJ(thetaJ.cotHalf(), t).addMultiplyLeftK(
////				thetaK.tanHalf() / thetaJ.tanHalf(), t);
//	}
//	
//	
//	// Mutator.
//	public final Quaternion leftMultiplyExpKI(final CodedPrinciple thetaK, final CodedPrinciple thetaI) {
//		// I==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//		if (thetaK.isAcute()) {
//			if (thetaI.isAcute()) {
//				return this.addMultiplyLeftK(thetaK.tanHalf(), t)
//						.addMultiplyLeftI(thetaI.tanHalf(), t);
//			}
//			return this.multiply(thetaI.cotHalf()).addLeftMultiplyI(t)
//					.addMultiplyLeftK(thetaK.tanHalf() / thetaI.tanHalf(), t);
////			return this.multiplyAddLeftI(thetaI.cotHalf(), t)
////					.addMultiplyLeftK(thetaK.tanHalf() / thetaI.tanHalf(), t);
//		}
//		if (thetaI.isGreaterAbs(thetaK)) {
//			return this.multiply(thetaI.cotHalf()).addLeftMultiplyI(t)
//					.addMultiplyLeftK(thetaK.tanHalf() / thetaI.tanHalf(), t);
////			return this.multiplyAddLeftI(thetaI.cotHalf(), t)
////					.addMultiplyLeftK(thetaK.tanHalf() / thetaI.tanHalf(), t);
//		}
//		return this.multiply(thetaK.cotHalf()).addLeftMultiplyK(t).addMultiplyLeftI(
//				thetaI.tanHalf() / thetaK.tanHalf(), t);
////		return this.multiplyAddLeftK(thetaK.cotHalf(), t).addMultiplyLeftI(
////				thetaI.tanHalf() / thetaK.tanHalf(), t);
//	}
//	
//	// Mutator.
//	public final Quaternion leftMultiplyExpIJ(final CodedPrinciple thetaI, final CodedPrinciple thetaJ) {
//		// I==0...
//		final Quaternion t = new Quaternion(this);
//		//t.set(this);
//        if (thetaI.isAcute()) {
//			if (thetaJ.isAcute()) {
//				return this.addMultiplyLeftI(thetaI.tanHalf(), t)
//						.addMultiplyLeftJ(thetaJ.tanHalf(), t);
//			}
//			return this.multiply(thetaJ.cotHalf()).addLeftMultiplyJ(t)
//					.addMultiplyLeftI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
////			return this.multiplyAddLeftJ(thetaJ.cotHalf(), t)
////					.addMultiplyLeftI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
//		}
//		if (thetaJ.isGreaterAbs(thetaI)) { 
//			return this.multiply(thetaJ.cotHalf()).addLeftMultiplyJ(t)
//					.addMultiplyLeftI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
////			return this.multiplyAddLeftJ(thetaJ.cotHalf(), t)
////					.addMultiplyLeftI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
//		} 
//		return this.multiply(thetaI.cotHalf()).addLeftMultiplyI(t).addMultiplyLeftJ(
//				thetaJ.tanHalf() / thetaI.tanHalf(), t);
////		return this.multiplyAddLeftI(thetaI.cotHalf(), t).addMultiplyLeftJ(
////				thetaJ.tanHalf() / thetaI.tanHalf(), t);
//	}
//	
//	
//	// four states...{0,acute,obtuse,straight} : two angles .. 16 possibilities....
//	
//	
//	/* 
//	 * 
//	 * rank 1 axial transformations 
//	 * 
//	 */	
//
//	public final Quaternion leftMultiplyExpI(final CodedPrinciple angle) {
//		if (angle.isZero()) {
//			return this;
//		}
//		final Quaternion clone = new Quaternion(this);
//		//clone.set(this);
//		if (angle.isAcute()) { // return exp_iAcute(angle) ;
//			return (Rotator) this.addMultiplyLeftI(angle.tanHalf(), clone);
//		}
//		if (angle.isStraight()) { // return this.flip_i();
//			this.setLeftMultiplyI(clone);
//			return (Rotator) this;
//		}
//		// return exp_iObtuse(angle) ;
//		return (Rotator) this.multiply(angle.cotHalf()).addLeftMultiplyI(clone);
////		return (Operator) this.multiplyAddLeftI(angle.cotHalf(), clone);
//		
//	}
//
//	public final Quaternion leftMultiplyExpJ(final CodedPrinciple angle) {
//		if (angle.isZero()) {
//			return this;
//		}
//		final Quaternion clone = new Quaternion(this);
//		//clone.set(this);
//		if (angle.isAcute()) { // return exp_iAcute(angle) ;
//			return (Rotator) this.addMultiplyLeftJ(angle.tanHalf(), clone);
//		}
//		if (angle.isStraight()) { // return this.flip_i();
//			this.setLeftMultiplyJ(clone);
//			return (Rotator) this;
//		}
//		// return exp_iObtuse(angle) ;
//		return (Rotator) this.multiply(angle.cotHalf()).addLeftMultiplyJ(clone);
////		return (Operator) this.multiplyAddLeftJ(angle.cotHalf(),clone);
//		
//	}
//
//	public final Quaternion leftMultiplyExpK(final CodedPrinciple angle) {
//		if (angle.isZero()) {
//			return this;
//		}
//		final Quaternion clone = new Quaternion(this);
//		//clone.set(this);
//		if (angle.isAcute()) { // return exp_iAcute(angle) ;
//			return (Rotator) this.addMultiplyLeftK(angle.tanHalf(), clone);
//		}
//		if (angle.isStraight()) { // return this.flip_i();
//			this.setLeftMultiplyK(clone);
//			return (Rotator) this;
//		}
//		// return exp_iObtuse(angle) ;
//		return (Rotator) this.multiply(angle.cotHalf()).addLeftMultiplyK(clone);
////		return (Operator) this.multiplyAddLeftK(angle.cotHalf(), clone);
//	}
//	
//	public final Quaternion rightMultiplyExpI(final CodedPrinciple thetaI) {
//		if (thetaI.isZero()) {
//			return this;
//		}
//		final Quaternion clone = new Quaternion(this);
//		//clone.set(this);
//        if (thetaI.isAcute()) { // return exp_iAcute(angle) ;
//			return (Rotator) this.addMultiplyRightI(thetaI.tanHalf(), clone);
//		}
//		if (thetaI.isStraight()) { // return this.flip_i();
//			this.setRightMultiplyI(clone);
//			return (Rotator) this;
//		}
//		// return exp_iObtuse(angle) ;
//		return (Rotator) this.multiply(thetaI.cotHalf()).addRightMultiplyI(clone);
////		return (Operator) this.multiplyAddRightI(thetaI.cotHalf(),clone);
//	
//	}
//
//	public final Quaternion rightMultiplyExpJ(final CodedPrinciple thetaJ) {
//		if (thetaJ.isZero()) {
//			return this;
//		}
//		final Quaternion clone = new Quaternion(this);
//		//clone.set(this);
//
//		if (thetaJ.isAcute()) { // return exp_iAcute(angle) ;
//			return (Rotator) this.addMultiplyRightJ(thetaJ.tanHalf(), clone);
//		}
//		if (thetaJ.isStraight()) { // return this.flip_i();
//			this.setRightMultiplyJ(clone);
//			return (Rotator) this;
//		}
//		// return exp_iObtuse(angle) ;
//		return (Rotator) this.multiply(thetaJ.cotHalf()).addRightMultiplyJ(clone);
////		return (Operator) this.multiplyAddRightJ(thetaJ.cotHalf(), clone);
//	}
//
//	public final Quaternion rightMultiplyExpK(final CodedPrinciple thetaK) {
//		if (thetaK.isZero()) {
//			return this;
//		}
//		final Quaternion clone = new Quaternion(this);
//		//clone.set(this);
//		if (thetaK.isAcute()) { // return exp_iAcute(angle) ;
//			return (Rotator) this.addMultiplyRightK(thetaK.tanHalf(), clone);
//		}
//		if (thetaK.isStraight()) { // return this.flip_i();
//			this.setRightMultiplyK(clone);
//			return (Rotator) this;
//		}
//		// return exp_iObtuse(angle) ;
//		return (Rotator) this.multiply(thetaK.cotHalf()).addRightMultiplyK(clone);	
////		return (Operator) this.multiplyAddRightK(thetaK.cotHalf(), clone);	
//	}
//

	
//	/**
//	 * Mutator.
//	 * 
//	 * @param s   : scalar
//	 * @param t   : does not equal this
//	 * 
//	 * @return this.set(new Quaternion(t).multiply(s));
//	 */
//	protected final Quaternion put(final double s, final Quaternion t) {
//		return put(t).multiply(s);				
////		_w = t._w * s;
////		_x = t._x * s;
////		_y = t._y * s;
////		_z = t._z * s;
////		return this;
//	}

//	/** 
//	 * Mutator:<p>
//	 * @param s
//	 * @param p does not equal this
//	 * @return this.put(new Quaternion(p).rightMultiply(Quaternion.UNIT_I).multiply(s));
//	 * */
//	protected final Quaternion putRightI(final double s, final Quaternion p) {
//		return putRightI(p).multiply(s);				
////		_w = -p._x * s;
////		_x = p._w * s;
////		_y = p._z * s;
////		_z = -p._y * s;
////		return this;
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param s
//	 * @param p does not equal this
//	 * @return this.put(new Quaternion(p).rightMultiply(Quaternion.UNIT_J).multiply(s));
//	 * */
//	protected final Quaternion putRightJ(final double s, final Quaternion p) {
//		return putRightJ(p).multiply(s);				
////		_w = -p._y * s;
////		_x = -p._z * s;
////		_y = p._w * s;
////		_z = p._x * s;
////		return this;
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param s
//	 * @param p 
//	 * @return this.put(new Quaternion(p).multiply(s).rightMultiply(Quaternion.UNIT_K));
//	 * */
//	protected final Quaternion putRightK(final double s, final Quaternion p) {
//		return putRightK(p).multiply(s);				
////		_w = -p._z * s;
////		_x = p._y * s;
////		_y = -p._x * s;
////		_z = p._w * s;
////		return this;
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param s
//	 * @param q 
//	 * @return this.put(new Quaternion(q).multiply(s).leftMultiply(Quaternion.UNIT_I));
//	 * */
//	protected final Quaternion putLeftI(final double s, final Quaternion q) {
//		return putLeftI(q).multiply(s);				
////		_w = -q._x * s;
////		_x = q._w * s;
////		_y = -q._z * s;
////		_z = q._y * s;
////		return this;
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param s
//	 * @param q 
//	 * @return this.put(new Quaternion(q).leftMultiply(Quaternion.UNIT_J).multiply(s));
//	 * */
//	protected final Quaternion putLeftJ(final double s, final Quaternion q) {
//		return putLeftJ(q).multiply(s);				
////		_w = -q._y * s;
////		_x = q._z * s;
////		_y = q._w * s;
////		_z = -q._x * s;
////		return this;
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param s
//	 * @param q 
//	 * @return this.put(new Quaternion(q).leftMultiply(Quaternion.UNIT_K).multiply(s));
//	 * */
//	protected final Quaternion putLeftK(final double s, final Quaternion q) {
//		return putLeftK(q).multiply(s);				
////		_w = -q._z * s;
////		_x = -q._y * s;
////		_y = q._x * s;
////		_z = q._w * s;
////		return this;
//	}
	
	
//	/**
//	 * mutator: this.multiply(scalar).add(q);
//	 * 
//	 * @param a
//	 *            double
//	 * 
//	 * @param q
//	 *            Quaternion
//	 * 
//	 */
//	protected final Quaternion multiplyAdd(final double a, final Quaternion q) {
//		return this.multiply(a).add(q);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w += q._w;
////		_x += q._x;
////		_y += q._y;
////		_z += q._z;
////		return this;
//	}

	
	
//	/** 
//	 * Mutator. 
//	 * @return this.multiply(a).add( new Quaternion(p).rightMultiply(UNIT_I) );
//	 * */
//	protected final Quaternion multiplyAddRightI(final double a, final Quaternion p) {
//		return this.multiply(a).addRightI(p);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w -= p._x;
////		_x += p._w;
////		_y += p._z;
////		_z -= p._y;
////		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * @return this.add(new Quaternion(p).rightMultiply(UNIT_J));
//	 * */
//	protected final Quaternion multiplyAddRightJ(final double a, final Quaternion p) {
//		return this.multiply(a).addRightJ(p);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w -= p._y;
////		_x -= p._z;
////		_y += p._w;
////		_z += p._x;
////		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * @return this.add(new Quaternion(p).rightMultiply(UNIT_K));
//	 * */
//	protected final Quaternion multiplyAddRightK(final double a, final Quaternion p) {
//		return this.multiply(a).addRightK(p);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w -= p._z;
////		_x += p._y;
////		_y -= p._x;
////		_z += p._w;
////		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * @return this.add(new Quaternion(q).leftMultiply(UNIT_I));
//	 * */
//	protected final Quaternion multiplyAddLeftI(final double a, final Quaternion q) {
//		return this.multiply(a).addLeftI(q);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w -= q._x;
////		_x += q._w;
////		_y -= q._z;
////		_z += q._y;
////		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * @return this.add(new Quaternion(q).leftMultiply(UNIT_J));
//	 * */
//	protected final Quaternion multiplyAddLeftJ(final double a, final Quaternion q) {
//		return this.multiply(a).addLeftJ(q);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w -= q._y;
////		_x += q._z;
////		_y += q._w;
////		_z -= q._x;
////		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * @return this.add(new Quaternion(q).leftMultiply(UNIT_K));
//	 * */
//	protected final Quaternion multiplyAddLeftK(final double a, final Quaternion q) {
//		return this.multiply(a).addLeftK(q);
////		_w *= a;
////		_x *= a;
////		_y *= a;
////		_z *= a;
////		_w -= q._z;
////		_x -= q._y;
////		_y += q._x;
////		_z += q._w;
////		return this;
//	}



//	/** 
//	 * Mutator. 
//	 * */
//	protected final Quaternion leftMultiply(final double pW, final double pX, final double pY, final double pZ) {
//		final double qW = _w;
//		final double qX = _x;
//		final double qY = _y;
//		final double qZ = _z;
//		_w = pW * qW - (pX * qX + pY * qY + pZ * qZ);
//		_x = pW * qX + pX * qW + pY * qZ - pZ * qY;
//		_y = pW * qY - pX * qZ + pY * qW + pZ * qX;
//		_z = pW * qZ + pX * qY - pY * qX + pZ * qW;
//		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * */
//	protected final Quaternion rightMultiply(final double qW, final double qX, final double qY, final double qZ) {
//		final double pW = _w;
//		final double pX = _x;
//		final double pY = _y;
//		final double pZ = _z;
//		_w = pW * qW - (pX * qX + pY * qY + pZ * qZ); //{pw,-px,-py,-pz}{qw,qx,qy,qz}'
//		_x = pX * qW + pW * qX - pZ * qY + pY * qZ;   //{px,pw,-pz,py}{qw,qx,qy,qz}'
//		_y = pY * qW + pZ * qX + pW * qY - pX * qZ;   //{py,pz,pw,-px}{qw,qx,qy,qz}'
//		_z = pZ * qW - pY * qX + pX * qY + pW * qZ;   //{pz,-py,px,pw}{qw,qx,qy,qz}'
//		return this;
//	}
//	
//
//	/** 
//	 * Mutator. 
//	 * */
//	protected final Quaternion leftMultiply(final double pX, final double pY, final double pZ) {
//		final double qW = _w;
//		final double qX = _x;
//		final double qY = _y;
//		final double qZ = _z;
//		_w = -(pX * qX + pY * qY + pZ * qZ);
//		_x = pX * qW + pY * qZ - pZ * qY;
//		_y = -pX * qZ + pY * qW + pZ * qX;
//		_z = pX * qY - pY * qX + pZ * qW;
//		return this;
//	}
//
//	/** 
//	 * Mutator. 
//	 * */
//	protected final Quaternion rightMultiply(final double qX, final double qY, final double qZ) {
//		final double pW = _w;
//		final double pX = _x;
//		final double pY = _y;
//		final double pZ = _z;
//		_w = -(pX * qX + pY * qY + pZ * qZ);
//		_x = pW * qX - pZ * qY + pY * qZ;
//		_y = pZ * qX + pW * qY - pX * qZ;
//		_z = -pY * qX + pX * qY + pW * qZ;
//		return this;
//	}
	
	
	
	

	//Single rotations	
	
	//FOLDOVER... rank 2 rotation, vector to frame axis alignment
		
	//Rotation about frame axis... rank 1 


		
	/* 
	 * 
	 * rank 3 axial transformations 
	 * 
	 */	
		
//		//Mutator.
//		public Quaternion rightMultiplyRotation(Vector3 fromDir, Vector3 toDir) {
//			//I==0...
//			final Vector3 a = new Vector3(fromDir).unit();
//			final Vector3 b = new Vector3(toDir).unit();
//			final Vector3 c = a.cross(b);
//			return this.rightMultiply(a.getDot(b),c.getX(),c.getY(),c.getZ());
//		}
	//	


	
	
	
	
//	public static Quaternion leftMultiplyFoldoverI(Quaternion product, Quaternion right, Vector3 direction) {
//	//I==0...
//	if (product.equals(null)) {
//		product = new Quaternion(right);
//	} else {
//		product.put(right);
//	}
//	double w = direction.getAbs() + direction.getX();
//	double yAbs = StrictMath.abs(direction.getY());
//	if (w > yAbs) {
//		if (w > StrictMath.abs(direction.getZ())) {
//			//I
//			return product.addLeftJ(direction.getZ() / w, right).addLeftK(
//					-direction.getY() / w, right);
//		}
//	} else {
//		if (yAbs > StrictMath.abs(direction.getZ())) {
//			//J
//			return product.multiply(-w / direction.getY())
//					.addLeftJ(-direction.getZ() / direction.getY(), right)
//					.addLeftK(product);
//		}
//	}
//	//K
//	return product.multiply(w / direction.getZ()).addLeftJ(right)
//			.addLeftK(-direction.getY() / direction.getZ(), right);
//}
//
//public static Quaternion rightMultiplyFoldoverI(Quaternion product, Quaternion left, Vector3 direction) {
//	//I==0...
//	if (product.equals(null)) {
//		product = new Quaternion(left);
//	} else {
//		product.put(left);
//	}
//	double w = direction.getAbs() + direction.getX();
//	double yAbs = StrictMath.abs(direction.getY());
//	if (w > yAbs) {
//		if (w > StrictMath.abs(direction.getZ())) {
//			//I
//			return product.addRightJ(direction.getZ() / w, left).addRightK(
//					-direction.getY() / w, left);
//		}
//	} else {
//		if (yAbs > StrictMath.abs(direction.getZ())) {
//			//J
//			return product.multiply(-w / direction.getY())
//					.addRightJ(-direction.getZ() / direction.getY(), left)
//					.addRightK(left);
//		}
//	}
//	//K
//	return product.multiply(w / direction.getZ()).addRightJ(left)
//			.addRightK(-direction.getY() / direction.getZ(), left);
//}
//
//public static Quaternion leftMultiplyFoldoverJ(Quaternion product, Quaternion right, Vector3 direction) {
//	//J==0...
//	if (product.equals(null)) {
//		product = new Quaternion(right);
//	} else {
//		product.put(right);
//	}
//	double w = direction.getAbs() + direction.getY();
//	double xAbs = StrictMath.abs(direction.getX());
//	
//	if (w > xAbs) {
//		if (w > StrictMath.abs(direction.getZ())) {
//			//J
//			//return new Operator(1, -direction.getZ() / w, 0, direction.getX() / w);
//			return product.addLeftI(-direction.getZ() / w, right).addLeftK(
//					direction.getX() / w, right);
//		}
//
//	} else {
//		if (xAbs > StrictMath.abs(direction.getZ())) {
//			//I
//			//return new Operator(w / direction.getX(), -direction.getZ() / direction.getX(), 0, 1);
//			return product.multiply(w / direction.getX())
//					.addLeftI(-direction.getZ() / direction.getX(), right)
//					.addLeftK(right);
//		}
//	}
//	//K
//	//return new Operator(-w / direction.getZ(), 1, 0, -direction.getX() / direction.getZ());
//	return product.multiply(-w / direction.getZ()).addLeftI(product)
//			.addLeftK(-direction.getX() / direction.getZ(), product);
//}
//
//public static Quaternion rightMultiplyFoldoverJ(Quaternion product, Quaternion left,Vector3 direction) {
//	//J==0...
//	if (product.equals(null)) {
//		product = new Quaternion(left);
//	} else {
//		product.put(left);
//	}
//	double w = direction.getAbs() + direction.getY();
//	double xAbs = StrictMath.abs(direction.getX());
//	
//	if (w > xAbs) {
//		if (w > StrictMath.abs(direction.getZ())) {
//			//J
//			//return new Operator(1, -direction.getZ() / w, 0, direction.getX() / w);
//			return product.addRightI(-direction.getZ() / w, left).addRightK(
//					direction.getX() / w, left);
//		}
//
//	} else {
//		if (xAbs > StrictMath.abs(direction.getZ())) {
//			//I
//			//return new Operator(w / direction.getX(), -direction.getZ() / direction.getX(), 0, 1);
//			return product.multiply(w / direction.getX())
//					.addRightI(-direction.getZ() / direction.getX(), left)
//					.addRightK(left);
//		}
//	}
//	//K
//	//return new Operator(-w / direction.getZ(), 1, 0, -direction.getX() / direction.getZ());
//	return product.multiply(-w / direction.getZ()).addRightI(product)
//			.addRightK(-direction.getX() / direction.getZ(), left);
//}	
//
//public static Quaternion leftMultiplyFoldoverK(Quaternion product, Quaternion right, Vector3 direction) {
//	//K==0...
//	if (product.equals(null)) {
//		product = new Quaternion(right);
//	} else {
//		product.put(right);
//	}
//	double w = direction.getAbs() + direction.getZ();
//	double xAbs = StrictMath.abs(direction.getX());
//
//	if (w > xAbs) {
//		if (w > direction.getY()) {
//			//K
//			//return new Operator(1, direction.getY() / w, -direction.getX() / w, 0);
//			return product.addLeftI(direction.getY() / w, right).addLeftJ(
//					-direction.getX() / w, right);
//		}
//
//	} else {
//		if (direction.getY() > xAbs) {
//			//J
//			//return new Operator(w / direction.getY(), 1, -direction.getX() / direction.getY(), 0);
//			return product.multiply(w / direction.getY()).addLeftI(right)
//					.addLeftJ(-direction.getX() / direction.getY(), right);
//		}
//	}
//	//I
//	//return new Operator(-w / direction.getX(), -direction.getY() / direction.getX(), 1 , 0 );
//	return product.multiply(-w / direction.getX())
//			.addLeftI(-direction.getY() / direction.getX(), right)
//			.addLeftJ(right);
//}
//
//public static Quaternion rightMultiplyFoldoverK(Quaternion product, Quaternion left, Vector3 direction) {
//	//K==0...
//	if (product.equals(null)) {
//		product = new Quaternion(left);
//	} else {
//		product.put(left);
//	}
//	double w = direction.getAbs() + direction.getZ();
//	double xAbs = StrictMath.abs(direction.getX());
//
//	if (w > xAbs) {
//		if (w > direction.getY()) {
//			//K
//			//return new Operator(1, direction.getY() / w, -direction.getX() / w, 0);
//			return product.addRightI(direction.getY() / w, left).addRightJ(
//					-direction.getX() / w, left);
//		}
//
//	} else {
//		if (direction.getY() > xAbs) {
//			//J
//			//return new Operator(w / direction.getY(), 1, -direction.getX() / direction.getY(), 0);
//			return product.multiply(w / direction.getY()).addRightI(left)
//					.addRightJ(-direction.getX() / direction.getY(), left);
//		}
//	}
//	//I
//	//return new Operator(-w / direction.getX(), -direction.getY() / direction.getX(), 1 , 0 );
//	return product.multiply(-w / direction.getX())
//			.addRightI(-direction.getY() / direction.getX(), left)
//			.addRightJ(left);
//}	
//
//


}