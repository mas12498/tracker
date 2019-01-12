package tspi.rotation;


public class Vector3 {
	
	public static final Vector3 EMPTY = new Vector3(Double.NaN,Double.NaN,Double.NaN);
	public static final Vector3 UNIT_I = new Vector3(1,0,0);
	public static final Vector3 UNIT_J = new Vector3(0,1,0);
	
	public static final Vector3 UNIT_K = new Vector3(0,0,1);	
	public static final Vector3 ZERO = new Vector3(0,0,0);	
	protected double _x;	
	protected double _y;	
	protected double _z;

	//Constructors:
	public Vector3(double vi,double vj,double vk){ _x=vi; _y=vj; _z=vk; }
	public Vector3(Vector3 v){ _x=v._x; _y=v._y; _z=v._z; }
	
	/** 
	 * Mutator.
	 * @param addend
	 * @return Sum --> this plus addend. 
	 */
	public Vector3 add(Vector3 addend)
	{
		_x += addend._x;
		_y += addend._y;
		_z += addend._z;
		return this;
	}

	/** 
	 * Mutator.
	 * @param rightFactor 
	 * @return (right) crossproduct --> this cross-multiplied by rightFactor.
	 */
	public Vector3 crossProduct(Vector3 rightFactor) {
		set(QuaternionMath.crossproduct(this, rightFactor));
		return this;
	}

	/**
	 * Mutator.
	 * @param scalarDivisor
	 * @return quotient --> this divided by scalarDivisor 
	 */
	public Vector3 divide(double scalarDivisor)
	{
		if(scalarDivisor==0) {
			//infinity? how small to allow division?
			this.set(EMPTY);
			return this;
		}
		_x /= scalarDivisor;
		_y /= scalarDivisor;
		_z /= scalarDivisor;
		return this;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (obj == null) {
			return false;
		}
		if (!(obj instanceof Vector3)) {
			return false;
		}
		Vector3 other = (Vector3) obj;
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

	/**
	 * The exponential of Vector:
	 * @return Quaternion:
	 * 
	 */
	public Quaternion exp(){
		double s = getAbs();
		//double s = StrictMath.scalb(abs(),-1);//half angle[?]
		double a = StrictMath.sin(s)/s;
		return new Quaternion( StrictMath.cos(s), _x*a, _y*a, _z*a );
	}
	
	public double getAbs(){
			double halfNorm1 = StrictMath.scalb(getNorm1(),-1);		
			return (halfNorm1 == 0) 
			? 0 : (halfNorm1 < 2 && halfNorm1 > .5)  
				?   Math.sqrt(getDeterminant())		//...no need for scaling.
				:	halfNorm1*StrictMath.sqrt( new Vector3(this).divide(halfNorm1).getDeterminant() ) ;		
			
	//		return (_x > _y)
	//			? (_x > _z)
	//				? StrictMath.abs(_x)*StrictMath.hypot(<<sign(x)>!1d>,StrictMath.hypot(_y/_x, _z/_x))
	//				: StrictMath.abs(_z)*StrictMath.hypot(1d,StrictMath.hypot(_x/_z, _y/_z))
	//			:  (_y > _z)
	//				? StrictMath.abs(_y)*StrictMath.hypot(1d,StrictMath.hypot(_x/_y, _z/_y))
	//				: StrictMath.abs(_z)*StrictMath.hypot(1d,StrictMath.hypot(_x/_z, _y/_z));
		
	}

	public double getDeterminant() {
		return _x * _x + _y * _y + _z * _z;
	}

	/**
	 * 
	 * @param factor
	 * @return inner product --> this dot-multiplied by factor.
	 */
	public double getInnerProduct(Vector3 factor) {
		return _x * factor._x + _y * factor._y + _z * factor._z;
	}

	public double getNorm1() {
		return StrictMath.abs(_x) + StrictMath.abs(_y) + StrictMath.abs(_z);
	}
	
	public double getNormInf() {
		return StrictMath.max(StrictMath.abs(_x),
				StrictMath.max(StrictMath.abs(_y), StrictMath.abs(_z)));
	}
	
	public double getX() {
		return _x;
	}
	
	public double getY() {
		return _y;
	}
	
	public double getZ() {
		return _z;
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(_x);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(_y);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(_z);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}
	
	/**
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	public boolean isEquivalent(Object obj, double tolerance) {
		if (this == obj) {
			return true;
		}
		if (obj == null) {
			return false;
		}
		if (!(obj instanceof Vector3)) {
			return false;
		}
		Vector3 other = (Vector3) obj;
		if (new Vector3(other).subtract(this).getNormInf() > tolerance) {
			return false;
		}
		return true;
	}
	
	/**
	 * Mutator.
	 * 
	 * @param leftFactor
	 * @return left crossproduct --> leftFactor cross-multiplied by this.
	 */
	public Vector3 leftCrossProduct(Vector3 leftFactor) {
		set(QuaternionMath.crossproduct(leftFactor, this));
		return this;
	}
	
	/** 
	 * Quaternion Factory. 
	 * @param leftFactor 
	 * @return Quaternion product --> this pre-multiplied by leftFactor.
	 */
	public Quaternion leftMultiply(Vector3 leftFactor)
	{
		return new Quaternion( getInnerProduct(leftFactor),
				QuaternionMath.crossproduct(leftFactor,this));
	}	
	
	/**
	 * The natural log of vector: Special reduction...
	 * @return Quaternion
	 * 
	 */
	public Quaternion ln(){		
		double s = getAbs();
		double a = Angle.RIGHT.getRadians()/s;
		return(s == 0) 
		  ?  new Quaternion(Quaternion.EMPTY)
		  :  new Quaternion( 
				StrictMath.log(s),
				_x*a,
				_y*a,
				_z*a
		     );
	}

	/**
	 * The log ( of vector: Special reduction...
	 * @return Quaternion:
	 */
	public Quaternion log10() {
		return ln().multiply(Quaternion.INVERSE_LN10);
	}


	/** 
	 * Mutator.
	 * @param scalarFactor
	 * @return product --> this multiplied by scalarFactor 
	 */
	public Vector3 multiply(double scalarFactor)
	{
		_x *= scalarFactor;
		_y *= scalarFactor;
		_z *= scalarFactor;
		return this;
	}

	/** 
	 * @param rightFactor 
	 * @return Quaternion product --> this (post) multiplied by rightFactor.
	 */
	public Quaternion multiply(Vector3 rightFactor)
	{
		return new Quaternion( getInnerProduct(rightFactor),
				               QuaternionMath.crossproduct(this,rightFactor)
				             );
	}
	
	/** Negate mutator. */
	public Vector3 negate() {
		_x = -_x;
		_y = -_y;
		_z = -_z;
		return this;
	}

	/**
	 * Raise vector to scalar s power:
	 * @return Quaternion:
	 */
	public Quaternion power(double s) {
		return (ln().multiply(s)).exp();
	}
	
	/**
	 * Raise vector to quaternion p power:
	 * @return Quaternion:
	 */
	public Quaternion power(Quaternion p) {
		return QuaternionMath.multiply(ln(), p).exp();
	}
	
	/**
	 * Raise vector to vector v power:
	 * @return Quaternion:
	 */
	public Quaternion power(Vector3 v) {
		return QuaternionMath.multiply(ln(), v).exp();
	}
	
	
	public void set(double x, double y, double z) {
		_x = x;
		_y = y;
		_z = z;
	}
	
	public void set(Vector3 v) {
		_x = v._x;
		_y = v._y;
		_z = v._z;
	}
	
	public void setX(double x) {
		_x = x;
	}
	
	public void setY(double y) {
		_y = y;
	}

	public void setZ(double z) {
		_z = z;
	}

	/**
	 * Mutator.
	 * 
	 * SLERP mutator:
	 */
	public Vector3 slerp(Vector3 v, double t) {
		// return new Operator(0, this).slerp(new Quaternion(0, v), t);
		return QuaternionMath.multiplyAndGetImaginaryPart(new Vector3(this)
				.divide(this.getInnerProduct(this)).multiply(v).power(t), this);
	}
	
	/** 
	 * Mutator.
	 * @param subtrahend
	 * @return Difference --> this minus subtrahend. 
	 */
	public Vector3 subtract(Vector3 subtrahend)
	{
		_x -= subtrahend._x;
		_y -= subtrahend._y;
		_z -= subtrahend._z;
		return this;
	}	
	
	@Override
	public String toString() {
		int decimals=4;
		return toString(decimals);
	}

	public String toString(int decimals) {
		String fmt = "< %."+decimals+"fi + %."+decimals+"fj + %."+decimals + "fk >";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, _x, _y, _z).replace("+ -", "- ");
	}

	public String toTupleString() {
		return "{ " + _x + ",  " + _y + ",  " + _z + " }";
	}
	
	/** 
	 * Mutator.
	 * @return Direction cosines --> this normalized to unit-length. 
	 */
	public Vector3 unit() {
		divide(getAbs());
		return this;
	}

//	/** Vector3 crossproduct
//	 * @param Vector3 factor 
//	 * @param Op hand [left|right] Vector3 crossproduct operation
//	 * @return Vector3 product */
//	public Vector3 cross(Vector3 factor, Op application) {
//		return put(application.isRight() ? QuaternionMath.crossproduct(this,
//				factor) : QuaternionMath.crossproduct(factor, this));
//	}

//	/** Hamiltonion product
//	 * @param Vector3 factor 
//	 * @param Op hand [left|right] Vector3 Hamiltonion product operation
//	 * @return Quaternion product */
//	public Quaternion multiply(Vector3 factor, Op application) {
//		return new Quaternion(dot(factor),
//				(application.isRight()) ? QuaternionMath.crossproduct(this,
//						factor) : QuaternionMath.crossproduct(factor, this));
//	}
//
//	/**
//	 * The fast exponential of v equals magnified..; 
//	 *  by factor of sqrt(1+t*t)     : t<1, 
//	 *  or factor of sqrt(1+1/(t*t)) : t=>1.
//	 */
//	protected Quaternion aExp(double aCoef){
//		double s = abs(); 
//		double  t = StrictMath.tan(s);
//		double nv; 
//		double sv;
//		if (StrictMath.abs(t)<1) { 
//			nv = aCoef/StrictMath.hypot(1d,t); 
//			sv = nv; 
//			nv *= (t/s); 
//		} else { 
//			t = 1d/t;
//			nv = aCoef/StrictMath.hypot(1d,t); 
//			sv = nv*t; 
//			nv /= s;
//		};
//		return new Quaternion( sv, (new Vector3(this)).multiply(nv) );
//	}	
	
//	/**
//	 * The exponential of Vector:
//	 * 
//	 * @return Quaternion:
//	 * 
//	 * < cosine absolute value of V, 
//	 *  sine of the absolute value of V times V normalized to V >
//	 */
//	public Operator factoredExp(){
//		double d = Angle.inRadians(abs()).getRadians(); //.getPrincipleRadians(); //principle bearing
//		double t = 	StrictMath.tan(StrictMath.scalb(d,-1)); 		
//		if(StrictMath.abs(t)>1.d){
//			t = 1/t;
//		} else {
//			d  /= t;			
//		}
//		return new Operator(
//				d,
//				this);
//	}

	
	
}
