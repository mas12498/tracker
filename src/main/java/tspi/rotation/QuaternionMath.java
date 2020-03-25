package tspi.rotation;

public class QuaternionMath {
	
	/**
	 * Adjoint Quaternion static factory.
	 */
	public static Quaternion adjoint(Quaternion p) {
		double negNormP = -p.getDeterminant();
		if (negNormP == 0 || Double.isNaN(negNormP)) {
			return new Quaternion(Quaternion.EMPTY);
		}
		return new Quaternion(-p.getW() * negNormP, p.getX() * negNormP, p.getY()
				* negNormP, p.getZ() * negNormP);
	}	
	
	
	/** Conjugate static factory.	 */
	public static final Rotator conjugate(Quaternion p) {
		return new Rotator(p.getW(), -p.getX(), -p.getY(), -p.getZ());
	}	

	
	/** Vector product static factory.
	 * @param Vector3 left factor 
	 * @param Vector3 right factor
	 * @return Vector crossproduct   */ 
	public static Vector3 crossproduct(Vector3 left, Vector3 right) {
		return new Vector3(left.getY() * right.getZ() - left.getZ() * right.getY(),
				left.getZ() * right.getX() - left.getX() * right.getZ(), left.getX()
						* right.getY() - left.getY() * right.getX());
	}	
	
	
	
    	/** 
		 * Factory.
		 * Operator quaternion from Tait-Bryan | Centerline Euler Angle static factory.	 
		 * @param pK  (k-axis) Angle #1: yaw | az
		 * @param pJ  (j-axis) Angle #2: pitch | el
		 * @return Quaternion body | centerline frame orientation */
		public static final Rotator eulerRotate_kj(CodedPhase pK, CodedPhase pJ) {
			/**
			 * this algorithm has issues when pJ 90 or when 
			 * greater than 90 and rotate pK (= 0|360)...
			 * or 
			 */
			return QuaternionMath.rotate_k(pK).rotate_j(pJ);
		}

	/** Quaternion operator from Tait-Bryan | Centerline Euler Angle static factory.	 
 * @param pK  (k-axis) Angle #1: yaw | az
 * @param pJ  (j-axis) Angle #2: pitch | el
 * @param pI  (i-axis) Angle #3: roll | twist
 * @return Quaternion body | centerline frame orientation */
public static final Rotator eulerRotate_kji(CodedPhase pK, CodedPhase pJ,
		CodedPhase pI) {
	return QuaternionMath.rotate_k(pK).rotate_j(pJ).rotate_i(pI);
}

	/**
	 * Exponentiate: Put <b><i>e</i></b> raised to this Quaternion power mutator <br>
	 * (e to the t cosine absolute value of V, e to the t sine of the absolute
	 * value of V times V normalized to V )
	 * 
	 */
	public static Quaternion exp(Quaternion p) {
		Vector3 v = p.getV();
		double vectorMagnitude = v.getAbs(); // Radians measure

		if (vectorMagnitude == 0) {
			return new Quaternion(p);
		}
		double tan = StrictMath.tan(vectorMagnitude);
//		double tan = StrictMath.tan(StrictMath.IEEEremainder(vectorMagnitude, StrictMath.PI));
		double w = StrictMath.exp(p.getW());
		if (StrictMath.abs(tan) <= 1) {
			return new Quaternion(
					StrictMath.copySign(StrictMath.hypot(1, tan), tan),
					v.multiply((w / vectorMagnitude) * tan));
		}
		tan = 1 / tan;
		w /= StrictMath.hypot(1, tan);
		return new Quaternion(w * tan, v.multiply(w / vectorMagnitude));
	}

	/**
     * The natural log of q mutator:
     */
	public static Quaternion ln(Quaternion q)
	{		
		double s = q.getW();
		double v = q.getV().getAbs(); 
		double m = StrictMath.hypot(s,v);
		return new Quaternion( 
				     StrictMath.log(m),
		             (v==0)
				      ? Vector3.ZERO
				      : q.getV().multiply(StrictMath.scalb(StrictMath.acos(s/m),1)/v)
			       );
	}

	/**
	 * Hamiltonian Quaternion Product static factory.
	 * @param Quaternion left.
	 * @param Quaternion right.
	 */
	public static Quaternion multiply(Quaternion left, Quaternion right) {		
		return new Quaternion(
				left.getW()*right.getW() - left.getX()*right.getX() - left.getY()*right.getY() - left.getZ()*right.getZ(),
				left.getX()*right.getW() + left.getW()*right.getX() - left.getZ()*right.getY() + left.getY()*right.getZ(),
				left.getY()*right.getW() + left.getZ()*right.getX() + left.getW()*right.getY() - left.getX()*right.getZ(),
				left.getZ()*right.getW() - left.getY()*right.getX() + left.getX()*right.getY() + left.getW()*right.getZ() );
								
//		//Alternate outer-product accumulations:	

//		return new Quaternion(right).multiply(left.getW()).addLeftI(left.getX(), right)
//				.addLeftJ(left.getY(), right).addLeftK(left.getZ(), right);
		
//		return new Quaternion(left).multiply(right.getW()).addRightI(right.getX(), left)
//				.addRightJ(right.getY(), left).addRightK(right.getZ(), left);
		
//		// factored solution: fewer multiplies and uses more local storage.
//		double e = (left.getX() + left.getZ()) * (right.getX() + right.getY());
//		double f = (left.getX() - left.getZ()) * (right.getX() - right.getY());
//		double g = (left.getW() + left.getY()) * (right.getW() - right.getZ());
//		double h = (left.getW() - left.getY()) * (right.getW() + right.getZ());	
//		double epf = e+f;
//		double gph = g+h;
//		double emf = e-f;
//		double gmh = g-h;
//		return new Quaternion(
//				(left.getZ() - left.getY()) * (right.getY() - right.getZ()) + StrictMath.scalb((gph - epf), -1)
//				, (left.getW() + left.getX()) * (right.getW() + right.getX()) - StrictMath.scalb((gph + epf), -1)
//				, (left.getW() - left.getX()) * (right.getY() + right.getZ()) + StrictMath.scalb((emf + gmh), -1)
//				, (left.getY() + left.getZ()) * (right.getW() - right.getX()) + StrictMath.scalb((emf - gmh), -1)
//		);
		

	}

	/**
	 * Hamiltonian Quaternion Product static factory.
	 * @param Quaternion q (left).
	 * @param Vector3 v (right).
	 */
	public static Quaternion multiply(Quaternion q, Vector3 v) {
		return new Quaternion(
				-(q.getX() * v.getX() + q.getY() * v.getY() + q.getZ()
						* v.getZ()), q.getW() * v.getX() - q.getZ() * v.getY()
						+ q.getY() * v.getZ(), q.getZ() * v.getX() + q.getW()
						* v.getY() - q.getX() * v.getZ(), -q.getY() * v.getX()
						+ q.getX() * v.getY() + q.getW() * v.getZ());
		
//		// factored solution: fewer multiplies and uses local storage.
//		double e = (q.getX() + q.getZ()) * (v.getX() + v.getY());
//		double f = (q.getX() - q.getZ()) * (v.getX() - v.getY());
//		double g = (q.getW() + q.getY()) * v.getZ();
//		double h = (q.getW() - q.getY()) * v.getZ();
//		double epf = e + f;
//		double gph = g + h;
//		double emf = e - f;
//		double gmh = g - h;
//		return new Quaternion((q.getZ() - q.getY()) * (v.getY() - v.getZ())
//				+ StrictMath.scalb(-(epf + gmh), -1), (q.getW() + q.getX())
//				* v.getX() - StrictMath.scalb(epf - gmh, -1),
//				(q.getW() - q.getX()) * (v.getY() + v.getZ())
//						+ StrictMath.scalb(emf - gph, -1),
//				(q.getY() + q.getZ()) * (-v.getX())
//						+ StrictMath.scalb(emf + gph, -1));
		

	}
	
	/**
	 * Hamiltonian Quaternion Product static factory.
	 * @param Vector3 v.
	 * @param Quaternion q.
	 */
	public static Quaternion multiply(Vector3 v, Quaternion q) {
		return new Quaternion(
				-(v.getX() * q.getX() + v.getY() * q.getY() + v.getZ()
						* q.getZ()), v.getX() * q.getW() - v.getZ() * q.getY()
						+ v.getY() * q.getZ(), v.getY() * q.getW() + v.getZ()
						* q.getX() - v.getX() * q.getZ(), v.getZ() * q.getW()
						- v.getY() * q.getX() + v.getX() * q.getY());
		
//		// factored solution: fewer multiplies and uses local storage.
//
//		double e = (v.getX() + v.getZ()) * (q.getX() + q.getY());
//		double f = (v.getX() - v.getZ()) * (q.getX() - q.getY());
//		double g = (v.getY()) * (q.getW() - q.getZ());
//		double h = v.getY() * (q.getW() + q.getZ());
//		double epf = e + f;
//		double gph = g + h;
//		double emf = e - f;
//		double gmh = g - h;
//		return new Quaternion((v.getZ() - v.getY()) * (q.getY() - q.getZ())
//				+ StrictMath.scalb((-epf + gmh), -1),
//				v.getX() * (q.getW() + q.getX())
//						- StrictMath.scalb((epf + gmh), -1), (-v.getX())
//						* (q.getY() + q.getZ())
//						+ StrictMath.scalb((emf + gph), -1),
//				(v.getY() + v.getZ()) * (q.getW() - q.getX())
//						+ StrictMath.scalb((emf - gph), -1));
	}

//	/** Angle-about-frame-axis rotation factory: 
//	 * <p> [ {Right|Left} multiply( exp(theta*{I|J|K}) ) ]
//	 * @param theta Principle angle (halved per factor multiplication) 
//	 * @param axis UnitBasis  {.I |.J |.K } fixed in rotation
//	 * @return FactoredQuaternion*/
//	public static Operator exp(Principle theta, BasisUnit axis) {
//		switch (axis.index()) {
//		case 1:
//			return exp_i(theta);
//		case 2:
//			return exp_j(theta);
//		case 3:
//			return exp_k(theta);
//		default:
//			return new Operator(Quaternion.IDENTITY);
//		}
//	}
	
	/** save few unnecessary operations for vector rotation. */
	static Vector3 multiplyAndGetImaginaryPart(Quaternion left,
			Vector3 right) {
		// factored solution: fewer multiplies and uses local storage.
		double a = (left.getW() + left.getX()) * right.getX();
		double c = (left.getW() - left.getX()) * (right.getY() + right.getZ());
		double d = (left.getY() + left.getZ()) * ( - right.getX());
		double e = (left.getX() + left.getZ()) * (right.getX() + right.getY());
		double f = (left.getX() - left.getZ()) * (right.getX() - right.getY());
		double g = (left.getW() + left.getY()) * ( - right.getZ());
		double h = (left.getW() - left.getY()) * right.getZ();
		return new Vector3(a - StrictMath.scalb((e + f + g + h), -1), c
				+ StrictMath.scalb((e - f + g - h), -1), d
				+ StrictMath.scalb((e - f - g + h), -1));
	}
	
	/**
	 * Reciprocal Quaternion static factory.
	 */
	public static Quaternion reciprocal(Quaternion p) {
		// Invert the rotation. Normalize product magnitude with the Quaternion
		// norm.
		// conjugate().divide(norm());
		double negNormP = -1 / p.getDeterminant();
		if (negNormP == 0 || Double.isNaN(negNormP)) {
			return new Quaternion(Quaternion.EMPTY);
		}
		return new Quaternion(-p.getW() * negNormP, p.getX() * negNormP, p.getY()
				* negNormP, p.getZ() * negNormP);
	}
	
	
	

//	/** Right multiply left Quaternion factor by UNIT_I and scalar s. Product static factory. */
//	protected static Quaternion rightIMultiply(double scalar, Quaternion left) {
//		return new Quaternion(-scalar * left.getX(), scalar * left.getW(), scalar * left.getZ(), -scalar
//				* left.getY());
//	}
//	
//	/** Right multiply left Quaternion factor by UNIT_J and scalar. Product static factory. */
//	protected static Quaternion rightJMultiply(double scalar, Quaternion left) {
//		return new Quaternion(-scalar * left.getY(), -scalar * left.getZ(), scalar * left.getW(), scalar
//				* left.getX());
//	}
//
//	/** Right multiply left Quaternion factor by UNIT_K and scalar. Product static factory. */
//	protected static Quaternion rightKMultiply(double scalar, Quaternion left) {
//		return new Quaternion(-scalar * left.getZ(), scalar * left.getY(), -scalar * left.getX(), scalar
//				* left.getW());
//	}
	

//	/** Left multiply right Quaternion factor by UNIT_I and scalar. Product static factory. */
//	protected static Quaternion leftIMultiply(double scalar, Quaternion right) {
//		return new Quaternion(-scalar * right.getX(), scalar * right.getW(), -scalar * right.getZ(), scalar
//				* right.getY());
//	}
//
//	/** Left multiply right Quaternion factor by UNIT_J and scalar. Product static factory. */
//	protected static Quaternion leftJMultiply(double scalar, Quaternion right) {
//		return new Quaternion(-scalar * right.getY(), scalar * right.getZ(), scalar * right.getW(), -scalar
//				* right.getX());
//	}
//
//	/** Left multiply right Quaternion factor by UNIT_K and scalar. Product static factory. */
//	protected static Quaternion leftKMultiply(double scalar, Quaternion right) {
//		return new Quaternion(-scalar * right.getZ(), -scalar * right.getY(), scalar * right.getX(), scalar
//				* right.getW());
//	}
	
//	/** Right multiply left Quaternion factor by UNIT_I. Product static factory. */
//	protected static Quaternion rightI(Quaternion left) {
//		return new Quaternion(-left.getX(), left.getW(), left.getZ(), -left.getY());
//	}
//
//	/** Right multiply left Quaternion factor by UNIT_J. Product static factory. */
//	protected static Quaternion rightJ(Quaternion left) {
//		return new Quaternion(-left.getY(), -left.getZ(), left.getW(), left.getX());
//	}
//
//	/** Right multiply left Quaternion factor by UNIT_K. Product static factory. */
//	protected static Quaternion rightK(Quaternion left) {
//		return new Quaternion(-left.getZ(), left.getY(), -left.getX(), left.getW());
//	}
//
//	/** Left multiply right Quaternion factor by Unit-I. Product static factory. */
//	protected static Quaternion leftI(Quaternion right) {
//		return new Quaternion(-right.getX(), right.getW(), -right.getZ(), right.getY());
//	}
//
//	/** Left multiply right Quaternion factor by Unit-J. Product static factory. */
//	protected static Quaternion leftJ(Quaternion right) {
//		return new Quaternion(-right.getY(), right.getZ(), right.getW(), -right.getX());
//	}
//
//	/** Left multiply right Quaternion factor by Unit-K. Product static factory. */
//	protected static Quaternion leftK(Quaternion right) {
//		return new Quaternion(-right.getZ(), -right.getY(), right.getX(), right.getW());
//	}

	/**
	 * Rotation operator factory: Princple angle of theta about vector
	 * 
	 * @return FactoredQuaternion
	 */
	public static final Rotator rotate(CodedPhase theta, Vector3 v) {
		double m = v.getAbs();
		if (m == 0) {
			return new Rotator(Quaternion.IDENTITY);
		}
		if (theta.isAcute()) {
			return new Rotator(1.d, v.multiply(theta.tanHalf() / m));
		}
		return new Rotator(theta.cotHalf(), v.divide(m));
	}

	/**
	 * Rotation operator factory: Angle of vector magnitude about vector
	 * 
	 * @return FactoredQuaternion
	 */
	public static final Rotator rotate(Vector3 v) {
		double m = v.getAbs();
		if (m == 0) {
			return new Rotator(Quaternion.IDENTITY);
		}
		double t = StrictMath.tan(m);
//		double t = StrictMath.tan(StrictMath.IEEEremainder(m, StrictMath.PI));
		if (StrictMath.abs(t) > 1) {
			return new Rotator(1.d / t, v.divide(m));
		}
		return new Rotator(1.d, v.multiply(t / m));
	}	

	
	/** 
	 * I-axis rotation factory: (halved per factor multiplication)<p>
	 * [ exp(I*theta/2) ]
	 * @return factoredQuaternion Operator
	 * @param theta Principle angle (right-handed rotation sense)  
	 */
	public static final Rotator rotate_i(CodedPhase theta) {
		if (theta.isAcute()) {
			return new Rotator(1d, theta.tanHalf(), 0, 0);
		}
		return new Rotator(theta.cotHalf(), 1d, 0, 0);		
	}
	
//	/** 
//	 * Mutator:<p>
//	 * @param p 
//	 * @return this.set(new Quaternion(p).rightMultiply(Quaternion.UNIT_I));
//	 * */
//	public static final Quaternion rightMultiplyI(final Quaternion p) {
//		return new Quaternion(-p.getX(), p.getW(), p.getZ(), -p.getY());
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param p 
//	 * @return this.set(new Quaternion(p).rightMultiply(Quaternion.UNIT_J));
//	 * */
//	public static final Quaternion rightMultiplyJ(final  Quaternion p) {
//		return new Quaternion(-p.getY(), -p.getZ(), p.getW(), p.getX());
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param p 
//	 * @return this.set(new Quaternion(p).rightMultiply(Quaternion.UNIT_K));
//	 * */
//	public static final Quaternion rightMultiplyK(final Quaternion p) {
//		return new Quaternion(-p.getZ(), p.getY(), -p.getX(), p.getW());
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param q 
//	 * @return this.set(new Quaternion(q).leftMultiply(Quaternion.UNIT_I));
//	 * */
//	public static final Quaternion leftMultiplyI(final Quaternion q) {
//		return new Quaternion(-q.getX(), q.getW(), -q.getZ(), q.getY());
//	}
//
//	/** 
//	 * Mutator:<p>
//	 * @param q 
//	 * @return this.set(new Quaternion(q).leftMultiply(Quaternion.UNIT_J));
//	 * */
//	public static final Quaternion leftMultiplyJ(final Quaternion q) {
//		return new Quaternion(-q.getY(), q.getZ(), q.getW(), -q.getX());
//	}
//
//	/** 
//	 * Mutator: Must not use q self-referent.<p>
//	 * @param q 
//	 * @return this.set(new Quaternion(q).leftMultiply(Quaternion.UNIT_K));
//	 * */
//	public static final Quaternion leftMultiplyK(final Quaternion q) {
//		return new Quaternion(-q.getZ(), -q.getY(), q.getX(), q.getW());
//	}
//

	
	
	
	
	/** 
	 * J-axis rotation factory: (halved per factor multiplication)<p>
	 * [  exp(J*theta) ]
	 * @return FactoredQuaternion operator
	 * @param theta Principle angle (right-handed rotation sense)  */
	public static final Rotator rotate_j(CodedPhase theta) {
		if (theta.isAcute()) {
			return new Rotator(1d, 0, theta.tanHalf(), 0);
		}
		return new Rotator(theta.cotHalf(), 0, 1d, 0);		
	}
	
	/** 
	 * K-axis rotation operator factory: (halved per factor multiplication)<p>
	 * [  exp(K*theta) ]
	 * @return FactoredQuaternion
	 * @param theta Principle angle (right-handed rotation sense)  */
	public static final Rotator rotate_k(CodedPhase theta) {
		if (theta.isAcute()) {
			return new Rotator(1d, 0, 0, theta.tanHalf());
		}
		return new Rotator(theta.cotHalf(), 0, 0, 1d);
		
	}
	
	public static Rotator slerp(Quaternion q0,Quaternion q1, double t )  {
		return new Rotator( multiply( multiply(q1,reciprocal(q0)).power(t) , q0 ));
	}	

//	/**
//	 * Elementary Right Product -- static factory.
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) 
//	 * @param Quaternion left factor.
//	 * @param zBasisUnit { .S | .I | .J | .K } right hand factor.
//	 */
//	public static Quaternion multiply(Quaternion left, BasisUnit right) {
//		switch (right.index()) {
//		case 1:
//			return rightMultiply_i(left);
//		case 2:
//			return rightMultiply_j(left);
//		case 3:
//			return rightMultiply_k(left);
//		default:
//			return new Quaternion(left);
//		}
//	}	
	
	
//	/**
//	 * Elementary Left Product -- static factory.
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) 
//	 * @param zBasisUnit { .S | .I | .J | .K } left hand factor.
//	 * @param Quaternion right factor.
//	 */
//	public static Quaternion multiply(BasisUnit left, Quaternion right) {
//		switch (left.index()) {
//		case 1:
//			return leftMultiply_i(right);
//		case 2:
//			return leftMultiply_j(right);
//		case 3:
//			return leftMultiply_k(right);
//		default:
//			return new Quaternion(right);
//		}
//	}	
	

	//public static functions:
	public static final Rotator tilt_i(Vector3 direction) {
		//I==0...
		double w = direction.getAbs() + direction.getX();
		if (w > direction.getY()) {
			if (w > direction.getZ()) {
				//I
				return new Rotator(1, 0, direction.getZ() / w, -direction.getY() / w);
			}
		} else {
			if (direction.getY() > direction.getZ()) {
				//J
				return new Rotator(-w / direction.getY(), 0, -direction.getZ() / direction.getY(), 1);
			}
		}
		//K
		return new Rotator(w / direction.getZ(), 0, 1, -direction.getY() / direction.getZ());
	}

	public static final Rotator tilt_j(Vector3 direction) {
		//J==0...
		double w = direction.getAbs() + direction.getY();
		if (w > direction.getX()) {
			if (w > direction.getZ()) {
				//J
				return new Rotator(1, -direction.getZ() / w, 0, direction.getX() / w);
			}

		} else {
			if (direction.getX() > direction.getZ()) {
				//I
				return new Rotator(w / direction.getX(), -direction.getZ() / direction.getX(), 0, 1);
			}
		}
		//K
		return new Rotator(-w / direction.getZ(), 1, 0, -direction.getX() / direction.getZ());
	}

	public static final Rotator tilt_k(Vector3 direction) {
		//K==0...
		double w = direction.getAbs() + direction.getZ();
		if (w > direction.getX()) {
			if (w > direction.getY()) {
				//K
				return new Rotator(1, direction.getY() / w, -direction.getX() / w, 0);
			}

		} else {
			if (direction.getY() > direction.getX()) {
				//J
				return new Rotator(w / direction.getY(), 1, -direction.getX() / direction.getY(), 0);
			}
		}
		//I
		return new Rotator(-w / direction.getX(), -direction.getY() / direction.getX(), 1 , 0 );
	}	

	/** Quaternion versor (Unit-sized) static factory.	 */
	public static Quaternion unit(Quaternion p) {
		return new Quaternion(p).divide(p.getAbs());
	}

//	/** save few unnecessary operations for vector rotation. */
//	private static Vector3 multiplyAndGetImaginaryPart(Quaternion left,
//			Quaternion right) {
//		// factored solution: fewer multiplies and uses local storage.
//		double a = (left.l() + left.i()) * (right.l() + right.i());
//		double c = (left.l() - left.i()) * (right.j() + right.k());
//		double d = (left.j() + left.k()) * (right.l() - right.i());
//		double e = (left.i() + left.k()) * (right.i() + right.j());
//		double f = (left.i() - left.k()) * (right.i() - right.j());
//		double g = (left.l() + left.j()) * (right.l() - right.k());
//		double h = (left.l() - left.j()) * (right.l() + right.k());
//		return new Vector3(a - StrictMath.scalb((e + f + g + h), -1), c
//				+ StrictMath.scalb((e - f + g - h), -1), d
//				+ StrictMath.scalb((e - f - g + h), -1));
//	}


// /** Hamiltonian (Right multiply Unit-Basis S) Product (copy) static
// factory. */
// public static Quaternion leftS(Quaternion p){ return new
// Quaternion(p.lcalar(),p.i(),p.j(),p.k()); }

//public static functions:

//		/** Hamiltonian (Right multiply Unit-Basis-S) Product (copy) static factory.  */
//		public static Quaternion rightS(Quaternion p){ return new Quaternion(p.lcalar(),p.i(),p.j(),p.k()); }
	

//	/** Hamiltonian (Right multiply Unit-Basis-S) Product (copy) static factory.  */
//	public static Quaternion rightS(Quaternion p){ return new Quaternion(p.lcalar(),p.i(),p.j(),p.k()); }
	
	// public static functions:

//	/** Quaternion operator from Tait-Bryan | Centerline Euler Angle static factory.	 
//	 * @param pK  (k-axis) Angle #1: yaw | az
//	 * @param pJ  (j-axis) Angle #2: pitch | el
//	 * @param pI  (i-axis) Angle #3: roll | twist
//	 * @return Quaternion body | centerline frame orientation */
//	public static Operator eulerRotate_kji(Angle pK, Angle pJ, Angle pI)
//	{	
//			double ys = StrictMath.sin( pK.getRadians()/2.0 );
//			double yc = StrictMath.cos( pK.getRadians()/2.0 );
//			double ps = StrictMath.sin( pJ.getRadians()/2.0 );
//			double pc = StrictMath.cos( pJ.getRadians()/2.0 );
//			double rs = StrictMath.sin( pI.getRadians()/2.0 );
//			double rc = StrictMath.cos( pI.getRadians()/2.0 );
//			return (Operator) new Quaternion( 			
//					rc*pc*yc + rs*ps*ys,
//					rs*pc*yc - rc*ps*ys,
//					rc*ps*yc + rs*pc*ys,
//					rc*pc*ys - rs*ps*yc
//		    );	
//	}

//	/** Quaternion from Partial Tait-Bryan | Centerline Euler Angle static factory.	 
//	 * @param pK  (k-axis) Angle #1: az | dh | yaw yaw 
//	 * @param pJ  (j-axis) Angle #2: el | dv | pitch
//	 * @return Quaternion body | centerline frame orientation */
//	public static Operator eulerRotate_kj(Angle pK, Angle pJ)
//	{	
//	    //z[K]
//		double pc = StrictMath.cos( pJ.getRadians()/2.0 );
//		double ps = StrictMath.sin( pJ.getRadians()/2.0 );
//		
//		//z[J]
//		double yc = StrictMath.cos( pK.getRadians()/2.0 );
//		double ys = StrictMath.sin( pK.getRadians()/2.0 );
//	
//		return (Operator) new Quaternion( 			
//					 pc*yc,
//					-ps*ys,
//					 ps*yc,
//					 pc*ys
//		);	
//	}

//	/** Vector3 basis unit-I (image mapped by rotation operator q) static factory  */
//	public static Vector3 image_i(Operator t) { 
////		return QuaternionMath.multiplyGetV(  //project I-axis unit (Op.Right)
////				new Quaternion(-p.i(),p.l(),p.k(),-p.j()),
////				new Quaternion(p.l(),-p.i(),-p.j(),-p.k()) 
////		)//.divide(p.determinant())  
////		;
//		double a = t.l() - t.i();
//		double b = t.l() + t.i();
//		double e = (t.j() - t.l()) * (t.i() + t.j());
//		double f = (t.l() + t.j()) * (t.i() - t.j());
//		double g = (t.k() - t.i()) * (t.l() + t.k());
//		double h = (t.i() + t.k()) * (t.k() - t.l());
//		return new Vector3(a * a - StrictMath.scalb((e - f + g + h), -1), b
//				* (t.j() + t.k()) + StrictMath.scalb((e + f + g - h), -1), b
//				* (t.k() - t.j()) + StrictMath.scalb((e + f - g + h), -1))
//				.divide(t.determinant());
//	}
//	
//	/** Vector3 Projected unit-J (Basis transformed by rotation operator q) static factory  */
//	public static Vector3 image_j(Operator t) { 
////		return QuaternionMath.multiplyGetV(  //project J-axis unit (Op.Right)
////              new Quaternion(-p.j(),-p.k(),p.l(),p.i()),
////				new Quaternion(p.l(),-p.i(),-p.j(),-p.k()) 
////		)//.divide(p.determinant())  
////		;
//		double a = (t.l() + t.i());
//		double b = (t.j() + t.k());
//		double e = (t.k() - t.i()) * (t.i() + t.j());
//		double f = (t.k() + t.i()) * (t.j() - t.i());
//		double g = (t.l() - t.j()) * (t.l() + t.k());
//		double h = (t.j() + t.l()) * (t.k() - t.l());
//		return new Vector3(b * (t.i() - t.l())
//				- StrictMath.scalb((e - f + g + h), -1), b * (t.j() - t.k())
//				+ StrictMath.scalb((e + f + g - h), -1), a * a
//				+ StrictMath.scalb((e + f - g + h), -1))
//				.divide(t.determinant());
//	}
//	/** Vector3 Projected unit-K (Basis transformed by rotation operator q) static factory  */
//	public static Vector3 image_k(Operator t) { 
////		return QuaternionMath.multiplyGetV(  //project K-axis unit (Op.Right)
////              new Quaternion(-p.k(),p.j(),-p.i(),p.l())
////				new Quaternion(p.l(),-p.i(),-p.j(),-p.k()) 
////		)//.divide(p.determinant())  
////		;
//		double a = (t.k()+t.j());
//		double b = (t.l()-t.i());
//		double e = -(t.j()+t.l())*(t.i()+t.j());
//		double f = (t.j()-t.l())*(t.i()-t.j());
//		double g = -(t.k()+t.i())*(t.l()+t.k());
//		double h = (t.i()-t.k())*(t.l()-t.k());
//		return new Vector3(b * (t.j() - t.k())
//				- StrictMath.scalb((e - f + g + h), -1), a * a
//				+ StrictMath.scalb((e + f + g - h), -1), b * (t.l() + t.i())
//				+ StrictMath.scalb((e + f - g + h), -1))
//				.divide(t.determinant());
//	}
//	
//	/** Vector3 basis unit-I (pre-image mapped by rotation operator q) static factory  */
//	public static Vector3 preImage_i(Operator q) { 
//
////		return QuaternionMath.multiplyGetV(  //project I-axis unit (Op.Right)
////		        new Quaternion(p.l(),-p.i(),-p.j(),-p.k()) 
////              new Quaternion(-p.i(),p.l(),-p.k(),p.j()); }
////		)//.divide(p.determinant())  
////		;
//		double a = q.l() - q.i();
//		double b = q.l() + q.i();
//		double e = (q.i() + q.k()) * (-q.l() + q.k());
//		double f = (q.i() - q.k()) * (q.l() + q.k());
//		double g = (q.j() - q.l()) * (q.i() + q.j());
//		double h = (q.l() + q.j()) * (-q.i() + q.j());
//		return new Vector3(a * a - StrictMath.scalb((e - f + g + h), -1), b
//				* (q.j() - q.k()) + StrictMath.scalb((e + f + g - h), -1), b
//				* (q.j() + q.k()) + StrictMath.scalb((e + f - g + h), -1))
//				.divide(q.determinant());
//	}
//	
//	/** Vector3 basis unit-J (pre-image mapped by rotation operator q) static factory  */
//	public static Vector3 preImage_j(Operator q) { 
//
////		return QuaternionMath.multiplyGetV(  //project I-axis unit (Op.Right)
////		        new Quaternion(p.l(),-p.i(),-p.j(),-p.k()) 
////              new Quaternion(-p.j(),p.k(),p.l(),-p.i()); }
////		)//.divide(p.determinant())  
////		;
//		double a = q.j() + q.k();
//		double b = q.l() - q.i();
//		double e = -(q.i() + q.k()) * (q.k() + q.l());
//		double f = (q.i() - q.k()) * (q.k() - q.l());
//		double g = (q.l() - q.j()) * (-q.j() + q.i());
//		double h = (q.l() + q.j()) * (-q.j() - q.i());
//		return new Vector3(b * (q.k() - q.j())
//				- StrictMath.scalb((e - f + g + h), -1), b * (q.l() + q.i())
//				+ StrictMath.scalb((e + f + g - h), -1), a * a
//				+ StrictMath.scalb((e + f - g + h), -1))
//				.divide(q.determinant());
//	}
//	
//	/** Vector3 basis unit-K (pre-image mapped by rotation operator q) static factory  */
//	public static Vector3 preImage_k(Operator q) { 
////		return QuaternionMath.multiplyGetV(  //project I-axis unit (Op.Right)
////		        new Quaternion(p.l(),-p.i(),-p.j(),-p.k()) 
////              new Quaternion(-p.k(),-p.j(),p.i(),p.l()); }
////		)//.divide(p.determinant())  
////		;
//		double a = q.l() + q.i();
//		double b = q.k() + q.j();
//		double e = (q.i() + q.k()) * (q.j() - q.i());
//		double f = (q.k() - q.i()) * (q.j() + q.i());
//		double g = (q.j() - q.l()) * (q.k() + q.l());
//		double h = (q.l() + q.j()) * (-q.k() + q.l());
//		return new Vector3(b * (-q.l() + q.i())
//				- StrictMath.scalb((e + f + g + h), -1), a * a
//				+ StrictMath.scalb((e - f + g - h), -1), b * (q.k() - q.j())
//				+ StrictMath.scalb((e - f - g + h), -1))
//				.divide(q.determinant());
//	}
//	
//	public static Quaternion image(Quaternion object,Operator operator) {
//		return multiply(multiply(operator,object),reciprocal(operator));	
//	}
//	
//	public static Quaternion preImage(Quaternion object,Operator operator)  {
//		return multiply(multiply(reciprocal(operator),object),operator);
//	}


//	/**
//	 * Tait-Bryan Yaw | Centerline azimuth :
//	 * 
//	 * @return Angle [0..2*PI)
//	 */
//	public static Angle euler_k_kji(Operator q) {
//		double part = q.j() * q.j() + q.k() * q.k();
//		double y = StrictMath
//				.atan2(q.l() * q.k() + q.i() * q.j(),
//						StrictMath.scalb(q.l() * q.l() + q.i() * q.i()
//								+ part, -1)
//								- part);
//		return Angle.inRadians((y >= 0) ? y : Angle.REVOLUTION_RADIANS + y);
//
//	}
//	
//	/** Tait-Bryan Pitch | Centerline elevation : 
//	 * @return Angle   [-PI/2..PI/2] */
//	public static Angle euler_j_kji(Operator q) {
//		return Angle.inRadians(StrictMath.asin((q.l() * q.j() - q.k()
//				* q.i())
//				/ StrictMath.scalb(q.determinant(), -1)));
//	}
//	
//	/** Tait-Bryan Roll | Centerline twist : 
//	 * @return Angle  (-PI..PI] */
//	public static Angle euler_i_kji(Operator q) {
//		return Angle.inRadians(StrictMath.atan2(
//				q.l() * q.i() + q.j() * q.k(),
//				StrictMath.scalb(q.l() * q.l() - q.i() * q.i() - q.j()
//						* q.j() + q.k() * q.k(), -1)));
//	}
		
//	public static Operator right(Quaternion q, UnitBasis axis)
//	{
//		switch(axis.index()) {
//		case 1:  return (Operator) rightMultiply_i(q);		
//		case 2:  return (Operator) rightMultiply_j(q);		
//		case 3:  return (Operator) rightMultiply_k(q);		
//		default: return (Operator) new Quaternion(q); }
//	}	

//	public static Operator left(UnitBasis axis, Quaternion q)
//	{
//		switch(axis.index()) {
//		case 1:  return (Operator) leftMultiply_i(q);		
//		case 2:  return (Operator) leftMultiply_j(q);		
//		case 3:  return (Operator) leftMultiply_k(q);		
//		default: return (Operator) new Quaternion(q); }
//	}	
//
//	public static Operator left(double w, UnitBasis axis, Quaternion q)
//	{
//		switch(axis.index()) {
//		case 1:  return (Operator) leftMultiply_i(w,q);		
//		case 2:  return (Operator) leftMultiply_j(w,q);		
//		case 3:  return (Operator) leftMultiply_k(w,q);		
//		default: return (Operator) new Quaternion(q).multiply(w); }
//	}	
//	/** Scale and Right multiply Unit-Basis I -- Product static factory. */
//	protected static Quaternion rightMultiply_i(double w, Quaternion q) {
//		return new Quaternion(-w * q.i(), w * q.l(), w * q.k(), -w * q.j());
//	}
//	/**
//	 * Scale and Right multiply Unit-Basis J -- Product static factory.
//	 */
//	protected static Quaternion rightMultiply_j(double w, Quaternion q) {
//		return new Quaternion(-w * q.j(), -w * q.k(), w * q.l(), w * q.i());
//	}
//	/**
//	 * Scale and Right multiply Unit-Basis K) Product static factory.
//	 */
//	protected static Quaternion rightMultiply_k(double w, Quaternion q) {
//		return new Quaternion(-w * q.k(), w * q.j(), -w * q.i(), w * q.l());
//	}
//	/**
//	 * (Scale and Left multiply Unit-Basis I) Product static factory.
//	 */
//	protected static Quaternion leftMultiply_i(double w, Quaternion q) {
//		return new Quaternion(-w * q.i(), w * q.l(), -w * q.k(), w * q.j());
//	}
//	/**
//	 * (Scale and Left multiply Unit-Basis J) Product static factory.
//	 */
//	protected static Quaternion leftMultiply_j(double w, Quaternion q) {
//		return new Quaternion(-w * q.j(), w * q.k(), w * q.l(), -w * q.i());
//	}
//	/**
//	 * (Scale and Left multiply Unit-Basis K) Product static factory.
//	 */
//	protected static Quaternion leftMultiply_k(double w, Quaternion q) {
//		return new Quaternion(-w * q.k(), -w * q.j(), w * q.i(), w * q.l());
//	}

	
	
	
//	/**
//	 * <B>Quaternion and unit-basis product ("flip") static factory.</B>
//	 * (90 degrees per multiplication)
//	 * @param Quaternion factor
//	 * @param UnitBasis {.S |.I |.J |.K } unit basis factor
//	 * @param HandRule {.RIGHT |.LEFT } hand
//	 */
//	public static Quaternion flip(Quaternion factor, UnitBasis axis, HandRule hand)
//	{
//		return (hand.isRight()) 
//				? QuaternionMath.multiply(factor, axis) 
//				: QuaternionMath.multiply(axis, factor) ;
//	}
	
	
//	/**
//	 * Scaled by w Hamiltonian Elementary Right Product static factory.
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) 
//	 * @param Quaternion left factor.
//	 * @param UnitBasis { .S | .I | .J | .K } right hand factor.
//	 */
//	public static Quaternion multiply(double w, Quaternion left, UnitBasis right)
//	{
//		switch(right.index()) {
//			case 1:  return rightMultiply_i(w,left);		
//			case 2:  return rightMultiply_j(w,left);		
//			case 3:  return rightMultiply_k(w,left);		
//			default: return new Quaternion(left).multiply(w); 
//		}
//	}	


	
//	/**
//	 * Scaled by w Hamiltonian Elementary Left Product static factory.
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) 
//	 * @param UnitBasis { .S | .I | .J | .K } left hand factor.
//	 * @param Quaternion right factor.
//	 */
//	public static Quaternion multiply( double w, UnitBasis left, Quaternion right ) {
//		switch(left.index()) {
//			case 1:  return leftMultiply_i(w, right);		
//			case 2:  return leftMultiply_j(w, right);		
//			case 3:  return leftMultiply_k(w, right);		
//			default: return new Quaternion(right).multiply(w);
//		}
//	}
	
	
//	public static Vector3 fastImage(UnitBasis frameAxis, Operator t){
//		return multiplyGetV(((Quaternion) (multiply(t, frameAxis))),
//				conjugate(t)).divide(t.determinant());
//	}
	
//	public static Quaternion projection(UnitBasis projected, double length, Quaternion operator){
//		return multiply(QuaternionMath.multiply(operator,projected),conjugate(operator))
//	           .multiply(length/operator.determinant());
//	}
	
	
//	/** Rotation operator factory: Angle-about-vector, of 
//	 * angle magnitude radians.
//	 * <p> [ {Right|Left} multiply( exp(theta*{I|J|K}) ) ]
//	 * @return FactoredQuaternion
//	 * @param angle - Principle angle (halved per factor multiplication) 
//	 * @param v Vector3 - dirction cosines of axis fixed in rotation
//	 * @param sense TODO
//	 */
//	public static Operator exp(Principle angle, Vector3 v)
//	{	
//		assert(false);
//		double m = v.abs();          		// vector magnitude...
//		if (m == 0) return new Operator( Quaternion.IDENTITY );	//return scalar Op
//		double t = angle.tanHalf();       // tan(v%PI)
//		if (StrictMath.abs(t)>1) {
//			return new Operator(1.d/t, v.divide(m) );		    	
//		} else { 
//			return new Operator(1.d, v.multiply(t/m) );		    	
//		}		
//	}



//	public static Vector3 fastPreImage(UnitBasis frameAxis, Operator operator){
//		return multiplyGetV(
//				  conjugate(operator),
//				  multiply(frameAxis,operator)
//				).divide(operator.determinant())
//	           ;
//	}
		
	
	
//	public static Quaternion image(UnitBasis projected, double length, Quaternion operator){
//		return multiply(QuaternionMath.multiply(conjugate(operator),projected),operator )
//		       .multiply(length/operator.determinant());
//	}
	
	
//	/**
//	 * The exponential of q equals 
//	 * (e to the t cosine absolute value of V, 
//	 *  e to the t sine of the absolute value of V times V normalized to V ) 
//	 */
//	public static Quaternion exp(Quaternion q) 
//	{
//		FastQuaternion fq = FastQuaternion.exp(q);
//		return new Quaternion(fq.getQuaternionFactor().divide(StrictMath.sqrt(fq.getScalarFactor())));
//	
//	}

	
//	/** Rotation operator factory: Angle-about-vector, of 
//	 * vector magnitude radians.
//	 * <p> [ {Right|Left} multiply( exp(theta*{I|J|K}) ) ]
//	 * @return FactoredQuaternion
//	 * @param theta Principle angle (halved per factor multiplication) 
//	 * @param axis UnitBasis  {.I |.J |.K } fixed in rotation
//	 * @param sense HandRule {.RIGHT |.LEFT } multiplication (handedness)  */
//	public static Operator exp(Vector3 v) {
//		double m = v.abs(); // vector magnitude...
//		if (m == 0)
//			return new Operator(Quaternion.IDENTITY); // return scalar Op
//		double t = StrictMath.tan(m); // tan(v%PI)
//		if (StrictMath.abs(t) > 1) {
//			return new Operator(1.d / t, v.divide(m));
//		} else {
//			return new Operator(1.d, v.multiply(t / m));
//		}
//	}
//

	//public static functions:
	
	//cos(z1+z2)=Z1*z2,sin(z1+z2)=Z1 x Z2
	
	
//	/** angle,vector constructor. */
//	public static Operator exp(Angle measure, Vector3 v)
//	{
//		Principle theta = new Principle(measure);
//		double m = v.abs();          		// vector magnitude...
//		if (m == 0) return new Operator( Quaternion.IDENTITY );	//return scalar Op
//		return (theta.isAcute())
//		? new Operator( 1.d, v.multiply(theta.tanHalf()/m) )
//		: new Operator( 1.d/theta.tanHalf(), v.divide(m) );			
//		
////		return exp(new Principle(theta),v);
//
//		//		return new Quaternion( 			
////				StrictMath.cos( StrictMath.scalb(measure.getRadians(),-1) ),
////				aboutAxis.divide(aboutAxis.abs()*StrictMath.sin( StrictMath.scalb(measure.getRadians(),-1)) )
////		);
//		
//	}
	
	
	
//	/**
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) mutator.
//	 * @param Axis {.I |.J |.K } fixed
//	 * @param HandRule {.RIGHT |.LEFT } hand 
//	 */
//	public Quaternion flip( Axis fixed, HandRule hand ) {
//	// Multiplication by unit-vector element: 
//		return this.set((hand.isRight())? flip(this,fixed): flip(fixed,this)); 
//	}
	

	
//	public static Quaternion invert(Quaternion q) {
//	Quaternion qInv = new Quaternion(q);
//	qInv.reciprocal();
//	return qInv;
//}	
//public static Quaternion conjugate(Quaternion q) {
//	Quaternion qC = new Quaternion(q);
//	qC.conj();
//	return qC;
//}	
//
    
//  protected double radiansHalfAngle(double units, double unitsRevolution){
//	double tmp = (units%unitsRevolution)/unitsRevolution;
//	if (Math.abs(tmp)>.5) tmp -= Math.signum(units);
//	return tmp*Math.PI;
//}
	
//	public Quaternion rotate(double degrees, Axis element, Operator order){
////Fast Rotation operator -- and avoid manipulation if angle is zero...;
//if (degrees == 0) return this;
//
////[active] angle definition (Sign sense agrees with definition...)
//double tangent = Math.tan(radiansHalfAngle(degrees,360));
//
////handed factor operation
//Quaternion pStack = new Quaternion(this).flip(element, order);
//
//if ( Math.abs(tangent) < 1.0) {
//	pStack.multiply(tangent);
//}else {
//	this.divide(tangent);
//}
//return this.add(pStack);
//}
//

//  public double get(Axis vector){    
//	switch(vector.enumIndex) {
//	case 1:  // Axis.I.index():
//		return I;
//	case 2:  // Axis.J.index():
//		return J;
//	case 3:  // Axis.K.index():
//		return K;
//	default:
//		return S;
//	}
//}
//public void set(Axis vector, double d){    
//	switch(vector.enumIndex) {
//	case 1:  // Axis.I.index():
//		I=d;
//	case 2:  // Axis.J.index():
//		J=d;
//	case 3:  // Axis.K.index():
//		K=d;
//	default:
//		S=d;
//	}
//}
	
//Static Factories:	
	
	
	
//	protected Quaternion add_sQ(Quaternion q, double sS) {
//		return new Quaternion(this).add(q.multiply(sS));
//	}
//	protected Quaternion add_sQI(Quaternion q, double sI) {
//		return new Quaternion(-q.I, q.S, q.K, -q.J).multiply(sI).add(this);
//	}
//	protected Quaternion add_sQJ(Quaternion q, double sJ) {
//		return new Quaternion(-q.J, -q.K, q.S, q.I).multiply(sJ).add(this);
//	}
//	protected Quaternion add_sQK(Quaternion q, double sK) {
//		return new Quaternion(-q.K, q.J, -q.I, q.S).multiply(sK).add(this);
//	}
//	
//	protected static Quaternion multiplyI(double pI, Quaternion q) {
//		return new Quaternion(-pI*q.I, pI*q.S, -pI*q.K, pI*q.J);
//	}
	
//	protected static Quaternion multiplyI(Quaternion p, double qI) {
//		return new Quaternion(-p.I*qI, p.S*qI, p.K*qI, -p.J*qI);
//	}
	
//	protected static Quaternion multiplyJ(double pJ, Quaternion q) {
//		return new Quaternion(-pJ*q.J, pJ*q.K, pJ*q.S, -pJ*q.I);
//	}
	
//	protected static Quaternion multiplyJ(Quaternion p, double qJ) {
//		return new Quaternion(-p.J*qJ, -p.K*qJ, p.S*qJ, p.I*qJ);
//	}
	
//	protected static Quaternion multiplyK(double pK, Quaternion q) {
//		return new Quaternion(-pK*q.K, -pK*q.J, pK*q.I, pK*q.S);
//	}
	
//	protected static Quaternion multiplyK(Quaternion p, double qK) {
//		return new Quaternion(-p.K*qK, p.J*qK, -p.I*qK, p.S*qK);
//	}
	
	

//  private void setVector(double v[]) {
//	S = 0;
//	I = v[0];
//	J = v[1];
//	K = v[2];
//  return this;	
//}
//public double[] getVector() {
//  return  new double [] {this.I,this.J,this.K};	  
//}

	
	
	//p.setDegrees(xd);
	
	
//	System.out.println( "q.sign():  " +t.toString());

  //Static converters from primitive doubles:
	
//	//Vector index Incrementer:
//	protected static int inqr(int j){
//		return (j<3)? j+1 : 1;
//	}
//	
//	//Vector index Decrementer:
//	protected static int deqr(int j){
//		return (j>1)? j-1 : 3;
//	}
	//Constructors:
	
//	/**
//	 * Default constructor: Unit-scalar Quaternion.
//	 */
//	public Quaternion() {
//		q[0] = 1;
//		q[1] = 0;
//		q[2] = 0;
//		q[3] = 0;
//	}

//  public double[] getVector() {
//	double[] vector = new double[3];
//	vector[0] = q[1];
//	vector[1] = q[2];
//	vector[2] = q[3];
//	return vector;
//}
//
//public double i() {
//return q[1];
//}
//
//public double j() {
//return q[2];
//}
//
//public double k() {
//return q[3];
//}

// 	/**
//	 * Elementary unit-vector assignment.
//	 * 
//	 * @param h : Axis of elementary vector unit length.
//	 * @param h : Operator {RIGHT|LEFT} order for handedness -- left is negative.
//	 */
//	public Quaternion setFlip(Axis h, Operator side) {
//		q[0] = 0;
//		q[1] = 0;
//		q[2] = 0;
//		q[3] = 0;
//		q[h.index()] = side.order();
//		return this;
//	}
//	
//
	
//	/**
//	 * [Post] Quaternion multiplication.
//	 */
//	public Quaternion multiply(Quaternion right)
//	{
//		//return multiply(right, Operate.Post);
//		return multiplyQ(this, right );
//	}
//	
	
		
	

//	/**
//	 * Post-multiplication right-turns operator.
//	 * [Pre-multiplication left-turns operator.]
//	 * 
//	 * @param int
//	 *            m: index {1|2|3} of left Quaternion's unit-element factor.
//	 */
//	public Quaternion turnOp(Axis element){
//		
//		//right factor operation
////		Quaternion qStack = new Quaternion(this).flipOp(element);		
//		Quaternion qStack = new Quaternion(this).flipOp(element,Operate.Post);		
//		return add(qStack);
//	}
//
//	/**
//	 * [Post] Multiplication by elementary unit vector flips-right.
//	 * [Pre-Multiplication by elementary unit vector flips-left.]
//	 * 
//	 * @param Axis
//	 *            unit index {I,J,K} of elementary right Quaternion factor.
//	 */
//	public Quaternion flipOp(Axis fixed){
//			double tmp = q[0];
//			q[0] = -q[fixed.index()];
//			q[fixed.index()] = tmp;
//
//			tmp = q[fixed.deqr()];
//			q[fixed.deqr()] = -q[fixed.inqr()];
//			q[fixed.inqr()] = tmp;
//			return this;
//	}
//
//	
//	/**
//	 * [Right operation] Quaternion [active] rotation; yields rotated Quaternion operator.
//	 * 
//	 * @param double
//	 *            angle: factor element of sparse Quaternion.
//	 * @param int
//	 *            m: index {1|2|3} of left Quaternion's unit-element factor.
//	 */
//	public Quaternion rotateOp(double degrees,Axis element){
////	    Fast Rotation operator -- and avoid manipulation if angle is zero...;
//		if (degrees == 0) return this;
//		
//		//[active] angle definition
//		double tangent = Math.tan(radiansHalfAngle(degrees,360)); 
//		
//		//right factor operation
//		Quaternion qStack = new Quaternion(this).flipOp(element);
//		
//		if ( Math.abs(tangent) < 1.0) {
//			qStack.multiplyScalar(tangent);
//		}else {
//			this.divideScalar(tangent);
//		}
//		return add(qStack);
//	}
//
//	/**
//	 * Post-multiplication pre-multiplies Turn operator.
//	 * [Pre-multiplication right-turns operator.]
//	 * 
//	 * @param int
//	 *            m: index {1|2|3} of left Quaternion's unit-element factor.
//	 */
//	public Quaternion leftTurnOp(Axis element){		
//		//left factor operation
//		Quaternion qStack = new Quaternion(this).flipOp(element);		
//		return subtract(qStack);
//	}
//
//	/**
//	 * [Pre] Multiplication by elementary unit vector.
//	 * 
//	 * @param Axis
//	 *            unit index {I,J,K} of elementary left Quaternion factor.
//	 */
//	public Quaternion leftFlipOp(Axis fixed){
//			double tmp = q[0];
//			q[0] = -q[fixed.index()];
//			q[fixed.index()] = tmp;
//
//			tmp = -q[fixed.deqr()];
//			q[fixed.deqr()] = q[fixed.inqr()];
//			q[fixed.inqr()] = tmp;
//			return this;
//	}
//
//	/**
//	 * Pre-multiplication [Left operation] yields rotated Quaternion operator.
//	 * 
//	 * @param double
//	 *            angle: factor element of sparse Quaternion.
//	 * @param int
//	 *            m: index {1|2|3} of left Quaternion's unit-element factor.
//	 */
//	public Quaternion leftRotateOp(double degrees,Axis element){
////	    Fast Rotation operator -- and avoid manipulation if angle is zero...;
//		if (degrees == 0) return this;
//		
//		//[active] angle definition
//		double tangent = Math.tan(radiansHalfAngle(degrees,360)); 
//		
//		//right factor operation
////		Quaternion qStack = new Quaternion(this).leftFlipOp(element);
//		Quaternion qStack = new Quaternion(this).flipOp(element,Operate.Pre);
//		
//		if ( Math.abs(tangent) < 1.0) {
//			qStack.multiplyScalar(tangent);
//		}else {
//			this.divideScalar(tangent);
//		}
//		return add(qStack);
//	}

//	/**
//	 * Left Quaternion multiplication.
//	 */
//	public Quaternion preMultiply(Quaternion left)
//	{
//		return multiplyQ(left,this);
//	}
//	/**
//	 * [Right operation] Quaternion passive rotation; yields rotated Quaternion operator.
//	 * 
//	 * @param double
//	 *            angle: factor element of sparse Quaternion.
//	 * @param int
//	 *            m: index {1|2|3} of left Quaternion's unit-element factor.
//	 */
//	public Quaternion opRotatePassive(double degrees,Axis element){
////	    Fast Rotation operator -- and avoid manipulation if angle is zero...;
//		if (degrees == 0) return this;
//		
//		//passive angle definition
//		double tangent = -Math.tan(radiansHalfAngle(degrees,360)); 
//		
//		//right factor operation
//		Quaternion qStack = new Quaternion(this).flipAbout(element);
//		
//		if ( Math.abs(tangent) < 1.0) {
//			qStack.multiplyScalar(tangent);
//		}else {
//			this.divideScalar(tangent);
//		}
//		return add(qStack);
//	}

//	/**
//	 * Left operation Quaternion passive rotation; yields rotated Quaternion operator.
//	 * 
//	 * @param double
//	 *            angle: Radians measured left-handed about unit Element.
//	 * @param int
//	 *            unitElement: index {1|2|3} of left Quaternion's unit-element factor.
//	 */
//	public Quaternion leftOpRotatePassive(double degrees,Axis element){
////	    Fast Rotation operator -- and avoid manipulation if angle is zero...;
//		if (degrees == 0) return this;
//
//		//passive angle definition (Sign sense agrees with definition...)
//		double tangent = -Math.tan(radiansHalfAngle(degrees,360));
//		
//		//left factor operation
//		Quaternion pStack = new Quaternion(this).leftFlipAbout(element);
//		
//		if ( Math.abs(tangent) < 1.0) {
//			pStack.multiplyScalar(tangent);
//		}else {
//			this.divideScalar(tangent);
//		}
//		return add(pStack);
//	}
//	

	
//  protected Quaternion vectorMultiply(Quaternion t){
//	this.get(Axis.I)*=t.q[0];
//	this.q[1]*=t.q[1];
//	this.q[2]*=t.q[2];
//	this.q[3]*=t.q[3];
//	return this;
//}

//protected double sumElements(){
//	return q[0]+q[1]+q[2]+q[3];
//}

//protected Quaternion absElements(){
//	q[0]=Math.abs(q[0]);
//	q[1]=Math.abs(q[1]);
//	q[2]=Math.abs(q[2]);
//	q[3]=Math.abs(q[3]);
//	return this;
//}

//	//Vector index Incrementer:
//	protected static int inqr(int j){
//		return (j<3)? j+1 : 1;
//	}
//	
//	//Vector index Decrementer:
//	protected static int deqr(int j){
//		return (j>1)? j-1 : 3;
//	}
	
	
	
	
//	public static Quaternion rotateActive(Quaternion op, Quaternion q) {
//		return op.multiply(q).multiply(inverse(op));
//	}
//	
//	public static Quaternion rotatePassive(Quaternion op, Quaternion q) {
//		return inverse(op).multiply(q).multiply(op);
//	}
	
	
//	/**
//	 * Quaternion <b>p</b> rotation mutator:<p>(<b>p</b>)(<b>this</b>)(<b>p.conj()</b>)(<b>1/p.norm()</b>).
//	 * @param Quaternion <b>p</b> 
//	 */
//	public Quaternion doRotation(Quaternion p) {
//		//return multiply(p,Op.LEFT).multiply(new Quaternion(p).reciprocal(),Op.RIGHT);
//		return rotation(p,this);
//	}
//
//	/**
//	 * Quaternion <b>p</b> passive rotation mutator:<p>(<b>p.conj()</b>)(<b>this</b>)(<b>p</b>)(<b>1/p.norm()</b>).
//	 * @param Quaternion <b>p</b> 
//	 */
//	public Quaternion doRotationPassive(Quaternion p) {
//		//return multiply(new Quaternion(p).reciprocal(),Op.LEFT).multiply(p,Op.RIGHT);
//		return rotationPassive(p,this);
//	}
//
//	/**
//	 * Quaternion axis projection mutator:
//	 * @param Axis projected 
//	 * @param double length 
//	 */
//	public Quaternion doProjection(Axis projected, double length){
////		Quaternion w = new Quaternion(this);
////		return this
////		       .flip(projected,Op.RIGHT)
////		       .multiply(w.conj(),Op.RIGHT)
////		       .multiply(length/w.norm());
//		return projection(projected, length, this);
//	}	
//	
//	/**
//	 * Quaternion axis image projection mutator:
//	 * @param Axis projected image
//	 * @param double length 
//	 */
//	public Quaternion doImage(Axis projected, double length){
////		Quaternion w = new Quaternion(this);
////		return this.conj()
////	       .flip(projected,Op.RIGHT)
////	       .multiply(w,Op.RIGHT)
////	       .multiply(length/w.norm());
//		return image(projected, length, this);
//		
//	}

//	/**
//	 * Reciprocal mutator.
//	 */
//	public Quaternion reciprocal(){
//		//Invert the rotation. Normalize product magnitude with the Quaternion norm.
//		//		conj().divide(norm());
//		double scalarDivisor = norm();
//		if (scalarDivisor==0||Double.isNaN(scalarDivisor)){
//			set(NAN_QUATERNION);
//			return this;
//		}
//		q[0] /= scalarDivisor;
//		q[1] /= -scalarDivisor;
//		q[2] /= -scalarDivisor;
//		q[3] /= -scalarDivisor;
//
//		return this;
//	}
	
//	Static Factories:
	
//	/**
//	 * Hamiltonian Elementary Product static factory: <B>PI</B><p>
//	 * (right-hand 90 degrees rotation about <B>I</B> unit factor) 
//	 * @param Quaternion P factor.
//	 */
//	public static Quaternion rightMultiply_i(Quaternion p) {
//		return new Quaternion(-p.I,p.S,p.K,-p.J);
//	}	
//	/**
//	 * Hamiltonian Elementary Product static factory: <B>PJ</B><p>
//	 * (right-hand 90 degrees rotation about <B>J</B> unit factor) 
//	 * @param Quaternion P factor.
//	 */
//	public static Quaternion rightMultiply_j(Quaternion p) {
//		return new Quaternion(-p.J,-p.K,p.S,p.I);
//	}	
//	/**
//	 * Hamiltonian Elementary Product static factory: <B>PK</B><p>
//	 * (right-hand 90 degrees rotation about <B>K</B> unit factor) 
//	 * @param Quaternion P factor.
//	 */
//	public static Quaternion rightMultiply_k(Quaternion p) {
//		return new Quaternion(-p.K,p.J,-p.I,p.S);
//	}	
//	/**
//	 * Hamiltonian Elementary Product static factory: <B>IQ</B><p>
//	 * (left-hand 90 degrees rotation about <B>I</B> unit factor) 
//	 * @param Quaternion Q factor.
//	 */
//	public static Quaternion leftMultiply_i(Quaternion q) {
//		return new Quaternion(-q.I,q.S,-q.K,q.J);
//	}
//	/**
//	 * Hamiltonian Elementary Product static factory: <B>JQ</B><p>
//	 * (left-hand 90 degrees rotation about <B>J</B> unit factor) 
//	 * @param Quaternion Q factor.
//	 */
//	public static Quaternion leftMultiply_j(Quaternion q) {
//		return new Quaternion(-q.J,q.K,q.S,-q.I);
//	}	
//	/**
//	 * Hamiltonian Elementary Product static factory: <B>KQ</B><p>
//	 * (left-hand 90 degrees rotation about <B>K</B> unit factor) 
//	 * @param Quaternion Q factor.
//	 */
//	public static Quaternion leftMultiply_k(Quaternion q) {
//		return new Quaternion(-q.K,-q.J,q.I,q.S);
//	}
//	/**
//	 * Conjugate static factory.
//	 */
//	public static Quaternion conjugate(Quaternion p)
//	{
//		return new Quaternion( p.S,-p.I,-p.J,-p.K );
//	}
//
//	/**
//	 * Multiplicative Reciprocal static factory.
//	 */
//	public static Quaternion reciprocal(Quaternion p){
//		//Invert the rotation. Normalize product magnitude with the Quaternion norm.
//		//		conj().divide(norm());
//		double negNormP = -p.norm();
//		if (negNormP==0||Double.isNaN(negNormP)){
//			return new Quaternion(NAN_QUATERNION);
//		}
//		return new Quaternion( 	-p.S / negNormP,
//								p.I / negNormP,
//								p.J / negNormP,
//								p.K / negNormP  );
//		//		return new Quaternion( 	p.S / normP,
//		//								p.I / -normP,
//		//								p.J / -normP,
//		//								p.K / -normP  );
//	}
//	
//	/**
//	 * Hamiltonian Quaternion Product static factory.
//	 * @param Quaternion left.
//	 * @param Quaternion right.
//	 */
//	public static Quaternion multiply(Quaternion left, Quaternion right) {
//		return new Quaternion(
//				left.S*right.S - left.I*right.I - left.J*right.J - left.K*right.K,
//				left.I*right.S + left.S*right.I - left.K*right.J + left.J*right.K,
//				left.J*right.S + left.K*right.I + left.S*right.J - left.I*right.K,
//				left.K*right.S - left.J*right.I + left.I*right.J + left.S*right.K );
//				
////		return new Quaternion(left).multiply(right.S)
////		.add(rightMultiply_i(left).multiply(right.I))
////		.add(rightMultiply_j(left).multiply(right.J))
////		.add(rightMultiply_k(left).multiply(right.K));	
//				
////		return new Quaternion(right).multiply(left.S)
////		.add(leftMultiply_i(right).multiply(left.I))
////		.add(leftMultiply_j(right).multiply(left.J))
////		.add(leftMultiply_k(right).multiply(left.K));
//
//	}

//	/**
//	 * Hamiltonian Elementary Right Product static factory.
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) 
//	 * @param Quaternion left factor.
//	 * @param Unit { .S | .I | .J | .K } right hand factor.
//	 */
//	public static Quaternion multiply( Quaternion left, Unit right ) {
//		switch(right.enumIndex) {
//		case 1:  return QuaternionMath.rightMultiply_i(left);		
//		case 2:  return QuaternionMath.rightMultiply_j(left);		
//		case 3:  return QuaternionMath.rightMultiply_k(left);		
//		default: return new Quaternion(left);
//		}
//	}
//	
//
//	/**
//	 * Hamiltonian Elementary Left Product static factory.
//	 * <B>Defines 180 degrees rotation</B>
//	 * (90 per multiplication) 
//	 * @param Unit { .S | .I | .J | .K } left hand factor.
//	 * @param Quaternion right factor.
//	 */
//	public static Quaternion multiply( Unit left, Quaternion right ) {
//		switch(left.enumIndex) {
//		case 1:  return QuaternionMath.leftMultiply_i(right);		
//		case 2:  return QuaternionMath.leftMultiply_j(right);		
//		case 3:  return QuaternionMath.leftMultiply_k(right);		
//		default: return new Quaternion(right);
//		}
//	}
//
//	
	
//	/**
//	 * Scalar constructor.
//	 * 
//	 * @param s :
//	 *            double - scalar value.
//	 */
//	public Quaternion(double Scalar) {
//		S = Scalar;
//		I = 0;
//		J = 0;
//		K = 0;
//	}
	
//	public static double norm(Quaternion q)
//	{   return q.norm();	}
//	
//	public static boolean hasNaN(Quaternion q){
//		return q.hasNan(); 
//	}	

//	/**
//	 * Vector constructor.
//	 * 
//	 * @param vI :
//	 *            double - first vector element.
//	 * @param vJ :
//	 *            double - second vector element.
//	 * @param vK :
//	 *            double - third vector element.
//	 */
//	public Quaternion(double vI, double vJ, double vK) {
//		S = 0;
//		I = vI;
//		J = vJ;
//		K = vK;
//	}

	
	
	
	
}
