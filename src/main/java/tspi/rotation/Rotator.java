/**
 * 
 */
package tspi.rotation;

/**
 * 
 * Stores factored Quaternion: Ignores magnitude and allows fast rotations
 * 
 * @author mike
 * 
 */
public class Rotator extends Quaternion {

	/**
	 * Empty Constructor FastQuaternion:
	 */
	public Rotator() {
		super(Quaternion.EMPTY);
	}

	/**
	 * 
	 * Constructor FastQuaternion:
	 * 
	 * @param w
	 *            (double) -- Quaternion factor scalar part
	 * @param x
	 *            (double) -- Quaternion factor Vector3 element
	 * @param y
	 *            (double) -- Quaternion factor Vector3 element
	 * @param z
	 *            (double) -- Quaternion factor Vector3 element
	 */
	public Rotator(double w, double x, double y, double z) {
		super(w, x, y, z);
	}

	/**
	 * Constructor FastQuaternion:
	 * 
	 * @param w
	 *            (double) -- Quaternion factor scalar part
	 * @param v
	 *            (Vector3) -- Quaternion factor Vector part
	 */
	public Rotator(double w, Vector3 v) {
		super(w, v);
	}

	/**
	 * Constructor FastQuaternion:
	 */
	public Rotator(Quaternion q) {
		super(q);
	}

	/**
	 * <B>Rotate 180-Degree-rotation about unit basis axis mutator.</B> (90
	 * degrees right per multiplication)
	 * 
	 * @param axis
	 *            UnitBasis fixed {i}
	 */
	public final Rotator flip(Vector3 v) {
		Vector3 u = new Vector3(v).unit(); // vector magnitude...
		Quaternion t = new Quaternion(this);
		this.setRightMultiplyI(t);
		return (Rotator) this.multiply(u.getX()).addMultiplyRightJ(u.getY(), t).addMultiplyRightK(u.getZ(), t);
		// return (Operator) this.putRightI(u.getX(),
		// t).addMultiplyRightJ(u.getY(), t)
		// .addMultiplyRightK(u.getZ(), t);
	}

	/**
	 * <B>Mutator: Rotate 180-Degree right hand rotation about UNIT_I axis.</B>
	 * (Rotator 90 degrees per multiplication)
	 */
	public final Rotator flip_i() {
		this.setRightMultiplyI(new Quaternion(this));
		return this;
	}

	/**
	 * <B>Mutator: Rotate 180-Degree right hand rotation about UNIT_J axis.</B>
	 * (Rotator 90 degrees per multiplication)
	 */
	public final Rotator flip_j() {
		this.setRightMultiplyJ(new Quaternion(this));
		return this;
	}

	/**
	 * <B>Mutator: Rotate 180-Degree right hand rotation about UNIT_K axis.</B>
	 * (Rotator 90 degrees per multiplication)
	 */
	public final Rotator flip_k() {
		this.setRightMultiplyK(new Quaternion(this));
		return this;
	}

//	public final void set( Rotator t){
//		if (this != t) {
//			set(t);
//		} // do nothing ...else throw?
//    }

//	//public static functions:
//	public final void setTilt_i(Vector3 direction) {
//		//I==0...
//		double w = direction.getAbs() + direction.getX();
//		if (w > direction.getY()) {
//			if (w > direction.getZ()) {
//				//I
//				set(1, 0, direction.getZ() / w, -direction.getY() / w);
//			}
//		} else {
//			if (direction.getY() > direction.getZ()) {
//				//J
//				set(-w / direction.getY(), 0, -direction.getZ() / direction.getY(), 1);
//			}
//		}
//		//K
//		set(w / direction.getZ(), 0, 1, -direction.getY() / direction.getZ());
//	}	
//	
//	
//	public final void setTilt_j(Vector3 direction) {
//		//J==0...
//		double w = direction.getAbs() + direction.getY();
//		if (w > direction.getX()) {
//			if (w > direction.getZ()) {
//				//J
//				set(1, -direction.getZ() / w, 0, direction.getX() / w);
//			}
//
//		} else {
//			if (direction.getX() > direction.getZ()) {
//				//I
//				set(w / direction.getX(), -direction.getZ() / direction.getX(), 0, 1);
//			}
//		}
//		//K
//		set(-w / direction.getZ(), 1, 0, -direction.getX() / direction.getZ());
//	}	
//
//	
//	public final void setTilt_k(Vector3 direction) {
//		//K==0...
//		double w = direction.getAbs() + direction.getZ();
//		if (w > direction.getX()) {
//			if (w > direction.getY()) {
//				//K
//				set(1, direction.getY() / w, -direction.getX() / w, 0);
//			}
//
//		} else {
//			if (direction.getY() > direction.getX()) {
//				//J
//				set(w / direction.getY(), 1, -direction.getX() / direction.getY(), 0);
//			}
//		}
//		//I
//		set(-w / direction.getX(), -direction.getY() / direction.getX(), 1 , 0 );
//	}	
	
	/**
	 * Tait-Bryan Roll | Centerline twist :
	 * 
	 * @return Principle
	 */
	public CodedPhase getEuler_i_kji() {
		double w = getW();
		double x = getX();
		double y = getY();
		double z = getZ();
		double si = w * x + y * z;
		double ci = StrictMath.scalb( (w * w + z * z) - (x * x + y * y) , -1);
		double r = StrictMath.hypot(si, ci);
		double p = si / (r + ci);
		if ((r==-ci))
		{
			return CodedPhase.encodes(0);
		}
		if (Double.isNaN(p)) { //or very very large?
			p = (si > 0d) ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY;
		}
		return CodedPhase.encodes(p);
	}

	/**
	 * 
	 * @return CodedPhase of dumping pedestal
	 */
	public CodedPhase getEuler_j_kj() {
		double w = getW();
		double x = getX();
		double y = getY();
		double z = getZ();
		double ww = w * w;
		double xx = x * x;
		double yy = y * y;
		double zz = z * z;
		//double si = getW() * getX() + getY() * getZ();
		double sj = (w * y - z * x) / StrictMath.scalb((ww+xx+yy+zz), -1);
		if (StrictMath.abs(1d - StrictMath.abs(sj)) < 1e-15) {
			return CodedPhase.encodes(StrictMath.copySign(1d, sj));
		} 
		CodedPhase dumping = CodedPhase.encodes(sj / (1 + StrictMath.sqrt(1 - sj * sj)));
		
		if ( (w * x + y * z == 0) && ( (ww + zz) < (xx + yy) ) ){
			//is dumped!
			dumping.negate().subtractStraight();           
		} 
		return dumping;

	}
		
	/**
	 * Tait-Bryan Pitch | Centerline elevation : [-90..90]
	 * 
	 * @return Principle
	 */
	public CodedPhase getEuler_j_kji() {
		double sj = (getW() * getY() - getZ() * getX()) / StrictMath.scalb(getDeterminant(), -1);
		// TODO: Define real numerical limit to smallness. ... Euler numerical instability.
		if (StrictMath.abs(1d - StrictMath.abs(sj)) < 1e-15) {
			return CodedPhase.encodes(StrictMath.copySign(1d, sj));
		}
		return CodedPhase.encodes(sj / (1 + StrictMath.sqrt(1 - sj * sj)));
	}	
	
	/**
	 * 
	 * @return boolean
	 */
	public CodedPhase getEuler_k_kj() {
		double w = getW();
		double x = getX();
		double y = getY();
		double z = getZ();
		double xx = x * x;
		double yy = y * y;
		double zz = z * z;
		double ww = w * w;	
		
		//Case of pole: degenerate i
		if( ( ww + zz == xx + yy ) && ( w * x == - y * z ) ) {
	       return CodedPhase.encodes(z / w); 
	    }
		

		double sk = w * z + x * y;
		double ck = ((ww + xx) - (yy + zz))/2;
		Double h;
		if((sk==0)&&(ck <0)) {
			h = Double.POSITIVE_INFINITY;
		} else {

			h = sk / (StrictMath.hypot(sk, ck) + ck);
		}
		
        //Calculate heading...		
		CodedPhase dumping = CodedPhase.encodes(h);
		
		if ( ( w * x + y * z == 0) && ( ww + zz < xx + yy ) ){
			//is dumped!.negate()
			dumping.subtractStraight();           
		} 
		return dumping;
		
	}

	/**
	 * Tait-Bryan Yaw | Centerline azimuth :
	 * 
	 * @return Principle
	 */
	public CodedPhase getEuler_k_kji() {
		double w = getW();
		double x = getX();
		double y = getY();
		double z = getZ();
		double xx = x * x;
		double yy = y * y;
		double zz = z * z;
		double ww = w * w;		
		//Case of pole: degenerate i
		if( ( ww + zz == xx + yy ) && ( w * x == - y * z ) ) {
	       return CodedPhase.encodes(z / w); 
	    }
        //Calculate heading...
		double sk = w * z + x * y;
		double ck = ((ww + xx) - (yy + zz))/2;
		Double h = (sk / (StrictMath.hypot(sk, ck) + ck));
		if(h.isNaN()) {
			h=CodedPhase.CODE_STRAIGHT;
		}
		return CodedPhase.encodes(h);
	}

	// @Note: need array of Principle: type rotation vector.
	public Vector3 getEuler_kji() {
		//except when qx * qy + qz * qw = 0.5 (north pole)
		//except when qx * qy + qz * qw = -0.5 (south pole)
		double w = getW();
		double x = getX();
		double y = getY();
		double z = getZ();
		double ww = w * w;
		double xx = x * x;
		double yy = y * y;
		double zz = z * z;
		double wwpxx = ww+xx;
		double yypzz = yy + zz;
		double ck = StrictMath.scalb(wwpxx - yypzz, -1);
		double sj = (w * y - z * x) / StrictMath.scalb(wwpxx + yypzz, -1);
		double ci = StrictMath.scalb((ww+ zz) - (xx + yy), -1);
		double sk = w * z + x * y;
		double si = w * x + y * z;
		Double r = si / (StrictMath.hypot(si, ci) + ci);
		double p = sj / (1 + StrictMath.sqrt(1 - sj * sj));
		Double h = (sk / (StrictMath.hypot(sk, ck) + ck));
		if(r.isNaN()) {
			r = CodedPhase.CODE_STRAIGHT;
		}		
		if(h.isNaN()) {
			h = CodedPhase.CODE_STRAIGHT;
		}
		if((si==0) && (ci == 0)) { //pole!!!
			r = 0d;
			h = z / w;
		}
		return new Vector3((r), (p),
				(h));
	}

	/**
	 * Mutator.
	 * 
	 * this image after operation by object:
	 */
	public Rotator getImage(Quaternion object) {
		this.set(QuaternionMath.multiply(QuaternionMath.multiply(this, object), QuaternionMath.reciprocal(this)));
		return this;
	}

	
	
	public Vector3 getImage(Vector3 object) {
		return QuaternionMath.multiply(QuaternionMath.multiply(this, object), QuaternionMath.reciprocal(this)).getV();
	}

	/**
	 * Vector3 basis unit-I (image mapped by rotation operator q) static factory
	 */
	public Vector3 getImage_i() {
		return new Vector3(getW() * getW() + getX() * getX() - getY() * getY() - getZ() * getZ(),
				2 * (getX() * getY() + getW() * getZ()), 2 * (getX() * getZ() - getW() * getY()));
	}

	/**
	 * Vector3 Projected unit-J (Basis transformed by rotation operator q)
	 * static factory
	 */
	public Vector3 getImage_j() {
		return new Vector3(2 * (getX() * getY() - getW() * getZ()),
				getW() * getW() - getX() * getX() + getY() * getY() - getZ() * getZ(),
				2 * (getW() * getX() + getY() * getZ()));
	}

	/**
	 * Vector3 Projected unit-K (Basis transformed by rotation operator q)
	 * static factory
	 */
	public Vector3 getImage_k() {
		return new Vector3(2 * (getW() * getY() + getX() * getZ()), 2 * (getY() * getZ() - getW() * getX()),
				getW() * getW() - getX() * getX() - getY() * getY() + getZ() * getZ());
	}

	public double getImageNormalizationFactor() {
		return 1d / (getDeterminant());
	}

	public double getNormaliztionFactor() {
		return 1d / (getAbs());
	}

	/**
	 * Mutator.
	 * 
	 * this pre-image of objevt before rotation operation:
	 */
	public Rotator getPreImage(Quaternion object) {
		this.set(QuaternionMath.multiply(QuaternionMath.multiply(QuaternionMath.reciprocal(this), object), this));
		return this;
	}

	public Vector3 getPreImage(Vector3 object) {
		return QuaternionMath.multiply(QuaternionMath.multiply(QuaternionMath.reciprocal(this), object), this).getV();
	}

	/**
	 * Vector3 basis unit-I (pre-image mapped by rotation operator q) static
	 * factory
	 */
	public Vector3 getPreImage_i() {
		return new Vector3(getW() * getW() + getX() * getX() - getY() * getY() - getZ() * getZ(),
				2 * (getX() * getY() - getW() * getZ()), 2 * (getW() * getY() + getX() * getZ()));
	}

	/**
	 * Vector3 basis unit-J (pre-image mapped by rotation operator q) static
	 * factory
	 */
	public Vector3 getPreImage_j() {
		return new Vector3(2 * (getX() * getY() + getW() * getZ()),
				getW() * getW() - getX() * getX() + getY() * getY() - getZ() * getZ(),
				2 * (getY() * getZ() - getW() * getX()));
	}

	/**
	 * Vector3 basis unit-K (pre-image mapped by rotation operator q) static
	 * factory
	 */
	public Vector3 getPreImage_k() {
		return new Vector3(2 * (getX() * getZ() - getW() * getY()), 2 * (getW() * getX() + getY() * getZ()),
				getW() * getW() - getX() * getX() - getY() * getY() + getZ() * getZ());
	}

	public Angle getRotationAngle() {
		return this.getArg().angle().signedPrinciple();
	}

	public Vector3 getRotationAxis() {
		return getV().unit();
	}

	/**
	 * 
	 * @return boolean
	 */
	public boolean isNormal_kj() {
		//double si = getW() * getX() + getY() * getZ();
		return ( (getW() * getX() + getY() * getZ() == 0)
			      && ( getW() * getW()+getZ() * getZ()) >= (getX() * getX()+getY() * getY() )
			   );

	}

	// /**
	// * <B>Rotate 180-Degree-rotation about unit basis axis mutator.</B>
	// * (90 degrees right per multiplication)
	// *
	// * @param axis
	// * UnitBasis fixed for flip
	// */
	// public final Rotator flip(BasisUnit e){
	// switch(e)
	// {
	// case I:
	// return flip_i();
	// case J:
	// return flip_j();
	// case K:
	// return flip_k();
	// default:
	// System.out.println("Error: not BasisAxis to flip about...");
	// }
	// return this;
	// }

	/**
	 * <B>Rotate 180-Degree-Pre-rotation about unit basis axis mutator.</B> (90
	 * degrees left per multiplication)
	 * 
	 * @param axis
	 *            UnitBasis fixed {i}
	 */
	public final Rotator preFlip(Vector3 v) {
		Vector3 u = new Vector3(v).unit(); // vector magnitude...
		Quaternion t = new Quaternion(this);
		this.setLeftMultiplyI(t);
		return (Rotator) this.multiply(u.getX()).addMultiplyLeftJ(u.getY(), t).addMultiplyLeftK(u.getZ(), t);
	}

	/**
	 * <B>Mutator: Rotate 180-Degree left hand [pre] rotation about UNIT_I
	 * axis.</B> (Rotator 90 degrees per multiplication)
	 */
	public final Rotator preFlip_i() {
		this.setLeftMultiplyI(new Quaternion(this));
		return this;
	}

	/**
	 * <B>Mutator: Rotate 180-Degree left hand [pre] rotation about UNIT_J
	 * axis.</B> (Rotator 90 degrees per multiplication)
	 */
	public final Rotator preFlip_j() {
		this.setLeftMultiplyJ(new Quaternion(this));
		return this;
	}

	// /**
	// * <b>Quarter (90) Degree-Turn about basis frame axis mutator.</B> (45
	// degrees
	// * per multiplication)
	// * @param e BasisUnit imaginary axis to turn-about 90 degrees.
	// */
	// public final Rotator turn(BasisUnit e){
	// switch(e)
	// {
	// case I:
	// return this.turn_i();
	// case J:
	// return this.turn_j();
	// case K:
	// return this.turn_k();
	// default:
	// System.out.println("Error: not BasisAxis to turn about...");
	// return this;
	// }
	// }

	/**
	 * <B>Mutator: Rotate 180-Degree left hand [pre] rotation about UNIT_K
	 * axis.</B> (Rotator 90 degrees per multiplication)
	 */
	public final Rotator preFlip_k() {
		this.setLeftMultiplyK(new Quaternion(this));
		return this;
	}

	public final Rotator preRotate(CodedPhase angle, Vector3 v) {
		if (angle.isZero()) {
			return this;
		}
		Vector3 clone = new Vector3(v);
		double m = clone.getAbs();
		if (angle.isAcute()) {
			clone.multiply(angle.tanHalf() / m);
			return (Rotator) this.leftMultiply(new Quaternion(1.d, clone.getX(), clone.getY(), clone.getZ()));
		}
		if (angle.isStraight()) {
			clone.divide(m);
			return (Rotator) this.leftMultiply(clone);
		}
		// (angle.isObtuse())
		clone.divide(m);
		return (Rotator) this.leftMultiply(new Quaternion(angle.cotHalf(), clone.getX(), clone.getY(), clone.getZ()));
	}

	public final Rotator preRotate(Quaternion p) {
		this.leftMultiply(p);
		return this;
	}

	

	// Mutator.

	public final Rotator preRotate(Vector3 v) {

		Vector3 clone = new Vector3(v);
		double m = clone.getAbs();
		CodedPhase angle = Angle.inRadians(m).codedPhase();

		if (angle.isZero()) {
			return this;
		}
		if (angle.isAcute()) {
			clone.multiply(angle.tanHalf() / m);
			return (Rotator) this.leftMultiply(new Quaternion(1.d, clone.getX(), clone.getY(), clone.getZ()));
		}
		if (angle.isStraight()) {
			clone.divide(m);
			return (Rotator) this.leftMultiply(clone);
		}
		// (angle.isObtuse())
		clone.divide(m);
		return (Rotator) this.leftMultiply(new Quaternion(angle.cotHalf(), clone.getX(), clone.getY(), clone.getZ()));
	}

	/**
	 * pre- sense i rotation mutator.
	 * <p>
	 * [leftMultiply( exp(i*angle) ) ]
	 * 
	 * @param omega_i
	 *            Principle rotation (halved per factor multiplication)
	 */
	public final Rotator preRotate_i(final CodedPhase omega_i) {
		if (omega_i.isZero()) {
			return this;
		}
		final Rotator clone = new Rotator(this);
		// clone.set(this);
		if (omega_i.isAcute()) { // return exp_iAcute(angle) ;
			return (Rotator) this.addMultiplyLeftI(omega_i.tanHalf(), clone);
		}
		if (omega_i.isStraight()) { // return this.flip_i();
			this.setLeftMultiplyI(clone);
			return this;
		}
		// return exp_iObtuse(angle) ;
		return (Rotator) this.multiply(omega_i.cotHalf()).addLeftMultiplyI(clone);
		// return (Operator) this.multiplyAddLeftI(angle.cotHalf(), clone);

	}

	/**
	 * Pre- sense J rotation mutator.
	 * <p>
	 * [ leftMultiply( exp(j*angle) ) ]
	 * 
	 * @param Angle
	 *            rotation (halved per factor multiplication)
	 */
	public final Rotator preRotate_j(final CodedPhase omega_j) {
		if (omega_j.isZero()) {
			return this;
		}
		final Rotator clone = new Rotator(this);
		// clone.set(this);
		if (omega_j.isAcute()) { // return exp_iAcute(angle) ;
			return (Rotator) this.addMultiplyLeftJ(omega_j.tanHalf(), clone);
		}
		if (omega_j.isStraight()) { // return this.flip_i();
			this.setLeftMultiplyJ(clone);
			return this;
		}
		// return exp_iObtuse(angle) ;
		return (Rotator) this.multiply(omega_j.cotHalf()).addLeftMultiplyJ(clone);
		// return (Operator) this.multiplyAddLeftJ(angle.cotHalf(),clone);

	}

	// Mutator.
	
	/**
	 * Pre- sense K rotation mutator.
	 * <p>
	 * [ leftMultiply( exp(k*angle) ) ]
	 * 
	 * @param Angle
	 *            rotation (halved per factor multiplication)
	 */
	public final Rotator preRotate_k(final CodedPhase omega_k) {
		if (omega_k.isZero()) {
			return this;
		}
		final Rotator clone = new Rotator(this);
		// clone.set(this);
		if (omega_k.isAcute()) { // return exp_iAcute(angle) ;
			return (Rotator) this.addMultiplyLeftK(omega_k.tanHalf(), clone);
		}
		if (omega_k.isStraight()) { // return this.flip_i();
			this.setLeftMultiplyK(clone);
			return this;
		}
		// return exp_iObtuse(angle) ;
		return (Rotator) this.multiply(omega_k.cotHalf()).addLeftMultiplyK(clone);
		// return (Operator) this.multiplyAddLeftK(angle.cotHalf(), clone);
	}

	// Mutator.
	public final Rotator preTilt_i(final CodedPhase omega_j, final CodedPhase omega_k) {
		// I==0...
		Rotator t = new Rotator(this); // t anonymous with gc.
		// if (t==null||t==this){
		// t = new Rotator(this); //t anonymous with gc.
		// } else {
		// t.set(this); //re-uses external t...class needs be unique per
		// instance.
		// }
		// t.set(this);
	
		if (omega_j.isAcute()) {
			if (omega_k.isAcute()) {
				return (Rotator) this.addMultiplyLeftJ(omega_j.tanHalf(), t).addMultiplyLeftK(omega_k.tanHalf(), t);
			}
			return (Rotator) this.multiply(omega_k.cotHalf()).addLeftMultiplyK(t)
					.addMultiplyLeftJ(omega_j.tanHalf() / omega_k.tanHalf(), t);
			// return this.multiplyAddLeftK(thetaK.cotHalf(), t)
			// .addMultiplyLeftJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
		}
		if (omega_k.codedMagnitude() > omega_j.codedMagnitude()) {
			return (Rotator) this.multiply(omega_k.cotHalf()).addLeftMultiplyK(t)
					.addMultiplyLeftJ(omega_j.tanHalf() / omega_k.tanHalf(), t);
			// return this.multiplyAddLeftK(thetaK.cotHalf(), t)
			// .addMultiplyLeftJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
		}
		return (Rotator) this.multiply(omega_j.cotHalf()).addLeftMultiplyJ(t)
				.addMultiplyLeftK(omega_k.tanHalf() / omega_j.tanHalf(), t);
		// return this.multiplyAddLeftJ(thetaJ.cotHalf(), t).addMultiplyLeftK(
		// thetaK.tanHalf() / thetaJ.tanHalf(), t);
	}

	/**
	 * Rank 2 Mutator.
	 * 
	 * @param direction
	 *            Vector3
	 * @return this left multiplied by Rotator defined by tilt of basis unit
	 *         <b>I</b> to align with <b>direction</b> vector3.
	 */
	public final Rotator preTilt_i(final Vector3 direction) {
		// I==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
		final double w = direction.getAbs() + direction.getX();
		final double yAbs = StrictMath.abs(direction.getY());
		if (w > yAbs) {
			if (w > StrictMath.abs(direction.getZ())) { // I
				return (Rotator) this.addMultiplyLeftJ(direction.getZ() / w, t).addMultiplyLeftK(-direction.getY() / w,
						t);
			}
		} else {
			if (yAbs > StrictMath.abs(direction.getZ())) { // J
				return (Rotator) this.multiply(-w / direction.getY()).addLeftMultiplyK(t)
						.addMultiplyLeftJ(-direction.getZ() / direction.getY(), t);
				// return this.multiplyAddLeftK(-w / direction.getY(), t)
				// .addMultiplyLeftJ(-direction.getZ() / direction.getY(),t);
			}
		}
		// K
		return (Rotator) this.multiply(w / direction.getZ()).addLeftMultiplyJ(t)
				.addMultiplyLeftK(-direction.getY() / direction.getZ(), t);
		// return this.multiplyAddLeftJ(w / direction.getZ(),
		// t).addMultiplyLeftK(
		// -direction.getY() / direction.getZ(), t);
	}

	// Mutator.
	public final Rotator preTilt_j(final CodedPhase omega_k, final CodedPhase omega_i) {
		// I==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
		if (omega_k.isAcute()) {
			if (omega_i.isAcute()) {
				return (Rotator) this.addMultiplyLeftK(omega_k.tanHalf(), t).addMultiplyLeftI(omega_i.tanHalf(), t);
			}
			return (Rotator) this.multiply(omega_i.cotHalf()).addLeftMultiplyI(t)
					.addMultiplyLeftK(omega_k.tanHalf() / omega_i.tanHalf(), t);
			// return this.multiplyAddLeftI(thetaI.cotHalf(), t)
			// .addMultiplyLeftK(thetaK.tanHalf() / thetaI.tanHalf(), t);
		}
		if (omega_i.codedMagnitude() > omega_k.codedMagnitude()) {
			return (Rotator) this.multiply(omega_i.cotHalf()).addLeftMultiplyI(t)
					.addMultiplyLeftK(omega_k.tanHalf() / omega_i.tanHalf(), t);
			// return this.multiplyAddLeftI(thetaI.cotHalf(), t)
			// .addMultiplyLeftK(thetaK.tanHalf() / thetaI.tanHalf(), t);
		}
		return (Rotator) this.multiply(omega_k.cotHalf()).addLeftMultiplyK(t)
				.addMultiplyLeftI(omega_i.tanHalf() / omega_k.tanHalf(), t);
		// return this.multiplyAddLeftK(thetaK.cotHalf(), t).addMultiplyLeftI(
		// thetaI.tanHalf() / thetaK.tanHalf(), t);
	}

	/**
	 * Rank 2 Mutator.
	 * 
	 * @param direction
	 *            Vector3
	 * @return this left multiplied by Rotator defined by tilt of basis unit
	 *         <b>J</b> to align with <b>direction</b> vector3.
	 */
	public final Rotator preTilt_j(final Vector3 direction) {
		// J==0...
		// t.set(this);
		final Rotator t = new Rotator(this);
		final double w = direction.getAbs() + direction.getY();
		final double xAbs = StrictMath.abs(direction.getX());

		if (w > xAbs) {
			if (w > StrictMath.abs(direction.getZ())) { // J
				return (Rotator) this.addMultiplyLeftI(-direction.getZ() / w, t).addMultiplyLeftK(direction.getX() / w,
						t);
			}
		} else {
			if (xAbs > StrictMath.abs(direction.getZ())) { // I
				return (Rotator) this.multiply(w / direction.getX()).addLeftMultiplyK(t)
						.addMultiplyLeftI(-direction.getZ() / direction.getX(), t);
				// return this.multiplyAddLeftK(w / direction.getX(), t)
				// .addMultiplyLeftI(-direction.getZ() / direction.getX(),t);
			}
		}
		// K
		return (Rotator) this.multiply(-w / direction.getZ()).addLeftMultiplyI(t)
				.addMultiplyLeftK(-direction.getX() / direction.getZ(), t);
		// return this.multiplyAddLeftI(-w / direction.getZ(), t)
		// .addMultiplyLeftK(-direction.getX() / direction.getZ(), t);
	}

	// Mutator.
	public final Rotator preTilt_k(final CodedPhase omega_i, final CodedPhase omega_j) {
		// I==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
		if (omega_i.isAcute()) {
			if (omega_j.isAcute()) {
				return (Rotator) this.addMultiplyLeftI(omega_i.tanHalf(), t).addMultiplyLeftJ(omega_j.tanHalf(), t);
			}
			return (Rotator) this.multiply(omega_j.cotHalf()).addLeftMultiplyJ(t)
					.addMultiplyLeftI(omega_i.tanHalf() / omega_j.tanHalf(), t);
			// return this.multiplyAddLeftJ(thetaJ.cotHalf(), t)
			// .addMultiplyLeftI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
		}
		if (omega_j.codedMagnitude() > omega_i.codedMagnitude()) {
			return (Rotator) this.multiply(omega_j.cotHalf()).addLeftMultiplyJ(t)
					.addMultiplyLeftI(omega_i.tanHalf() / omega_j.tanHalf(), t);
			// return this.multiplyAddLeftJ(thetaJ.cotHalf(), t)
			// .addMultiplyLeftI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
		}
		return (Rotator) this.multiply(omega_i.cotHalf()).addLeftMultiplyI(t)
				.addMultiplyLeftJ(omega_j.tanHalf() / omega_i.tanHalf(), t);
		// return this.multiplyAddLeftI(thetaI.cotHalf(), t).addMultiplyLeftJ(
		// thetaJ.tanHalf() / thetaI.tanHalf(), t);
	}

	/**
	 * Rank 2 Mutator.
	 * 
	 * @param direction
	 *            Vector3
	 * @return this left multiplied by Rotator defined by tilt of basis unit
	 *         <b>K</b> to align with <b>direction</b> vector3.
	 */
	public final Rotator preTilt_k(final Vector3 direction) {
		// K==0...
		final Rotator t = new Rotator(this);
		// t.set(this);

		final double w = direction.getAbs() + direction.getZ();
		final double xAbs = StrictMath.abs(direction.getX());

		if (w > xAbs) {
			if (w > direction.getY()) { // K
				return (Rotator) this.addMultiplyLeftI(direction.getY() / w, t).addMultiplyLeftJ(-direction.getX() / w,
						t);
			}
		} else {
			if (direction.getY() > xAbs) { // J
				return (Rotator) this.multiply(w / direction.getY()).addLeftMultiplyI(t)
						.addMultiplyLeftJ(-direction.getX() / direction.getY(), t);
				// return this.multiplyAddLeftI(w / direction.getY(), t)
				// .addMultiplyLeftJ(-direction.getX() / direction.getY(),t);
			}
		}
		// I
		return (Rotator) this.multiply(-w / direction.getX()).addLeftMultiplyJ(t)
				.addMultiplyLeftI(-direction.getY() / direction.getX(), t);
		// return this.multiplyAddLeftJ(-w / direction.getX(), t)
		// .addMultiplyLeftI(-direction.getY() / direction.getX(), t);
	}

	/**
	 * <B>Rotate 90-Degree-Pre-Turn about vector-axis mutator.</B> (45 degrees
	 * per multiplication)
	 * 
	 * @param v
	 *            Vector3 pre-turn-axis
	 */
	public final Rotator preTurn(Vector3 v) {
		if (v.getNormInf() == 0)
			return new Rotator(Quaternion.EMPTY); // ????
		Vector3 u = new Vector3(v).unit(); // vector magnitude...
		Quaternion t = new Quaternion(this);
		return (Rotator) this.addMultiplyLeftI(u.getX(), t).addMultiplyLeftJ(u.getY(), t).addMultiplyLeftK(u.getZ(), t);
	}

	/**
	 * <B>Mutator: Rotate 90-Degree left hand [pre] turn about UNIT_I axis.</B>
	 * (Rotator 45 degrees per multiplication)
	 */
	public final Rotator preTurn_i() {
		return (Rotator) this.addLeftMultiplyI(new Quaternion(this));
	}

	/**
	 * <B>Mutator: Rotate 90-Degree left hand [pre] turn about UNIT_J axis.</B>
	 * (Rotator 45 degrees per multiplication)
	 */
	public final Rotator preTurn_j() {
		return (Rotator) this.addLeftMultiplyJ(new Quaternion(this));
	}

	/**
	 * <B>Mutator: Rotate 90-Degree left hand [pre] turn about UNIT_K axis.</B>
	 * (Rotator 45 degrees per multiplication)
	 */
	public final Rotator preTurn_k() {
		return (Rotator) this.addLeftMultiplyK(new Quaternion(this));
	}

	/**
	 * Right sense rotation mutator. [ Right multiply( exp(angle*v.unit()) ) ]
	 * 
	 * @param angle
	 *            CodedPrinciple rotation (halved per factor multiplication)
	 * @param v
	 *            Vector3 axis of rotation
	 */
	public final Rotator rotate(CodedPhase angle, Vector3 v) {
		if (angle.isZero()) {
			return this;
		}
		Vector3 clone = new Vector3(v);
		double m = clone.getAbs();
		if (angle.isAcute()) {
			clone.multiply(angle.tanHalf() / m);
			return (Rotator) this.rightMultiply(new Quaternion(1.d, clone.getX(), clone.getY(), clone.getZ()));
		}
		if (angle.isStraight()) {
			clone.divide(m);
			return (Rotator) this.rightMultiply(clone);
		}
		// (angle.isObtuse())
		clone.divide(m);
		return (Rotator) this.rightMultiply(new Quaternion(angle.cotHalf(), clone.getX(), clone.getY(), clone.getZ()));
	}

	public final Rotator rotate(Quaternion p) {
		this.rightMultiply(p);
		return this;
	}

	/**
	 * Right sense rotation mutator. [ Right multiply( exp(angle*v.unit()) ) ]
	 * 
	 * @param Angle
	 *            rotation (halved per factor multiplication)
	 */
	public final Rotator rotate(Vector3 v) {

		Vector3 clone = new Vector3(v);
		double m = clone.getAbs();
		CodedPhase angle = Angle.inRadians(m).codedPhase();

		if (angle.isZero()) {
			return this;
		}
		if (angle.isAcute()) {
			clone.multiply(angle.tanHalf() / m);
			return (Rotator) this.rightMultiply(new Quaternion(1.d, clone.getX(), clone.getY(), clone.getZ()));
		}
		if (angle.isStraight()) {
			clone.divide(m);
			return (Rotator) this.rightMultiply(clone);
		}
		// (angle.isObtuse())
		clone.divide(m);
		return (Rotator) this.rightMultiply(new Quaternion(angle.cotHalf(), clone.getX(), clone.getY(), clone.getZ()));
	}

	/**
	 * Mutator: Right-hand rotate about <b>UNIT_I</b> axis: exp(iW).
	 * 
	 * @param omega_i CodedPrinciple angle of rotation.
	 */
	public final Rotator rotate_i(final CodedPhase omega_i) {
		if (omega_i.isZero()) {
			return this;
		}
		final Rotator clone = new Rotator(this);
		// clone.set(this);
		if (omega_i.isAcute()) { // return exp_iAcute(angle) ;
			return (Rotator) this.addMultiplyRightI(omega_i.tanHalf(), clone);
		}
		if (omega_i.isStraight()) { // return this.flip_i();
			this.setRightMultiplyI(clone);
			return this;
		}
		// return exp_iObtuse(angle) ;
		return (Rotator) this.multiply(omega_i.cotHalf()).addRightMultiplyI(clone);
		// return (Operator) this.multiplyAddRightI(thetaI.cotHalf(),clone);

	}

	/**
	 * Right sense J rotation mutator.
	 * <p>
	 * [ Right.multiply( exp(j*angle) ) ]
	 * 
	 * @param Angle
	 *            rotation (halved per factor multiplication)
	 */
	public final Rotator rotate_j(final CodedPhase omega_j) {
		if (omega_j.isZero()) {
			return this;
		}
		final Rotator clone = new Rotator(this);
		// clone.set(this);

		if (omega_j.isAcute()) { // return exp_iAcute(angle) ;
			return (Rotator) this.addMultiplyRightJ(omega_j.tanHalf(), clone);
		}
		if (omega_j.isStraight()) { // return this.flip_i();
			this.setRightMultiplyJ(clone);
			return this;
		}
		// return exp_iObtuse(angle) ;
		return (Rotator) this.multiply(omega_j.cotHalf()).addRightMultiplyJ(clone);
		// return (Operator) this.multiplyAddRightJ(thetaJ.cotHalf(), clone);
	}

	/**
	 * Right sense K rotation mutator.
	 * <p>
	 * [ Right.multiply( exp(k*angle) ) ]
	 * 
	 * @param Angle
	 *            rotation (halved per factor multiplication)
	 */
	public final Rotator rotate_k(final CodedPhase omega_k) {
		if (omega_k.isZero()) {
			return this;
		}
		final Rotator clone = new Rotator(this);
		// clone.set(this);
		if (omega_k.isAcute()) { // return exp_iAcute(angle) ;
			return (Rotator) this.addMultiplyRightK(omega_k.tanHalf(), clone);
		}
		if (omega_k.isStraight()) { // return this.flip_i();
			this.setRightMultiplyK(clone);
			return this;
		}
		// return exp_iObtuse(angle) ;
		return (Rotator) this.multiply(omega_k.cotHalf()).addRightMultiplyK(clone);
		// return (Operator) this.multiplyAddRightK(thetaK.cotHalf(), clone);
	}

	/**
	 * Mutator.
	 * 
	 * SLERP mutator:
	 */
	public Rotator slerp(final Quaternion p, final double t) {
		set(QuaternionMath.slerp(this, p, t));
		return this;
	}

	// Mutator.
	/**
	 * Rank 2 Mutator.
	 * 
	 * @param omega_j
	 *            defines w_j <-- omegaJ.signedAngle()
	 * @param omega_k
	 *            defines w_k <-- omegaK.signedAngle()
	 * 
	 * @return this right multiplied by Rotator defined by exp[ <b>j</b>(w_j)/2
	 *         +<b>k</b>(w_k)/2 ]
	 */
	public final Rotator tilt_i(final CodedPhase omega_j, final CodedPhase omega_k) {
		// I==0...
		// t.set(this);

		final Rotator t = new Rotator(this);
		if (omega_j.isAcute()) {
			if (omega_k.isAcute()) {
				return (Rotator) this.addMultiplyRightJ(omega_j.tanHalf(), t).addMultiplyRightK(omega_k.tanHalf(), t);
			}
			return (Rotator) this.multiply(omega_k.cotHalf()).addRightMultiplyK(t)
					.addMultiplyRightJ(omega_j.tanHalf() / omega_k.tanHalf(), t);
			// return this.multiplyAddRightK(thetaK.cotHalf(), t)
			// .addMultiplyRightJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
		}
		if (omega_k.codedMagnitude() > omega_j.codedMagnitude()) {
			return (Rotator) this.multiply(omega_k.cotHalf()).addRightMultiplyK(t)
					.addMultiplyRightJ(omega_j.tanHalf() / omega_k.tanHalf(), t);
			// return this.multiplyAddRightK(thetaK.cotHalf(), t)
			// .addMultiplyRightJ(thetaJ.tanHalf() / thetaK.tanHalf(), t);
		}
		return (Rotator) this.multiply(omega_j.cotHalf()).addRightMultiplyJ(t)
				.addMultiplyRightK(omega_k.tanHalf() / omega_j.tanHalf(), t);
		// return this.multiplyAddRightJ(thetaJ.cotHalf(), t).addMultiplyRightK(
		// thetaK.tanHalf() / thetaJ.tanHalf(), t);

	}

	// Mutator.
	/**
	 * Rank 2 Mutator.
	 * 
	 * @param direction
	 *            Vector3
	 * @return this right multiplied by Rotator defined by tilt of basis unit
	 *         <b>I</b> to align with <b>direction</b> vector3.
	 */
	public final Rotator tilt_i(final Vector3 direction) {
		// I==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
		final double w = direction.getAbs() + direction.getX();
		final double yAbs = StrictMath.abs(direction.getY());
		if (w > yAbs) {
			if (w > StrictMath.abs(direction.getZ())) { // I
				return (Rotator) this.addMultiplyRightJ(direction.getZ() / w, t)
						.addMultiplyRightK(-direction.getY() / w, t);
			}
		} else {
			if (yAbs > StrictMath.abs(direction.getZ())) { // J
				return (Rotator) this.multiply(-w / direction.getY()).addRightMultiplyK(t)
						.addMultiplyRightJ(-direction.getZ() / direction.getY(), t);
				// return this.multiplyAddRightK(-w / direction.getY(),t)
				// .addMultiplyRightJ(-direction.getZ() / direction.getY(), t);
			}
		}
		// K
		return (Rotator) this.multiply(w / direction.getZ()).addRightMultiplyJ(t)
				.addMultiplyRightK(-direction.getY() / direction.getZ(), t);
		// return this.multiplyAddRightJ(w / direction.getZ(), t)
		// .addMultiplyRightK(-direction.getY() / direction.getZ(), t);
	}

	/**
	 * Rank 2 Mutator.
	 * 
	 * @param omega_k
	 *            defines w_k <-- omegaK.signedAngle()
	 * @param omega_i
	 *            defines w_i <-- omegaI.signedAngle()
	 * 
	 * @return this right multiplied by Rotator defined by exp[ <b>j</b>(w_i)/2
	 *         +<b>k</b>(w_k)/2 ]
	 */
	public final Rotator tilt_j(final CodedPhase omega_k, final CodedPhase omega_i) {
		// I==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
		if (omega_k.isAcute()) {
			if (omega_i.isAcute()) {
				return (Rotator) this.addMultiplyRightK(omega_k.tanHalf(), t).addMultiplyRightI(omega_i.tanHalf(), t);
			}
			return (Rotator) this.multiply(omega_i.cotHalf()).addRightMultiplyI(t)
					.addMultiplyRightK(omega_k.tanHalf() / omega_i.tanHalf(), t);
			// return this.multiplyAddRightI(thetaI.cotHalf(), t)
			// .addMultiplyRightK(thetaK.tanHalf() / thetaI.tanHalf(), t);
		}
		if (omega_i.codedMagnitude() > omega_k.codedMagnitude()) {
			return (Rotator) this.multiply(omega_i.cotHalf()).addRightMultiplyI(t)
					.addMultiplyRightK(omega_k.tanHalf() / omega_i.tanHalf(), t);
			// return this.multiplyAddRightI(thetaI.cotHalf(), t)
			// .addMultiplyRightK(thetaK.tanHalf() / thetaI.tanHalf(), t);
		}
		return (Rotator) this.multiply(omega_k.cotHalf()).addRightMultiplyK(t)
				.addMultiplyRightI(omega_i.tanHalf() / omega_k.tanHalf(), t);
		// return this.multiplyAddRightK(thetaK.cotHalf(), t).addMultiplyRightI(
		// thetaI.tanHalf() / thetaK.tanHalf(), t);

	}

	/**
	 * Rank 2 Mutator.
	 * 
	 * @param direction
	 *            Vector3
	 * @return this right multiplied by Rotator defined by tilt of basis unit
	 *         <b>J</b> to align with <b>direction</b> vector3.
	 */
	public final Rotator tilt_j(final Vector3 direction) { // J==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
	
		final double w = direction.getAbs() + direction.getY();
		final double xAbs = StrictMath.abs(direction.getX());
	
		if (w > xAbs) {
			if (w > StrictMath.abs(direction.getZ())) { // J
				return (Rotator) this.addMultiplyRightI(-direction.getZ() / w, t)
						.addMultiplyRightK(direction.getX() / w, t);
			}
		} else {
			if (xAbs > StrictMath.abs(direction.getZ())) { // I
				return (Rotator) this.multiply(w / direction.getX()).addRightMultiplyK(t)
						.addMultiplyRightI(-direction.getZ() / direction.getX(), t);
				// return this.multiplyAddRightK(w / direction.getX(), t)
				// .addMultiplyRightI(-direction.getZ() / direction.getX(), t);
			}
		}
		// K
		return (Rotator) this.multiply(-w / direction.getZ()).addRightMultiplyI(t)
				.addMultiplyRightK(-direction.getX() / direction.getZ(), t);
		// return this.multiplyAddRightI(-w / direction.getZ(), t)
		// .addMultiplyRightK(-direction.getX() / direction.getZ(), t);
	}

	// Mutator.
	public final Rotator tilt_k(final CodedPhase omega_i, final CodedPhase omega_j) {
		// I==0...

		final Rotator t = new Rotator(this);
		// t.set(this);

		if (omega_i.isAcute()) {
			if (omega_j.isAcute()) {
				return (Rotator) this.addMultiplyRightI(omega_i.tanHalf(), t).addMultiplyRightJ(omega_j.tanHalf(), t);
			}
			return (Rotator) this.multiply(omega_j.cotHalf()).addRightMultiplyJ(t)
					.addMultiplyRightI(omega_i.tanHalf() / omega_j.tanHalf(), t);
			// return this.multiplyAddRightJ(thetaJ.cotHalf(), t)
			// .addMultiplyRightI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
		}
		if (omega_j.codedMagnitude() > omega_i.codedMagnitude()) {
			return (Rotator) this.multiply(omega_j.cotHalf()).addRightMultiplyJ(t)
					.addMultiplyRightI(omega_i.tanHalf() / omega_j.tanHalf(), t);
			// return this.multiplyAddRightJ(thetaJ.cotHalf(), t)
			// .addMultiplyRightI(thetaI.tanHalf() / thetaJ.tanHalf(), t);
		}
		return (Rotator) this.multiply(omega_i.cotHalf()).addRightMultiplyI(t)
				.addMultiplyRightJ(omega_j.tanHalf() / omega_i.tanHalf(), t);
		// return this.multiplyAddRightI(thetaI.cotHalf(), t).addMultiplyRightJ(
		// thetaJ.tanHalf() / thetaI.tanHalf(), t);

	}

	/**
	 * Rank 2 Mutator.
	 * 
	 * @param direction
	 *            Vector3
	 * @return this right multiplied by Rotator defined by tilt of basis unit
	 *         <b>K</b> to align with <b>direction</b> vector3.
	 */
	public final Rotator tilt_k(final Vector3 direction) {
		// K==0...
		final Rotator t = new Rotator(this);
		// t.set(this);
		final double w = direction.getAbs() + direction.getZ();
		final double xAbs = StrictMath.abs(direction.getX());
	
		if (w > xAbs) {
			if (w > direction.getY()) { // K
				return (Rotator) this.addMultiplyRightI(direction.getY() / w, t)
						.addMultiplyRightJ(-direction.getX() / w, t);
			}
	
		} else {
			if (direction.getY() > xAbs) { // J
				return (Rotator) this.multiply(w / direction.getY()).addRightMultiplyI(t)
						.addMultiplyRightJ(-direction.getX() / direction.getY(), t);
				// return this.multiplyAddRightI(w / direction.getY(), t)
				// .addMultiplyRightJ(-direction.getX() / direction.getY(), t);
			}
		}
		// I
		return (Rotator) this.multiply(-w / direction.getX()).addRightMultiplyJ(t)
				.addMultiplyRightI(-direction.getY() / direction.getX(), t);
		// return this.multiplyAddRightJ(-w / direction.getX(), t)
		// .addMultiplyRightI(-direction.getY() / direction.getX(), t);
	}

	/**
	 * <B>Rotate 90-Degree-rotation about vector-axis mutator.</B> (45 degrees
	 * per multiplication)
	 * 
	 * @param v
	 *            Vector3 turn-axis
	 */
	public final Rotator turn(Vector3 v) {
		if (v.getNormInf() == 0)
			return new Rotator(Quaternion.EMPTY); // ????
		Vector3 u = new Vector3(v).unit(); // vector magnitude...
		Quaternion t = new Quaternion(this);
		return (Rotator) this.addMultiplyRightI(u.getX(), t).addMultiplyRightJ(u.getY(), t).addMultiplyRightK(u.getZ(),
				t);
	}

	/**
	 * <B>Mutator: Rotate 90-Degree right hand turn about UNIT_I axis.</B> (Rotator 45
	 * degrees per multiplication)
	 */
	public final Rotator turn_i() {
		return (Rotator) this.addRightMultiplyI(new Quaternion(this));
	}

	/**
	 * <B>Mutator: Rotate 90-Degree right hand turn about UNIT_J axis.</B> (Rotator 45
	 * degrees per multiplication)
	 */
	public final Rotator turn_j() {
		return (Rotator) this.addRightMultiplyJ(new Quaternion(this));
	}

	/**
	 * <B>Mutator: Rotate 90-Degree right hand turn about UNIT_K axis.</B> (Rotator 45
	 * degrees per multiplication)
	 */
	public final Rotator turn_k() {
		return (Rotator) this.addRightMultiplyK(new Quaternion(this));
	}

}
