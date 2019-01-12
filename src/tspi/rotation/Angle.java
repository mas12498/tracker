package tspi.rotation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import junit.framework.Assert;

/**
 * Angle stores rotation measures:
 *  <br>-- gets and sets signed angle units:
 *  <br>---- piRadians, revolutions, degrees, radians, 
 *  <br>---- user-scaled units defined per straight angle measures
 *  <br>---- user-binary units defined per count bits encoding an arc-circle
 *  <br>---- NaN is 'empty' Angle assignment.
 *  <br>-- rotation measures stored may span revolutions.
 *  <br>-- limited Angle linear algebra support:
 *  <br>---- mutators add, subtract, abs, copySign, negate
 *  <br>-- factory for principle argument of stored rotation:
 *  <br>---- signed and unsigned principle Angles
 *  <br>---- CodedPhase angles
 */
public class Angle
{	
	public final static double PIRADIANS_REVOLUTION  = 2d;	
	public final static double PIRADIANS_STRAIGHT    = 1d;	
	public final static double PIRADIANS_RIGHT       = 1d/2;	
	public final static double PIRADIANS_ZERO        = 0d;		
	public final static double PIRADIANS_EMPTY       = Double.NaN;		
	public final static double RADIANS_STRAIGHT      = StrictMath.PI;
	public final static double DEGREES_STRAIGHT      = 180d;	
	public final static double ARCMINUTES_STRAIGHT   = DEGREES_STRAIGHT*60;
	public final static double ARCSECONDS_STRAIGHT   = DEGREES_STRAIGHT*3600;	
	public final static double MIL_STRAIGHT          = 3200d;	
	public final static double MILS_STRAIGHT         = 3000d;
	public final static double MILLIRADIANS_STRAIGHT = RADIANS_STRAIGHT*1000;	
	
	//Stored Angles: Native Pi Radians.
	public final static Angle REVOLUTION         = new Angle(PIRADIANS_REVOLUTION);
	public final static Angle STRAIGHT           = new Angle(PIRADIANS_STRAIGHT);
	public final static Angle RIGHT              = new Angle(PIRADIANS_RIGHT);
	public final static Angle ZERO               = new Angle(PIRADIANS_ZERO);
	public final static Angle EMPTY              = new Angle(PIRADIANS_EMPTY);	


    /** Angle static factory for int binary fractions expression of revolution. 
	 * @param binaryAngle integer (count of fractional lsb covering arc measure).
	 * @param numBitsRevolution byte number of significant bits required in binary circle-arc representation. */
	public static Angle inBinaryArc(int binaryAngle, byte numBitsRevolution)
	{ return new Angle(( StrictMath.scalb((double) binaryAngle, (byte)(1-numBitsRevolution)) )); } 
	    
	/** Angle static factory for long binary fractions expression of revolution. 
	 * @param binaryAngle long integer (count of fractional lsb covering arc measure).
	 * @param numBitsRevolution byte number of significant bits required in binary circle-arc representation. */
	public static Angle inBinaryArc(long binaryAngle, byte numBitsRevolution)
	{ return new Angle(( StrictMath.scalb((double) binaryAngle, (byte)(1-numBitsRevolution)) )); }

	/** Angle static factory for short binary fractional units expression of revolution. 
	 * @param binaryAngle short integer (count of fractional lsb covering arc measure).
	 * @param numBitsRevolution byte numbering significant bits required in binary circle-arc representation. */
	public static Angle inBinaryArc(short binaryAngle, byte numBitsRevolution)
	{ return new Angle(( StrictMath.scalb((double) binaryAngle, (byte)(1-numBitsRevolution)) )); }
	
	/** Angle static factory. 
	 * @param degrees double angle measure.*/
	public static Angle inDegrees(double degrees)
	{ return new Angle(degrees/DEGREES_STRAIGHT); }
		
	/** Angle static factory for any specified units... 
	 * @param unitsAngle double units arc measure.
	 * @param unitsStraightAngle double units of straight angle arc measure. */
	public static Angle inMeasure(double unitsAngle, double unitsStraightAngle)
	{ return new Angle( unitsAngle/unitsStraightAngle ); }
	
	/** Angle static factory. 
	 * @param piRadians double angle measure.*/
	public static Angle inPiRadians(double piRadians)
	{ return new Angle(piRadians); }

	/** Angle static factory. 
	 * @param radians double angle measure.*/
	public static Angle inRadians(double radians)
	{ return new Angle(radians/RADIANS_STRAIGHT); }
 
	/** Angle static factory. 
	 * @param revolutions double angle measure.*/
	public static Angle inRevolutions(double revolutions) 
	{ return new Angle(StrictMath.scalb(revolutions, 1)); }
 
	@SuppressWarnings("deprecation")
	public static void main(String args[]) 
//	public static void main() 
	{
		//test protected functions...
		
		Angle test = new Angle(1/2.d);		
		Assert.assertEquals(test.getPiRadians(),1d/2,1e-17);
		Assert.assertEquals(test.getCodedPhaseDouble(),1d,1e-15);
		
		test = new Angle(0.d);
		Assert.assertEquals(test.getPiRadians(),0d,1e-17);
		Assert.assertEquals(test.getCodedPhaseDouble(),0d,1e-17);

		test = new Angle(-1/2.d);
		Assert.assertEquals(test.getPiRadians(),-1d/2,1e-17);
		Assert.assertEquals(test.getCodedPhaseDouble(),-1d,1e-15);

		
		test = new Angle(1.d);
		Assert.assertTrue(Double.isInfinite(test.getCodedPhaseDouble()));
		
		test = new Angle(-1/2.d);
		Assert.assertEquals(test.getCodedPhaseDouble(),-1d,1e-15);
		
		test = new Angle(-1.d);
		Assert.assertTrue(Double.isInfinite(test.getCodedPhaseDouble()));

		test = new Angle(-0.d);
		Assert.assertEquals(test.getPiRadians(),0d,1e-17);
		Assert.assertEquals(test.getCodedPhaseDouble(),0d,1e-17);

		test = new Angle(Double.POSITIVE_INFINITY);
		Assert.assertEquals(test.getPiRadians(),Double.POSITIVE_INFINITY,1e-17);

		test = new Angle(Double.NEGATIVE_INFINITY);
		Assert.assertEquals(test.getPiRadians(),Double.NEGATIVE_INFINITY,1e-17);
		
		System.out.println("atan2(0,0)="+StrictMath.atan2(0, 0));
		System.out.println("atan2(-0,-0)="+StrictMath.atan2(-0, -0));
		System.out.println("atan2(-0,0)="+StrictMath.atan2(-0, 0));
		System.out.println("atan2(0,-0)="+StrictMath.atan2(0, -0));
		System.out.println("atan2(-0,INF)="+StrictMath.atan2(-0, Double.POSITIVE_INFINITY));
		System.out.println("atan2(0,INF)="+StrictMath.atan2(0, Double.POSITIVE_INFINITY));
		System.out.println("atan2(-0,-INF)="+StrictMath.atan2(-0, Double.NEGATIVE_INFINITY)/StrictMath.PI);
		System.out.println("atan2(0,-INF)="+StrictMath.atan2(0, Double.NEGATIVE_INFINITY)/StrictMath.PI);
		System.out.println("Math.PI="+StrictMath.PI + " " + (StrictMath.atan2(0, Double.NEGATIVE_INFINITY)==StrictMath.PI));
		
		System.out.println("atan2(INF,INF)="+StrictMath.toDegrees(StrictMath.atan2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)));
		System.out.println("atan2(INF,-0)="+StrictMath.toDegrees(StrictMath.atan2(Double.POSITIVE_INFINITY,0)));
		System.out.println("atan2(INF,-INF)="+StrictMath.toDegrees(StrictMath.atan2(Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY)));
		System.out.println("atan2(-INF,INF)="+StrictMath.toDegrees(StrictMath.atan2(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)));
		System.out.println("atan2(-INF,-0)="+StrictMath.toDegrees(StrictMath.atan2(Double.NEGATIVE_INFINITY,0)));
		System.out.println("atan2(-INF,-INF)="+StrictMath.toDegrees(StrictMath.atan2(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY)));
		System.out.println("(-0.0 < 0.0) = "+(-0.0<0.0));
		

			System.out.println("********Test Coded Phase support");
			Angle a = new Angle();
			assertEquals(Double.isNaN(a.getPiRadians()), true);
			a.set(Angle.EMPTY);
			assertEquals(Double.isNaN(a.getPiRadians()), true);

			Angle b = new Angle();
			double piRad;

			CodedPhase t = new CodedPhase(b.codedPhase());
			CodedPhase u = new CodedPhase(b.codedPhase());

			for (int i = -128; i <= 128; i++) {
				piRad = i / 8d;
				b = Angle.inPiRadians(piRad);
				a.set(b);
				t = b.codedPhase();
				assertEquals(b.getCodedPhaseDouble(), t.tanHalf(), 1e-18);
				u.set(b.codedPhase());
				assertEquals(u.tanHalf(), t.tanHalf(), 1e-18);
				 System.out.println(StrictMath.tan(a.signedPrinciple().getRadians()/2)+" in Pi Radians: "+ a.signedPrinciple().getPiRadians() +" unsigned: "+ a.unsignedPrinciple().getPiRadians());
				// System.out.println(2*StrictMath.atan(a.getCodedPhaseDouble())+" decoded "+t.getRadians());
				assertEquals(2 * StrictMath.atan(a.getCodedPhaseDouble()), t.angle().getRadians(), 1e-18);
				assertEquals(a.codedPhase().angle().getRadians(), t.angle().getRadians(), 1e-18);
			}

			double i;
			a = Angle.inDegrees(Double.NaN);
			for (int h = -32; h <= 32; h++) {
				i = h * 22.5;
				a = Angle.inDegrees(i);
				// System.out.println("("+i+")"
				// +" Unsigned Angle = "+a.unsignedPrinciple().toDegreesString(17)
				// +" floor rev = "+ (long)
				// StrictMath.floor(Angle.inDegrees(i).getRevolutions())
				// +" Signed Angle = "+a.signedPrinciple().toDegreesString(17)
				// +" near rev =
				// "+StrictMath.round(Angle.inDegrees(i).getRevolutions()) );
				assertEquals(StrictMath.round(Angle.inDegrees(i).getRevolutions()) + a.signedPrinciple().getRevolutions(),
						a.unsignedPrinciple().getRevolutions() + StrictMath.floor(Angle.inDegrees(i).getRevolutions()),
						1e-14);
			}

			a = Angle.inDegrees(180);
			assertTrue(Double.isInfinite(a.getCodedPhaseDouble()));
			a = Angle.inDegrees(-180);
			assertTrue(Double.isInfinite(a.getCodedPhaseDouble()));

			a = Angle.inDegrees(390);
			b = new Angle(a);

			CodedPhase g = b.codedPhase();

			t = new CodedPhase(Angle.inDegrees(0).codedPhase());
			assertEquals(t.angle().signedPrinciple().getDegrees(), 0, 1e-13);

			t = new CodedPhase(a.codedPhase());
			assertEquals(t.angle().signedPrinciple().getDegrees(), 30, 1e-13);

			assertEquals(t.angle().getRadians(), g.angle().getRadians(), 1e-14);
			assertEquals(t.angle().getPiRadians(), g.angle().getPiRadians(), 1e-14);
			assertEquals(t.angle().signedPrinciple().getPiRadians(), g.angle().signedPrinciple().getPiRadians(), 1e-13);
			assertEquals(t.angle().signedPrinciple().getRevolutions(), g.angle().signedPrinciple().getRevolutions(), 1e-13);
			assertEquals(t.angle().signedPrinciple().get(180), g.angle().signedPrinciple().get(Angle.DEGREES_STRAIGHT),
					1e-13);

			b.set(t.angle().signedPrinciple()); // convert principle angle to
												// angle...signed and unsigned.

			Angle c = Angle.inRadians(t.angle().signedPrinciple().getRadians());
			assertEquals(b.getDegrees(), 30, 1e-13);
			assertEquals(c.getDegrees(), 30, 1e-13);
		
		
			System.out.println("radinas conv const  error == "+(RADIANS_STRAIGHT-StrictMath.atan(1)*4)*10000000000000.0); //
		
		System.out.println("**** Done Angle main.");

	}
		
	private double _piRadiansAngle;
	
	/** Empty Angle constructor. */
	public Angle() 
	{ _piRadiansAngle = PIRADIANS_EMPTY; }
	
	
	/** Copy constructor. */
	public Angle(Angle a) 
	{ _piRadiansAngle = a._piRadiansAngle; }
	
	/** Internal native constructor -- assumes PiRadians... */
	protected Angle(double piRadians) 
	{ _piRadiansAngle = piRadians; }
	
	/** Mutator. Returns this absolute magnitude value. */
	public Angle abs()
	{ 
		_piRadiansAngle = StrictMath.abs(_piRadiansAngle); 
		return this; 
	}
	
	//@MAS: Add exceptions for input overflows and underflows [?] 

	/** Mutator. Returns this plus addend.*/
	public Angle add(Angle addend)
	{
		this._piRadiansAngle += addend._piRadiansAngle;
		return this;
	}
	
	/** CodedPhase Factory: Create new <i>CodedPhase</i> encoded of this Angle measure. */
	public CodedPhase codedPhase() 
	{ // [-inf..+inf]
		return CodedPhase.encodes(this.getCodedPhaseDouble());	
	}

	/** Mutator. Returns this with copied sign of that. */
	public Angle copySign(Angle that){
		_piRadiansAngle = StrictMath.copySign(_piRadiansAngle, that._piRadiansAngle);
		return this;
	}
	
	@Override
	public boolean equals(Object obj) 
	{
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Angle other = (Angle) obj;
		if (Double.doubleToLongBits(_piRadiansAngle) != Double.doubleToLongBits(other._piRadiansAngle))
			return false;
		return true;
	}
	
	/** Get <i>units measure</i> of this <b><i>Angle</b></i> given units measured in a straight angle. 
	 * @param unitsInStraightAngle
	 */
	public double get(double unitsInStraightAngle)
	{ return _piRadiansAngle*unitsInStraightAngle; }
	
	/** Get <i>int measure</i> defined from <i>binary</i> angle representation of this <b><i>Angle</b></i> measure. 
	 * @param number_bits_encoding_representation of arc circle.
	 */
	public int getBinaryArc(byte number_bits_encoding_representation)
	{ //need handle errors, max and min limits...
		return (int) StrictMath.rint(StrictMath.scalb(_piRadiansAngle,number_bits_encoding_representation - 1)); 
	}

	/** Get coded phase double: 
	 * <br>-- double <i>coded Angle value : [-inf..inf]</i> of PiRadian phase angle: [-1..1]
	 * <br>-- NaN implies empty Angle is coded.  */
	private double getCodedPhaseDouble() //exact binary angle fractions
	{ // [-inf..inf]
		double r = StrictMath.scalb(getFractionalPiRadians(),-1);
		if (r==0) return r; 
		if (StrictMath.abs(r) == 1/2d) return StrictMath.copySign(Double.POSITIVE_INFINITY, r); //straight angle
		if (StrictMath.abs(r) == 1/4d) return StrictMath.copySign(1d, r); //orthogonal angle
		return StrictMath.tan(r * RADIANS_STRAIGHT); //Note: NaN propagates empty angle.
	}
	
	public double getDegrees()
	{ return _piRadiansAngle * DEGREES_STRAIGHT; }
	
	private double getFractionalPiRadians()
	{ return StrictMath.IEEEremainder(this._piRadiansAngle, PIRADIANS_REVOLUTION); }
	
	/** Get <i>long measure</i> defined from <i>binary</i> angle representation of this <b><i>Angle</b></i> measure. 
	 * @param number_bits_encoding_representation of arc circle.
	 */
	public long getLongBinaryArc(byte number_bits_encoding_representation)
	{ //need handle errors, max and min limits...
		return (long) StrictMath.rint(StrictMath.scalb(_piRadiansAngle, number_bits_encoding_representation - 1)); 
	}	

	public double getPiRadians()
	{ return _piRadiansAngle; }
		
	public double getRadians()
	{ return _piRadiansAngle * RADIANS_STRAIGHT; }
	
	public double getRevolutions()
	{ return StrictMath.scalb(_piRadiansAngle, -1); }
	
	//Setters:
	
	/** Get <i>short measure</i> defined from <i>binary</i> angle representation of this <b><i>Angle</b></i> measure. 
	 * @param number_bits_encoding_representation of arc circle.
	 */
	public short getShortBinaryArc(byte number_bits_encoding_representation)
	{ //need handle errors, max and min limits...
		return (short) StrictMath.rint(StrictMath.scalb(_piRadiansAngle, number_bits_encoding_representation - 1)); 
	}
	
	@Override
	public int hashCode() 
	{
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(_piRadiansAngle);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}	

	/** Mutator. Returns this multiplied by scalar factor.*/
	public Angle multiply(double scalar)
	{
		this._piRadiansAngle *= scalar;
		return this;
	}	
	
	/** Mutator. Returns this negated. */
	public Angle negate()
	{
		_piRadiansAngle = -_piRadiansAngle;
		return this;
	}
	
	public void set(Angle copy) 
	{ _piRadiansAngle = copy._piRadiansAngle; }

    /**  Set equal to measured units of angle given count units defined in straight angle.*/ 
	public void set(double unitsMeasured, double unitsInStraightAngle)
	{ _piRadiansAngle=( unitsMeasured / unitsInStraightAngle ); }

	
	/** 
	 * Set equal to <b>binaryAngle</b> given number of <b>angleBits</b> define representations of a complete arc circle. 
	 */
	public void setBinaryArc(int binaryAngle, byte angleBits) 
	{ _piRadiansAngle = StrictMath.scalb((double) binaryAngle, 1 - angleBits); }

	/** 
	 * Set equal to <b>binaryAngle</b> given number of <b>angleBits</b> which define representation of a whole arc circle. 
	 */
	public void setBinaryArc(long binaryAngle, byte angleBits) 
	{ _piRadiansAngle = StrictMath.scalb((double) binaryAngle, 1 - angleBits); }

	/** 
	 * Set equal to <b>binaryAngle</b> given number of <b>angleBits</b> define representations of a complete arc circle. 
	 */
	public void setBinaryArc(short binaryAngle, byte angleBits) 
	{ _piRadiansAngle = StrictMath.scalb((double) (binaryAngle), 1 - angleBits); }
	
	// Mutators:
	
	public void setDegrees(double degreesRotation)
	{ _piRadiansAngle = degreesRotation / DEGREES_STRAIGHT; }

	public void setPiRadians(double piRadiansRotation)
	{ _piRadiansAngle = piRadiansRotation; }

	public void setRadians(double radiansRotation)
	{ _piRadiansAngle = radiansRotation / RADIANS_STRAIGHT; }
	
	public void setRevolutions(double revolutionsRotation) 
	{ _piRadiansAngle = StrictMath.scalb(revolutionsRotation,1); }
	
	/** Angle Factory: Create new <i>signed principle Angle </i> equal to residual of round (nearest) revolutions from this measure. 
	 * Round-up all half-angles.*/
	public Angle signedPrinciple() //Compactly spans baseband revolution ring...
	{ // [-1..+1)
		double piRadAngle = getFractionalPiRadians();                               //limits angle units: [-1..1]
		if(piRadAngle==PIRADIANS_STRAIGHT) return inPiRadians(-PIRADIANS_STRAIGHT); //forces negative straight: [-1..1)
		return Angle.inPiRadians(piRadAngle);                                       //factory
	}
	
	/** Mutator. Returns this minus subtrahend.*/
	public Angle subtract(Angle subtrahend)
	{
		this._piRadiansAngle -= subtrahend._piRadiansAngle;
		return this;
	}
	
	//Display help...
	
	public String toDegreesString(int decimals) 
	{
		String fmt = "%20."+decimals+"f";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, getDegrees());
	}
	
	public String toPiRadiansString(int decimals) 
	{
		String fmt = "%."+decimals+"f";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, getPiRadians());
	}
	
	public String toRadiansString(int decimals) 
	{
		String fmt = "%."+decimals+"f";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, getRadians());
	}
	
	@Override
	public String toString() 
	{
		int decimals=14;
		return toDegreesString(decimals);
	}	
		
	
	/** Angle Factory: Create new <i>unsigned principle Angle</i> equal to residual of floor revolutions from this measure. */
	public Angle unsignedPrinciple() 
	{ //Compactly spans principle revolution ring in pi radians: [0..2)
		double piRadAngle = getFractionalPiRadians();   //yields: [-1..1]		
		if(piRadAngle==PIRADIANS_ZERO) {  return inPiRadians(PIRADIANS_ZERO); } //forces positive zero
		if(StrictMath.abs(piRadAngle)==PIRADIANS_STRAIGHT) { return inPiRadians(PIRADIANS_STRAIGHT); } // Straight is now positive ...
		return Angle.inPiRadians((piRadAngle<0d) ? StrictMath.nextDown(piRadAngle) + PIRADIANS_REVOLUTION : piRadAngle); //forces positive: [0..2.0)
	}
	
//	/**
//	 * Get <i>int</i>: <i>nearest full revolution count</i> of stored
//	 * <b>Angle</b>.
//	 */
//	public int getNearestRevolution() {
//		return (int) StrictMath.round(angle / REVOLUTION_RADIANS);
//	}
//	/**
//	 * Get <i>double</i>: Principle <i>radians</i> of stored
//	 * <b>Angle</b>.
//	 */


//	/** Get <i>int</i>: <i>binary</i> angle representation of <b><i>Angle</b></i>. 
//	 * @param byte <b>number_bits</b> coded in angle of revolution.
//	 */
//	public long getLongBinary(byte number_bits)
//	{ return (long) StrictMath.rint(StrictMath.scalb(angle/QUARTER_PI_RADIANS,number_bits-3)); }

		
}
