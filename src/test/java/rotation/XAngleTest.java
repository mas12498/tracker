/**
 * 
 */
package rotation;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * @author mike
 *
 */
public class XAngleTest {

//	/**
//	 * @throws java.lang.Exception
//	 */
//	@BeforeEach
//	void setUp() {
//	}
	
//	
//	/**
//	 * Test method for {@link rotation.Angle#hashCode()}.
//	 */
//	@Test
//	void testHashCode() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#Angle(double)}.
//	 */
//	@Test
//	void testAngleDouble() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#Angle()}.
//	 */
//	@Test
//	void testAngle() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#Angle(rotation.Angle)}.
//	 */
//	@Test
//	void testAngleAngle() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#equals(java.lang.Object)}.
//	 */
//	@Test
//	void testEqualsObject() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inPiRadians(double)}.
//	 */
//	@Test
//	void testInPiRadians() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inRevolutions(double)}.
//	 */
//	@Test
//	void testInRevolutions() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inDegrees(double)}.
//	 */
//	@Test
//	void testInDegrees() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inRadians(double)}.
//	 */
//	@Test
//	void testInRadians() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inMeasure(double, double)}.
//	 */
//	@Test
//	void testInMeasure() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inBinaryArc(short, byte)}.
//	 */
//	@Test
//	void testInBinaryArcShortByte() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inBinaryArc(int, byte)}.
//	 */
//	@Test
//	void testInBinaryArcIntByte() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#inBinaryArc(long, byte)}.
//	 */
//	@Test
//	void testInBinaryArcLongByte() {
//		fail("Not yet implemented"); // TODO
//	}

	/**
	 * Test method for {@link rotation.Angle#signedPrinciple()}.
	 */
	@Test
	public void testSignedPrinciple() {
				
		final Angle straightPosMinus = Angle.inPiRadians(Math.nextDown(1d));
		final Angle straightPositive = Angle.inPiRadians(1d);
		final Angle straightPosPlus = Angle.inPiRadians(Math.nextUp(1d));

		assertTrue((straightPosMinus.signedPrinciple().getPiRadians() < 1d));
		//straight equality rolls sign to negative: No positive straight in signed principle window.
		//assertTrue((straightPositive.signedPrinciple().getPiRadians() < 0d));
		assertTrue((straightPositive.signedPrinciple().getPiRadians() == -1d));
		assertTrue((straightPosPlus.signedPrinciple().getPiRadians() > -1d));
		
		
		final Angle straightNegMinus = Angle.inPiRadians(Math.nextDown(-1d));
		final Angle straightnegative = Angle.inPiRadians(1d);
		final Angle straightNegPlus = Angle.inPiRadians(Math.nextUp(-1d));
		
		//straight negative decremented rolls positive
		assertTrue((straightNegMinus.signedPrinciple().getPiRadians() > 0d));
		assertTrue((straightNegMinus.signedPrinciple().getPiRadians() < 1d));
		assertTrue((straightnegative.signedPrinciple().getPiRadians() == -1d));
		assertTrue((straightNegPlus.signedPrinciple().getPiRadians() > -1d));
		
		
		final Angle zeroPositive = Angle.inPiRadians(0d);
		final Angle zeroPosPlus = Angle.inPiRadians(Math.nextUp(0d));
		final Angle zeroPosMinus = Angle.inPiRadians(Math.nextDown(0d));
		
		assertTrue((zeroPositive.equals(Angle.ZERO)));
		assertTrue((zeroPositive.signedPrinciple().getPiRadians() == 0d));		
		assertFalse((zeroPositive.equals(new Angle(Angle.ZERO).negate())));
		//zero decremented rolls negative:
		assertTrue((zeroPosMinus.signedPrinciple().getPiRadians() < 0d));
		assertTrue(((Double) (zeroPositive.signedPrinciple().getPiRadians())).equals(0d));		
		assertTrue((zeroPosPlus.signedPrinciple().getPiRadians() > 0d));

		
		final Angle zeronegative = Angle.inPiRadians(-0d);
		final Angle zeroNegPlus = Angle.inPiRadians(Math.nextUp(-0d));
		final Angle zeroNegMinus = Angle.inPiRadians(Math.nextDown(-0d));

		assertTrue((zeroNegMinus.signedPrinciple().getPiRadians() < 0d));
		assertTrue((zeronegative.equals(new Angle(Angle.ZERO).negate())));
		assertTrue((zeronegative.signedPrinciple().getPiRadians() == 0d));
		assertFalse(zeronegative.equals(Angle.ZERO));
		assertTrue(((Double) (zeronegative.signedPrinciple().getPiRadians())).equals(-0d));
		//negative zero incremented rolls positive
		assertTrue((zeroNegPlus.signedPrinciple().getPiRadians() > 0d));
		
		
//		final Angle rightPositive = Angle.inPiRadians(1d/2);
//		final Angle rightPosPlus = Angle.inPiRadians(Math.nextUp(1d/2));
//		final Angle rightPosMinus = Angle.inPiRadians(Math.nextDown(1d/2));
//		
//		final Angle rightnegative = Angle.inPiRadians(1d);
//		final Angle rightNegPlus = Angle.inPiRadians(Math.nextUp(-1d/2));
//		final Angle rightNegMinus = Angle.inPiRadians(Math.nextDown(-1d/2));

		
		
		//fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link rotation.Angle#unsignedPrinciple()}.
	 */
	@Test
	public void testUnsignedPrinciple() {
		
	final Angle straightPosMinus = Angle.inPiRadians(Math.nextDown(1d));
	final Angle straightPositive = Angle.inPiRadians(1.0d);
	final Angle straightPosPlus = Angle.inPiRadians(Math.nextUp(1.0d));

	assertTrue((straightPosMinus.unsignedPrinciple().getPiRadians() < 1d));
	//straight equality rolls sign to negative: No positive straight in signed principle window.
	//assertTrue((straightPositive.signedPrinciple().getPiRadians() < 0d));
	assertTrue((straightPositive.unsignedPrinciple().getPiRadians() == 1d));
	System.out.println(new Angle(straightPosPlus).unsignedPrinciple().getPiRadians());
	assertTrue((straightPosPlus.unsignedPrinciple().getPiRadians() >= 1d));
	assertTrue((StrictMath.nextUp(straightPosPlus.unsignedPrinciple().getPiRadians()) > 1d));
	
	
	final Angle straightNegMinus = Angle.inPiRadians(Math.nextDown(-1d));
	final Angle straightnegative = Angle.inPiRadians(1d);
	final Angle straightNegPlus = Angle.inPiRadians(Math.nextUp((-1d)));
	System.out.println(straightNegPlus.getPiRadians());
	
	//straight negative decremented rolls positive
	assertTrue((straightNegMinus.unsignedPrinciple().getPiRadians() > 0d));
	assertTrue((straightNegMinus.unsignedPrinciple().getPiRadians() < 1d));
	assertTrue((straightnegative.unsignedPrinciple().getPiRadians() == 1d));
	System.out.println(new Angle(straightNegPlus).unsignedPrinciple().getPiRadians());
	assertTrue((straightNegPlus.unsignedPrinciple().getPiRadians() == 1d));
	assertTrue((StrictMath.nextUp(straightNegPlus.unsignedPrinciple().getPiRadians()) > 1d));
	
	
	final Angle zeroPositive = Angle.inPiRadians(0d);
	final Angle zeroPosPlus = Angle.inPiRadians(Math.nextUp(0d));
	final Angle zeroPosMinus = Angle.inPiRadians(Math.nextDown(0d));
	
	assertTrue((zeroPositive.equals(Angle.ZERO)));
	assertTrue((zeroPositive.unsignedPrinciple().getPiRadians() == 0d));		
	assertFalse((zeroPositive.equals(new Angle(Angle.ZERO).negate())));
	//zero decremented rolls negative:
	assertTrue((zeroPosMinus.unsignedPrinciple().getPiRadians() > 0d));
	assertTrue((zeroPosMinus.unsignedPrinciple().getPiRadians() > 0d));

	assertTrue(((Double) (zeroPositive.unsignedPrinciple().getPiRadians())).equals(0d));		
	assertTrue((zeroPosPlus.unsignedPrinciple().getPiRadians() > 0d));

	
	final Angle zeronegative = Angle.inPiRadians(-0d);
	final Angle zeroNegPlus = Angle.inPiRadians(Math.nextUp(-0d));
	final Angle zeroNegMinus = Angle.inPiRadians(Math.nextDown(-0d));

	assertTrue((zeroNegMinus.unsignedPrinciple().getPiRadians() > 0d));
	assertTrue((zeronegative.equals(new Angle(Angle.ZERO).negate())));
	assertTrue((zeronegative.unsignedPrinciple().getPiRadians() == 0d));
	assertFalse(zeronegative.equals(Angle.ZERO));
	assertTrue(((Double) (zeronegative.unsignedPrinciple().getPiRadians())).equals(0d));
	//negative zero incremented rolls positive
	assertTrue((zeroNegPlus.unsignedPrinciple().getPiRadians() > 0d));
	
	
	
	
	
		//fail("Not yet implemented"); // TODO
	}

//	/**
//	 * Test method for {@link rotation.Angle#codedPhase()}.
//	 */
//	@Test
//	void testCodedPhase() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getDegrees()}.
//	 */
//	@Test
//	void testGetDegrees() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getRadians()}.
//	 */
//	@Test
//	void testGetRadians() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getPiRadians()}.
//	 */
//	@Test
//	void testGetPiRadians() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getRevolutions()}.
//	 */
//	@Test
//	void testGetRevolutions() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#get(double)}.
//	 */
//	@Test
//	void testGet() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getBinaryArc(byte)}.
//	 */
//	@Test
//	void testGetBinaryArc() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getShortBinaryArc(byte)}.
//	 */
//	@Test
//	void testGetShortBinaryArc() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#getLongBinaryArc(byte)}.
//	 */
//	@Test
//	void testGetLongBinaryArc() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#set(rotation.Angle)}.
//	 */
//	@Test
//	void testSetAngle() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setRadians(double)}.
//	 */
//	@Test
//	void testSetRadians() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setPiRadians(double)}.
//	 */
//	@Test
//	void testSetPiRadians() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setRevolutions(double)}.
//	 */
//	@Test
//	void testSetRevolutions() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setDegrees(double)}.
//	 */
//	@Test
//	void testSetDegrees() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#set(double, double)}.
//	 */
//	@Test
//	void testSetDoubleDouble() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setBinaryArc(short, byte)}.
//	 */
//	@Test
//	void testSetBinaryArcShortByte() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setBinaryArc(int, byte)}.
//	 */
//	@Test
//	void testSetBinaryArcIntByte() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#setBinaryArc(long, byte)}.
//	 */
//	@Test
//	void testSetBinaryArcLongByte() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#abs()}.
//	 */
//	@Test
//	void testAbs() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#negate()}.
//	 */
//	@Test
//	void testNegate() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#add(rotation.Angle)}.
//	 */
//	@Test
//	void testAdd() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#subtract(rotation.Angle)}.
//	 */
//	@Test
//	void testSubtract() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#multiply(double)}.
//	 */
//	@Test
//	void testMultiply() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#copySign(rotation.Angle)}.
//	 */
//	@Test
//	void testCopySign() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#toDegreesString(int)}.
//	 */
//	@Test
//	void testToDegreesString() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#toRadiansString(int)}.
//	 */
//	@Test
//	void testToRadiansString() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#toPiRadiansString(int)}.
//	 */
//	@Test
//	void testToPiRadiansString() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#toString()}.
//	 */
//	@Test
//	void testToString() {
//		fail("Not yet implemented"); // TODO
//	}
//
//	/**
//	 * Test method for {@link rotation.Angle#main(java.lang.String[])}.
//	 */
//	@Test
//	void testMain() {
//		fail("Not yet implemented"); // TODO
//	}

}
