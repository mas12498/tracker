/**
 * 
 */
package rotation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import rotation.Angle;

/**
 * Test rotation.Angle Adapter class for angle measure:
 * <p>
 * -- positive and negative units.
 * <p>
 * -- signed measures.
 * <p>
 * -- stores more than span of revolution.
 * 
 * @author mike
 *
 */
public class AngleTest {

	/**
	 * 
	 * Test method for {@link rotation.Angle#hashcode()}. Test method for
	 * {@link rotation.Angle#abs()}. Test method for
	 * {@link rotation.Angle#negate()}. Test method for
	 * {@link rotation.Angle#add(Angle)}. Test method for
	 * {@link rotation.Angle#subtract(Angle)}. Test method for
	 * {@link rotation.Angle#equal()}.
	 */
	@Test
	public void testAlgebra() {

		
		
		System.out.println("********Test hash abs negate add subtract equal");
		// init empty:Double.NaN
		Angle a = new Angle();
		Angle b = new Angle();
		Angle c = new Angle();
		
		
		c = Angle.inPiRadians(2);
		assertEquals(c.signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(-0d) );

		c = Angle.inPiRadians(-2);
		assertEquals(c.signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(-0d) );
		
		c = Angle.inPiRadians(0);
		assertEquals(c.signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(-0d) );

		c = Angle.inPiRadians(-0);
		assertEquals(c.signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.unsignedPrinciple().getPiRadians()).equals(-0d) );


		// hashcheck
		assertFalse(b.hashCode() == c.hashCode()); // empty
		
		// default creation..
		c = new Angle();		
		assertTrue(b.hashCode() == c.hashCode()); // empty		
		assertTrue(b.hashCode() == b.hashCode()); // empty
		assertFalse(new Angle(b).equals(null));

		b.setPiRadians(Double.POSITIVE_INFINITY);
		c.setPiRadians(Double.POSITIVE_INFINITY);
		assertTrue(b.hashCode() == c.hashCode());
		b.setPiRadians(Double.NEGATIVE_INFINITY);
		assertFalse(b.hashCode() == c.hashCode());
		c.setPiRadians(Double.NEGATIVE_INFINITY);
		assertTrue(b.hashCode() == c.hashCode());

		double piRad;
		double piRad2;
		for (int j = -128; j <= 128; j++) {
			piRad2 = j / 8d;
			for (int i = -128; i <= 128; i++) {
				piRad = i / 8d;
				b = Angle.inPiRadians(piRad);
				c = Angle.inPiRadians(piRad2);
				// copySign
				if (piRad2 >= 0) {
					assertEquals(new Angle(b).copySign(c).getPiRadians(), StrictMath.abs(piRad), 1e-17);
				} else {
					assertEquals(new Angle(b).copySign(c).getPiRadians(), -StrictMath.abs(piRad), 1e-17);
				}
				// equals, abs, negates
				if (piRad == piRad2) {
					assertEquals(b.hashCode(), c.hashCode());
					assertTrue(b.hashCode() == c.hashCode());
					assertTrue(b.equals(b));
					assertTrue(b.equals(c));
					assertTrue(c.equals(b));
					assertFalse(b.equals(c.getPiRadians()));
				} else if (piRad == -piRad2) {
					assertFalse(b.hashCode() == c.hashCode());
					assertTrue(b.hashCode() == new Angle(c).negate().hashCode());
					assertTrue(b.equals(new Angle(c).negate()));
					assertTrue(c.equals(new Angle(b).negate()));
				} else {
					assertFalse(b.hashCode() == c.hashCode());
					assertFalse(b.equals(c));
					assertFalse(c.equals(b));

				}
				// add
				assertEquals(new Angle(b).add(c).getPiRadians(), piRad + piRad2, 1e-17);
				// subtract
				assertEquals(new Angle(b).subtract(c).getPiRadians(), piRad - piRad2, 1e-17);
				// abs
				assertEquals(new Angle(b).abs().getPiRadians(), StrictMath.abs(piRad), 1e-17);
				// negate
				assertEquals(new Angle(b).negate().getPiRadians(), -piRad, 1e-17);
			}
		}
		
		b.set(Angle.inDegrees(5));
		c.set(b);
		b.multiply(2d);
		assertEquals(b.getDegrees(),2*c.getDegrees(),1e-15);		
		b.multiply(-1d);
		assertEquals(b.getDegrees(),-2*c.getDegrees(),1e-15);		
		
	}
	

	@Test
	public void testConstantsMain() {
		System.out.println("********Test Constants Main() Angle");

		Angle a = new Angle();

		assertEquals(Angle.STRAIGHT.getRadians(), StrictMath.PI, 1e-18);
		assertEquals(Angle.MILLIRADIANS_STRAIGHT, StrictMath.PI * 1000, 1e-18);
		assertEquals(Angle.MIL_STRAIGHT, 3200, 1e-18);
		assertEquals(Angle.MILS_STRAIGHT, 3000, 1e-18);
		assertEquals(Angle.STRAIGHT.getDegrees(), 180, 1e-18);
		assertEquals(Angle.ARCMINUTES_STRAIGHT, 180 * 60, 1e-18);
		assertEquals(Angle.ARCSECONDS_STRAIGHT, 180 * 3600, 1e-18);

		a.setPiRadians(1d);
		a.toString();
		a.toDegreesString(5);
		a.toRadiansString(14);
		a.toPiRadiansString(5);

		// System.out.println("**************** "+1d/Double.POSITIVE_INFINITY+"
		// "+1d/Double.NEGATIVE_INFINITY);
		// System.out.println("****************
		// "+StrictMath.toDegrees(StrictMath.atan(Double.POSITIVE_INFINITY))+"
		// "+StrictMath.toDegrees(StrictMath.atan(Double.NEGATIVE_INFINITY)));
		//
		// System.out.println("****************
		// "+2*StrictMath.atan(Double.NEGATIVE_INFINITY));
		// System.out.println("****************
		// "+2*StrictMath.atan(Double.POSITIVE_INFINITY));
		// System.out.println("**************** "+StrictMath.PI);
		//
		// System.out.println("********End of AngleTest exercises
		// *******************");

		// public final static double RADIANS_RIGHT_ANGLE =
		// RADIANS_STRAIGHT_ANGLE/2;
		// fail("Not yet implemented"); // TODO
		//boolean testZero;
		// testZero = (-0 == 0)?true:false;
		// System.out.println("neg zero equals pos zero:"+testZero);
		// testZero = (-0 < 0)?true:false;
		// System.out.println("neg less than pos zero:"+testZero);
		// testZero = (-0 < 0)?true:false;
		//Double num = 0d;
		//testZero = (num.equals(-0d)) ? true : false;
		// System.out.println("object zero same negZero:"+testZero);
		// System.out.println("object zero same negZero:"+testZero+"
		// "+num.compareTo(0d));
		//testZero = (num.compareTo(-0d) > 0) ? true : false;
		// System.out.println("object zero compares greater than
		// negZero:"+testZero+" "+num.compareTo(-0d));
		//num = -0d;
		// System.out.println("object negZero to one:"+num.compareTo(1d));
		// System.out.println("object negZero to zero:"+num.compareTo(0d));
		// System.out.println("object negZero to negZero:"+num.compareTo(-0d));
		// System.out.println("object negZero to negOne:"+num.compareTo(-1d));
		//num = 0d;
		// System.out.println("object zero to one:"+num.compareTo(1d));
		// System.out.println("object zero to zero:"+num.compareTo(0d));
		// System.out.println("object zero to negZero:"+num.compareTo(-0d));
		// System.out.println("object zero to negOne:"+num.compareTo(-1d));

		String[] args = new String[4];
		Angle.main(args);

	}

	/**
	 * Input and output of arbitrary binary arc-circle Angle units.
	 * <br>factory {@link rotation.Angle#inBinaryArc(int, byte)}. 
	 * <br>getter {@link rotation.Angle#getBinaryArc(byte)}.
	 * <br>setter {@link rotation.Angle#setBinaryArc(int, byte)}. 
	 */
	@Test
	public void testGetSetInBinary() {
		System.out.println("********Test Get Set Binary Arc Measures");

		Angle a = new Angle();
		Angle b = new Angle();

		int m;
		short s;
		long n;
		for (byte w = 4; w <= 16; w++) {
			for (short i = -96; i <= 96; i++) {
				s = (short) (i * 1024);
				b = Angle.inBinaryArc(s, w);
				assertEquals(b.getBinaryArc(w), s);
				assertEquals(b.getShortBinaryArc(w), s);				a.setBinaryArc((short) (b.getBinaryArc(w)), w);
				assertEquals(b.getLongBinaryArc(w), s);				a.setBinaryArc((short) (b.getBinaryArc(w)), w);
				assertEquals(a.getBinaryArc(w), s);
				assertEquals(a.getPiRadians(), StrictMath.scalb(s, -(w - 1)), 1e-12);
			}
		}

		for (byte w = 4; w <= 16; w++) {
			for (int i = -96; i <= 96; i++) {
				m = i * 8192;
				b = Angle.inBinaryArc(m, w);
				assertEquals(b.getBinaryArc(w), m);
				a.setBinaryArc((b.getBinaryArc(w)), w);
				assertEquals(a.getBinaryArc(w), m);
				assertEquals(a.getPiRadians(), StrictMath.scalb(m, -(w - 1)), 1e-12);
			}
		}

		for (byte w = 4; w <= 21; w++) {
			for (long i = -96; i <= 96; i++) {
				n = i * 8192;
				b = Angle.inBinaryArc(n, w);
				assertEquals(b.getBinaryArc(w), n);
				a.setBinaryArc((long) (b.getBinaryArc(w)), w);
				assertEquals(a.getBinaryArc(w), n);
				assertEquals(a.getPiRadians(), StrictMath.scalb(n, -(w - 1)), 1e-12);
			}
		}

	}

	/**
	 * Protractor Angle units: Degrees.
	 * 
	 * <br>factory {@link rotation.Angle#inDegrees(double)}.
	 * <br>getter {@link rotation.Angle#getDegrees()}. 
	 * <br>setter {@link rotation.Angle#setDegrees(double)}. 
	 */
	@Test
	public void testGetSetInDegrees() {
		System.out.println("********Test Get Set In Degrees");

		Angle a = new Angle();
		assertEquals(Double.isNaN(a.getDegrees()), true);
		a.set(Angle.EMPTY);
		assertEquals(Double.isNaN(a.getDegrees()), true);

		Angle b = new Angle();
		double deg;

		for (int i = -96; i <= 96; i++) {
			deg = i * 15;
			b = Angle.inDegrees(deg);
			assertEquals(b.getDegrees(), deg, 1e-13);
			a.setDegrees(b.getDegrees());
			assertEquals(a.getDegrees(), b.getPiRadians() * Angle.STRAIGHT.getDegrees(), 1e-13);
		}
	}

	/**
	 * Input and output of arbitrary Angle units.
	 * 
	 * <br>factory {@link rotation.Angle#inMeasure(double, double)}. 
	 * <br>setter {@link rotation.Angle#set(double, double)}. 
	 * <br>getter {@link rotation.Angle#get(double)}.
	 */
	@Test
	public void testGetSetInMeasure() {
		System.out.println("********Test Get Set In generic angle measures");

		// Degrees Measure
		Angle a = new Angle();
		assertEquals(Double.isNaN(a.getDegrees()), true);
		a.set(Double.NaN, Angle.STRAIGHT.getDegrees());
		assertEquals(Double.isNaN(a.get(Angle.STRAIGHT.getDegrees())), true);

		Angle b = new Angle();
		double deg;
		for (int i = -96; i <= 96; i++) {
			deg = i * 15;
			b = Angle.inMeasure(deg, Angle.STRAIGHT.getDegrees());
			assertEquals(b.get(Angle.STRAIGHT.getDegrees()), deg, 1e-13);
			a.set(b.get(Angle.STRAIGHT.getDegrees()), Angle.STRAIGHT.getDegrees());
			assertEquals(a.get(Angle.STRAIGHT.getDegrees()), b.getPiRadians() * Angle.STRAIGHT.getDegrees(), 1e-13);
		}

		// Radians Measure
		double rad;
		for (int i = -96; i <= 96; i++) {
			rad = i / 8;
			b = Angle.inMeasure(rad, Angle.STRAIGHT.getRadians());
			assertEquals(b.get(Angle.STRAIGHT.getRadians()), rad, 1e-13);
			a.set(b.get(Angle.STRAIGHT.getRadians()), Angle.STRAIGHT.getRadians());
			assertEquals(a.get(Angle.STRAIGHT.getRadians()), b.getPiRadians() * Angle.STRAIGHT.getRadians(), 1e-13);
		}

		// Milli-Radians Measure
		double mrad;
		for (int i = -96; i <= 96; i++) {
			mrad = i * 100;
			b = Angle.inMeasure(mrad, Angle.MILLIRADIANS_STRAIGHT);
			assertEquals(b.get(Angle.MILLIRADIANS_STRAIGHT), mrad, 1e-12);
			a.set(b.get(Angle.MILLIRADIANS_STRAIGHT), Angle.MILLIRADIANS_STRAIGHT);
			assertEquals(a.get(Angle.MILLIRADIANS_STRAIGHT), b.getPiRadians() * Angle.MILLIRADIANS_STRAIGHT,
					1e-12);
		}

		// Mil Measure
		double mil;
		for (int i = -96; i <= 96; i++) {
			mil = i * 100;
			b = Angle.inMeasure(mil, Angle.MIL_STRAIGHT);
			assertEquals(b.get(Angle.MIL_STRAIGHT), mil, 1e-12);
			a.set(b.get(Angle.MIL_STRAIGHT), Angle.MIL_STRAIGHT);
			assertEquals(a.get(Angle.MIL_STRAIGHT), b.getPiRadians() * Angle.MIL_STRAIGHT, 1e-12);
		}

		// Mils Measure
		double mils;
		for (int i = -96; i <= 96; i++) {
			mils = i * 100;
			b = Angle.inMeasure(mils, Angle.MILS_STRAIGHT);
			assertEquals(b.get(Angle.MILS_STRAIGHT), mils, 1e-12);
			a.set(b.get(Angle.MILS_STRAIGHT), Angle.MILS_STRAIGHT);
			assertEquals(a.get(Angle.MILS_STRAIGHT), b.getPiRadians() * Angle.MILS_STRAIGHT, 1e-12);
		}

	}

	/**
	 * PiRadians are internal class private units: Half-revolutions.
	 * 
	 * <br>factory {@link rotation.Angle#inPiRadians(double)}.
	 * <br>getter {@link rotation.Angle#getPiRadians()}. 
	 * <br>setter {@link rotation.Angle#setPiRadians(double)}. 
	 */
	@Test
	public void testGetSetInPiRadians() {

		System.out.println("********Test Get Set In PiRadians");

		// Empty angle: Double.NaN
		Angle a = new Angle();
		assertEquals(Double.isNaN(a.getPiRadians()), true);
		a.set(Angle.EMPTY);
		assertEquals(Double.isNaN(a.getPiRadians()), true);

		Angle b = new Angle();
		double piRad;

		for (int i = -128; i <= 128; i++) {
			piRad = i / 8d;
			b = Angle.inPiRadians(piRad);
			assertEquals(b.getPiRadians(), piRad, 1e-13);
			a.setPiRadians(b.getPiRadians());
			assertEquals(a.getPiRadians(), b.getPiRadians() * Angle.STRAIGHT.getPiRadians(), 1e-13);
		}
	}

	/**
	 * Units for java math class trigonometry functions: Radians.
	 * 
	 * <br>factory {@link rotation.Angle#inRadians(double)}.
	 * <br>getter {@link rotation.Angle#getRadians()}. 
	 * <br>setter {@link rotation.Angle#setRadians(double)}. 
	 */
	@Test
	public void testGetSetInRadians() {
		System.out.println("********Test Get Set In Radians");

		Angle a = new Angle();
		assertEquals(Double.isNaN(a.getRadians()), true);
		a.set(Angle.EMPTY);
		assertEquals(Double.isNaN(a.getRadians()), true);

		Angle b = new Angle();
		double rad;

		for (int i = -96; i <= 96; i++) {
			rad = i / 4d;
			b = Angle.inRadians(rad);
			assertEquals(b.getRadians(), rad, 1e-13);
			a.setRadians(b.getRadians());
			assertEquals(a.getRadians(), b.getPiRadians() * Angle.STRAIGHT.getRadians(), 1e-13);
		}
		
		b = Angle.inRadians(StrictMath.atan(Double.POSITIVE_INFINITY));
		assertTrue(b.getPiRadians()==Angle.RIGHT.getPiRadians());
		b = Angle.inRadians(StrictMath.atan2(0d,Double.NEGATIVE_INFINITY));
//		System.out.println("Test Dif = " + (StrictMath.atan2(0d,-1d) ));
		assertTrue(b.getPiRadians()==Angle.STRAIGHT.getPiRadians());
		
	}

//	/**
//	 * Test {@link rotation.Angle#getCodedPhaseDouble()}. <br>
//	 * Test {@link rotation.Angle#signedPrinciple()}. <br>
//	 * Test {@link rotation.Angle#unsignedPrinciple()}. <br>
//	 * Test {@link rotation.Angle#codedPhase()}.
//	 */
//	@Test
//	public void testGetCodedPrinciple() {
//		System.out.println("********Test Coded Phase support");
//		Angle a = new Angle();
//		assertEquals(Double.isNaN(a.getPiRadians()), true);
//		a.set(Angle.EMPTY);
//		assertEquals(Double.isNaN(a.getPiRadians()), true);
//
//		Angle b = new Angle();
//		double piRad;
//
//		CodedPhase t = new CodedPhase(b.codedPhase());
//		CodedPhase u = new CodedPhase(b.codedPhase());
//
//		for (int i = -128; i <= 128; i++) {
//			piRad = i / 8d;
//			b = Angle.inPiRadians(piRad);
//			a.set(b);
//			t = b.codedPhase();
//			assertEquals(b.getCodedPhaseDouble(), t.tanHalf(), 1e-18);
//			u.set(b.codedPhase());
//			assertEquals(u.tanHalf(), t.tanHalf(), 1e-18);
//			// System.out.println(StrictMath.tan(a.signedPrinciple().getRadians()/2)+"
//			// "+ a.signedPrinciple().getPiRadians());
//
//			// System.out.println(2*StrictMath.atan(a.getCodedPhaseDouble())+"
//			// decoded "+t.getRadians());
//			assertEquals(2 * StrictMath.atan(a.getCodedPhaseDouble()), t.angle().getRadians(), 1e-18);
//			assertEquals(a.codedPhase().angle().getRadians(), t.angle().getRadians(), 1e-18);
//		}
//
//		double i;
//		a = Angle.inDegrees(Double.NaN);
//		for (int h = -32; h <= 32; h++) {
//			i = h * 22.5;
//			a = Angle.inDegrees(i);
//			// System.out.println("("+i+")"
//			// +" Unsigned Angle = "+a.unsignedPrinciple().toDegreesString(17)
//			// +" floor rev = "+ (long)
//			// StrictMath.floor(Angle.inDegrees(i).getRevolutions())
//			// +" Signed Angle = "+a.signedPrinciple().toDegreesString(17)
//			// +" near rev =
//			// "+StrictMath.round(Angle.inDegrees(i).getRevolutions()) );
//			assertEquals(StrictMath.round(Angle.inDegrees(i).getRevolutions()) + a.signedPrinciple().getRevolutions(),
//					a.unsignedPrinciple().getRevolutions() + StrictMath.floor(Angle.inDegrees(i).getRevolutions()),
//					1e-14);
//		}
//
//		a = Angle.inDegrees(180);
//		assertTrue(Double.isInfinite(a.getCodedPhaseDouble()));
//		a = Angle.inDegrees(-180);
//		assertTrue(Double.isInfinite(a.getCodedPhaseDouble()));
//
//		a = Angle.inDegrees(390);
//		b = new Angle(a);
//
//		CodedPhase g = b.codedPhase();
//
//		t = new CodedPhase(Angle.inDegrees(0).codedPhase());
//		assertEquals(t.angle().signedPrinciple().getDegrees(), 0, 1e-13);
//
//		t = new CodedPhase(a.codedPhase());
//		assertEquals(t.angle().signedPrinciple().getDegrees(), 30, 1e-13);
//
//		assertEquals(t.angle().getRadians(), g.angle().getRadians(), 1e-14);
//		assertEquals(t.angle().getPiRadians(), g.angle().getPiRadians(), 1e-14);
//		assertEquals(t.angle().signedPrinciple().getPiRadians(), g.angle().signedPrinciple().getPiRadians(), 1e-13);
//		assertEquals(t.angle().signedPrinciple().getRevolutions(), g.angle().signedPrinciple().getRevolutions(), 1e-13);
//		assertEquals(t.angle().signedPrinciple().get(180), g.angle().signedPrinciple().get(Angle.DEGREES_STRAIGHT),
//				1e-13);
//
//		b.set(t.angle().signedPrinciple()); // convert principle angle to
//											// angle...signed and unsigned.
//
//		Angle c = Angle.inRadians(t.angle().signedPrinciple().getRadians());
//		assertEquals(b.getDegrees(), 30, 1e-13);
//		assertEquals(c.getDegrees(), 30, 1e-13);
//	}

	/**
	 * Revolutions are whole arc-circle units.
	 * 
	 * <br>factory {@link rotation.Angle#inRevolutions(double)}.
	 * <br>getter {@link rotation.Angle#getRevolutions()}. 
	 * <br>setter {@link rotation.Angle#setRevolutions(double)}. 
	 */
	@Test
	public void testGetSetInRevolutions() {
		System.out.println("********Test Get Set In Revolutions");

		Angle a = new Angle();
		assertEquals(Double.isNaN(a.getRevolutions()), true);
		a.set(Angle.EMPTY);
		assertEquals(Double.isNaN(a.getRevolutions()), true);

		Angle b = new Angle();
		double rev;

		for (int i = -256; i <= 256; i++) {
			rev = i / 16d;
			b = Angle.inRevolutions(rev);
			assertEquals(b.getRevolutions(), rev, 1e-13);
			a.setRevolutions(b.getRevolutions());
			assertEquals(a.getRevolutions(), b.getPiRadians() / Angle.REVOLUTION.getPiRadians(), 1e-13);
		}
	}

}
