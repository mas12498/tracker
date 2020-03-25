package rotation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import rotation.Angle;
import rotation.Quaternion;
import rotation.QuaternionMath;
import rotation.Vector3;

public class Vector3Test {

	@Test
	public void testAbs() {
		Vector3 v1 = new Vector3(3, 4, 0);
		double d = v1.getAbs();
		assertEquals(d, 5d, 1e-15d);
	}

	@Test
	public void testAdd() {
		Vector3 v1 = new Vector3(1, 2, 3);
		Vector3 v2 = new Vector3(4, 5, 6);
		// System.out.println(v1.toString());
		v1.add(v2);
		// System.out.println(v2.toString());
		// System.out.println(v1.toString());
		assertTrue(v1.isEquivalent(new Vector3(5,7,9),0));

	}

	@Test
	public void testCross() {
		Vector3 v1 = new Vector3(1, 0, 0);
		Vector3 v2 = new Vector3(0, 1, 0);
		Vector3 v3 = new Vector3(v1);

		v3.crossProduct(v2);
		assertTrue(v3.isEquivalent(new Vector3(0,0,1),0));

		v3.set(v1);
		v3.leftCrossProduct(v2);
		assertTrue(v3.isEquivalent(new Vector3(0,0,-1),0));
		
		v3.crossProduct(v2);
		assertTrue(v3.isEquivalent(new Vector3(1,0,0),0));

		v1.set(1, 0, 0);
		v2.set(0, 1, 0);
		v3.set(v1);
		v3.crossProduct(v2);
		assertTrue(v3.isEquivalent(new Vector3(0,0,1),0));

		v3.set(v1);
		v3.leftCrossProduct(v2);
		assertTrue(v3.isEquivalent(new Vector3(0,0,-1),0));

		v3.crossProduct(v2);
		assertTrue(v3.isEquivalent(new Vector3(1,0,0),0));

		// System.out.println(v1.toString());
	}

	@Test
	public void testCrossproduct() {
		Vector3 v1 = new Vector3(2, 0, 0);
		Vector3 v2 = new Vector3(0, 3, 0);
		Vector3 v3 = QuaternionMath.crossproduct(v1, v2);
		assertTrue(v1.isEquivalent(new Vector3(2,0,0),0));
		assertTrue(v2.isEquivalent(new Vector3(0,3,0),0));
		assertTrue(v3.isEquivalent(new Vector3(0,0,6),0));
	}

	@Test
	public void testDivide() {
		Vector3 v1 = new Vector3(2, 4, 6);
		double s = 2d;
		// System.out.println(v1.toString());
		v1.divide(s);
		// System.out.println(v1.toString());
		assertTrue(v1.isEquivalent(new Vector3(1,2,3),0));
		// System.out.println(v1.divide(0.0).toString());
		assertEquals(Boolean.TRUE, Double.isNaN(v1.divide(0.0).getAbs()));
	}

	@Test
	public void testDot() {
		Vector3 v1 = new Vector3(1, 2, 3);
		Vector3 v2 = new Vector3(0, 4, 1);
		double d = v1.getInnerProduct(v2);
		assertEquals(d, 11d, 1e-15);
		d = v2.getInnerProduct(v1);
		assertEquals(d, 11d, 1e-15);
	}

	@Test
	public void testExpLn() {

		Vector3 v3 = new Vector3(0, Angle.inDegrees(30).getRadians(), 0);
		Quaternion q3 = v3.exp();
		// System.out.println("exp(0,30,0) = " + q3.toString());
		assertEquals(q3.getW(), StrictMath.sqrt(3) / 2, 1e-14);
		assertEquals(q3.getY(), .5, 1e-14);
//		System.out.println("ln(exp(0,30,0)) = "
//				+ new Quaternion(q3).ln().multiply(45 / StrictMath.atan(1)).toString());
		assertTrue((new Quaternion(q3)).ln().multiply(45 / StrictMath.atan(1))
				.isEquivalent(new Quaternion(0, 0, 30, 0), 1e-14));

		q3.set(v3.exp());		
//		System.out
//				.println("exp(ln(exp(0,30,0))) = " + new Quaternion(q3).ln().exp().toString());
		assertTrue((new Quaternion(q3)).ln().exp()
				.isEquivalent(new Quaternion(StrictMath.sqrt(3)/2, 0, .5, 0), 1e-14));

		Vector3 v4 = new Vector3(0, Angle.inDegrees(30).getRadians(), 0);
		q3.set(v4.ln());
		// System.out.println("ln(0,30,0) = " + q3.toString());
		assertEquals(q3.getW(), StrictMath.log(StrictMath.PI / 6), 1e-14);

		
		q3.set(v4.ln());
//		System.out.println("ln(0,30,0) = " + new Quaternion(q3).toString());
		//TODO MAS	
		assertEquals(q3.getY(), StrictMath.PI/2, 1e-14);
		//assertEquals(q3.j(), 1, 1e-14);

		v4.set(new Vector3(0, 0, 0));
		q3.set(v4.ln());
		// System.out.println("ln(0,0,0) = " + q3.toString());
		assertEquals(Boolean.TRUE, Quaternion.EMPTY.equals(q3));

		v3.set(new Vector3(0, Angle.inDegrees(30).getRadians(), 0));
		q3.set(v3.exp());q3.ln().ln();
		// System.out.println("ln(0,30,0) = " + q3.toString());
		assertEquals(q3.getW(), StrictMath.log(StrictMath.PI / 6), 1e-14);
		assertEquals(q3.getY(), .5 * StrictMath.PI, 1e-14);

//		System.out.println("exp(ln(0,30,0)) = "
//				+ new Quaternion(q3).exp().multiply(45 / StrictMath.atan(1)).toString());
		assertTrue((new Quaternion(q3)).exp().multiply(45 / StrictMath.atan(1))
				.isEquivalent(new Quaternion(0, 0, 30, 0), 1e-14));

		Quaternion p = new Quaternion(StrictMath.sqrt(3), 1, 0, 0);
//		System.out.println(p.toString());

		double a = p.getAbs();
		// System.out.println(a);

		Quaternion q = new Quaternion(p.unit());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(StrictMath.sqrt(3)/2,.5,0,0),1e-14));

//		System.out.println("   ");

		q.multiply(a);
//		System.out.println(q.toString());

		q.set(p.unit());
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(StrictMath.sqrt(3)/2,.5,0,0),1e-14));

//		System.out.println("   ");

		q.multiply(a); //double
//		System.out.println(q.toString()); //<sqrt(3),1,0,0>

//		System.out.println("   ");
//		System.out.println("   ");

		p = new Quaternion(StrictMath.sqrt(3), -1, 0, 0);
//		System.out.println(p.toString());
		q = new Quaternion(p.unit());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(StrictMath.sqrt(3)/2,-.5,0,0),1e-14));

//		System.out.println("   ");

		q.multiply(a);
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(StrictMath.sqrt(3),-1,0,0),1e-14));

		q.set(p.unit());
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(StrictMath.sqrt(3)/2,-.5,0,0),1e-14));

//		System.out.println("   ");
//		System.out.println("   ");

		p = new Quaternion(-StrictMath.sqrt(3), 1, 0, 0);
//		System.out.println(p.toString());
		q = new Quaternion(p.unit());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(-StrictMath.sqrt(3)/2,.5,0,0),1e-14));

//		System.out.println("   ");

		q.multiply(a);
//		System.out.println(q.toString());

		q.set(p.unit());
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(-StrictMath.sqrt(3)/2,.5,0,0),1e-14));

//		System.out.println("   ");
//		System.out.println("   ");

		p = new Quaternion(-StrictMath.sqrt(3), -1, 0, 0);
//		System.out.println(p.toString());
		q = new Quaternion(p.unit());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(-StrictMath.sqrt(3)/2,-.5,0,0),1e-14));

//		System.out.println("   ");

		q.multiply(a);
//		System.out.println(q.toString());

		q.set(p.unit());
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(new Quaternion(-StrictMath.sqrt(3)/2,-.5,0,0),1e-14));

		p = new Quaternion(-StrictMath.sqrt(3), -1, -1, 0);
//		System.out.println(p.toString());
		q = new Quaternion(p.unit());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(
				new Quaternion(-StrictMath.sqrt(3) / StrictMath.sqrt(5), -1
						/ StrictMath.sqrt(5), -1 / StrictMath.sqrt(5), 0),
				1e-14));

//		System.out.println("   ");

		q.multiply(a);
//		System.out.println(q.toString());

		q.set(p.unit());
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		assertTrue(q.isEquivalent(
				new Quaternion(-StrictMath.sqrt(3) / StrictMath.sqrt(5), -1
						/ StrictMath.sqrt(5), -1 / StrictMath.sqrt(5), 0),
				1e-14));

//		System.out.println("   ");

		p = new Quaternion(-StrictMath.sqrt(3), -1, 1, 0);
//		System.out.println(p.toString());
		q = new Quaternion(p.unit());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println("here"+q.toString());
		assertTrue(q
				.isEquivalent(new Quaternion(
						-StrictMath.sqrt(3) / StrictMath.sqrt(5), -1
								/ StrictMath.sqrt(5), 1 / StrictMath.sqrt(5), 0),
						1e-14));

//		System.out.println("   ");

		q.multiply(a);
//		System.out.println(q.toString());

		q.set(p.unit());
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		assertTrue(q
				.isEquivalent(new Quaternion(
						-StrictMath.sqrt(3) / StrictMath.sqrt(5), -1
								/ StrictMath.sqrt(5), 1 / StrictMath.sqrt(5), 0),
						1e-14));

//		System.out.println("   ");
//		System.out.println("   ");
//		System.out.println("   ");

		q = new Quaternion(-StrictMath.sqrt(3), -1, 1, 0);
		// q.set(p.versor());
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println("first: "+q.toString());
		assertTrue(q.isEquivalent(new Quaternion(-StrictMath.sqrt(3), -1, 1, 0),
				1e-14));

//		System.out.println("   ");

		q = new Quaternion(-StrictMath.sqrt(3), 1, -1, 0);
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.ln();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println(q.toString());
		q.exp();
//		System.out.println("here:"+q.toString());
		assertTrue(q.isEquivalent(new Quaternion(-StrictMath.sqrt(3), 1, -1, 0),
				1e-14));

		// fail("Not yet implemented"); // TODO
	}

	@Test
	public void testMultiplyDouble() {
		Vector3 v1 = new Vector3(1, 2, 3);
		double s = 3d;
		// System.out.println(v1.toString());
		v1.multiply(s);
		// System.out.println(v1.toString());
		assertTrue(v1.isEquivalent(new Vector3(3,6,9),0));
	}

	@Test
	public void testMultiplyVector3Op() {
		Vector3 v1 = new Vector3(1, 2, 3);
		Vector3 v2 = new Vector3(0, 4, 1);
		
		Quaternion p = v1.multiply(v2);
		assertTrue(p.isEquivalent(new Quaternion(11,-10,-1,4),1e-13));

		
		p = v1.leftMultiply(v2);
		assertTrue(p.isEquivalent(new Quaternion(11,10,1,-4),1e-13));
		
		v1.set(1, 2, 3);
		v2.set(0, 4, 1);
		p = v1.multiply(v2);
		assertTrue(p.isEquivalent(new Quaternion(11,-10,-1,4),1e-13));
		
		p = v1.leftMultiply(v2);
		assertTrue(p.isEquivalent(new Quaternion(11,10,1,-4),1e-13));
	}

	@Test
	public void testNegate() {
		Vector3 v1 = new Vector3(1, 2, 3);
		v1.negate();
		assertTrue(v1.isEquivalent(new Vector3(-1,-2,-3),0));
	}

	@Test
	public void testNorm1() {
		Vector3 v1 = new Vector3(1, 2, 3);
		double d = v1.getNorm1();
		assertEquals(d, 6d, 0d);
	}

	@Test
	public void testNorm2() {
		Vector3 v1 = new Vector3(1, 2, 3);
		double d = v1.getDeterminant();
		assertEquals(d, 14d, 0d);
	}

	@Test
	public void testNormInf() {
		Vector3 v1 = new Vector3(1, 2, 3);
		double d = v1.getNormInf();
		assertEquals(d, 3d, 0d);
	}

	@Test
	public void testPow() {
//		Vector3 v1 = new Vector3(0, 1, 2);
//		Vector3 v2 = new Vector3(1, 0, -2);
//		Quaternion q1 = new Vector3(v1).power(2.0);
//		Quaternion q2 = new Vector3(v1).power(-2);
//		Quaternion q3 = new Vector3(v1).power(0.5);
//		Quaternion q4 = new Vector3(v1).power(-0.5);
//		Quaternion q5 = new Vector3(v1).power(v2);
//		Quaternion q6 = new Vector3(v1).power(q1);
//		Quaternion q7 = new Vector3(v1).log10();
//
////		System.out.println("===============");
//		System.out.println("v1 = " + v1.toString());
//		System.out.println("v2 = " + v2.toString());
//		System.out.println("q1 = " + q1.toString());
//		System.out.println("q2 = " + q2.toString());
//		System.out.println("q3 = " + q3.toString());
//		System.out.println("q4 = " + q4.toString());
//		System.out.println("q5 = " + q5.toString());
//		System.out.println("q6 = " + q6.toString());
//		System.out.println("q7 = " + q7.toString());
//		System.out.println("===============");

		Quaternion q = new Quaternion(1, 2, 3, 4);

//		System.out.println(q.toString());
		q.power(2);
//		System.out.println(q.toString());
		q.power(.5);
//		System.out.println("restore:"+q.toString());
		assertTrue(q.isEquivalent(new Quaternion(1,2,3,4),1e-14));

		q.set(new Quaternion(1, 2, 3, 4));

//		System.out.println(q.toString());
		q.power(.5);
//		System.out.println(q.toString());
		q.power(2);
//		System.out.println("restore:"+q.toString());
		assertTrue(q.isEquivalent(new Quaternion(1,2,3,4),1e-14));

		q.set(new Quaternion(1, -2, 3, -4));
//		System.out.println(q.toString());
		Quaternion p = new Quaternion(3, -1, 1, 2).unit();
//		System.out.println(p.toString());
		q.power(p);
//		System.out.println(q.toString());
		p.set(new Quaternion(3, -1, 1, 2).unit().reciprocal());
		q.power(p);
//		System.out.println("restore:"+q.toString());
		assertTrue(q.isEquivalent(new Quaternion(1,-2,3,-4),1e-14));

		q.set(new Quaternion(1, 2, -3, -4));
//		System.out.println(q.toString());
		// p = new Quaternion(3,-1,1,2);
		p.set(new Quaternion(3, -1, 1, 2).unit().divide(5));
//		System.out.println(p.toString());
		q.power(p);
//		System.out.println(q.toString());
		// p = new Quaternion(3,-1,1,2).reciprocal();
		p.set(new Quaternion(3, -1, 1, 2).unit().divide(5).reciprocal());
		q.power(p);
//		System.out.println("small 1:" + q.toString());
		assertTrue(q.isEquivalent(new Quaternion(1,2,-3,-4),1e-14));

		
		q.set(new Quaternion(1, -2, -3, -4));
//		System.out.println(q.toString());
		p.set(new Quaternion(3, -1, 1, 2).unit());
//		System.out.println(p.toString());
		q.power(p.unit());
//		System.out.println(q.toString());
		p.set((new Quaternion(3, -1, 1, 2).unit().reciprocal()));
		q.power(p);
//		System.out.println(" s and vector: " + q.toString());
		assertTrue(q.isEquivalent(new Quaternion(1,-2,-3,-4),1e-14));

		q = new Quaternion(-1, 2, -3, -4);
		// // Vector3 t = new Vector3(-1,1,2).unit().multiply(2);
		Vector3 t = new Vector3(-1, 1, 2).unit().multiply(1.819728631583); // .divide(2d);
//		System.out.println("vector test==: " + q.toString()
		// +"\n abs p "+t.abs()
		// +"  abs q "+q.abs()
//				);
//		System.out.println(t.toString());
		// q.pow(t);
		// System.out.println(q.toString());
		p.set(new Quaternion(0, t));
		// System.out.println(q.toString());
//		System.out.println("vector POWER == " + p.toString()
		// +"\n abs p "+p.abs()
		// +" (abs  q)^(abs p) "+StrictMath.pow(q.abs(),p.abs())
//				);

		// //q.pow(t);
		// //System.out.println(q.toString());
		// //q.set(-1,2,-3,-4);
		// q.set(q.pow(p));
		q.power(p);
//		System.out.println(q.toString()
		// +"\n abs p "+p.abs()
		// +"   q^p "+q.abs()
//				);
//		System.out.println(t.toString());
		p.set(new Quaternion(0, t).reciprocal());
//		System.out.println("vector inverse == " + p.toString()
		// +" "+p.abs()
//				);
		q.power(p);
//		System.out.println(q.toString()
		// +"\n abs (1/p) "+p.abs()
		// +"   (q^p)^(1/p)) "+q.determinant()
//				);
		assertTrue(q.isEquivalent(new Quaternion(-1,2,-3,-4),1e-14));

		// fail("Not yet implemented"); // TODO

	}

	@Test
	public void testSubtract() {
		Vector3 v1 = new Vector3(1, 2, 3);
		Vector3 v2 = new Vector3(4, 5, 6);
		// System.out.println(v1.toString());
		v1.subtract(v2);
		// System.out.println(v2.toString());
		// System.out.println(v1.toString());
		assertTrue(v1.isEquivalent(new Vector3(-3,-3,-3),0));
	}

	@Test
	public void testUnit() {
		Vector3 v1 = new Vector3(3, -4, 0);
		Vector3 d = v1.unit();
		d.unit();
		assertTrue(d.isEquivalent(new Vector3(.6,-.8,0),1e-15));
		d.set(Vector3.ZERO);
		assertEquals(d.getAbs(), 0d, 1e-15d);

	}

	@Test
	// public void testVector3DoubleDoubleDouble() {
	// public void testVector3Vector3() {
	// public void testGetI() {
	// public void testGetJ() {
	// public void testGetK() {
	// public void testSet() {
	// public void testToString() {
	public void testVector3SetGet() {
		Vector3 v1 = new Vector3(1, 2, 3);
		Vector3 v2 = new Vector3(v1);
		Vector3 v3 = new Vector3(4, 5, 6);

		assertTrue(v1.isEquivalent(new Vector3(1,2,3),0));		
		assertTrue(v2.isEquivalent(new Vector3(1,2,3),0));
		
		v2.set(v3);

		assertTrue(v2.isEquivalent(new Vector3(4,5,6),0));
//		System.out.println(v2.toString(1));
	}

	// @Test
	// public void testLog10() {
	// fail("Not yet implemented"); // TODO
	// }

}
