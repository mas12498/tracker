/**
 * 
 */
package rotation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import rotation.Angle;
import rotation.CodedPhase;
import rotation.Quaternion;
import rotation.QuaternionMath;
import rotation.Rotator;
import rotation.Vector3;

/**
 * @author mike
 *
 */
public class RotatorTest {

	@Test
	public void testEulerMath() {

		CodedPhase h =  Angle.inDegrees(10).codedPhase();
		CodedPhase p =  Angle.inDegrees(30).codedPhase();
		CodedPhase r =  Angle.inDegrees(40).codedPhase();
		CodedPhase hd =  Angle.inDegrees(10).codedPhase();
		CodedPhase pd =  Angle.inDegrees(30).codedPhase();

		assertEquals(h.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
		assertEquals(p.angle().signedPrinciple().getDegrees(), 30d, 1e-13);
		assertEquals(r.angle().signedPrinciple().getDegrees(), 40d, 1e-13);

		Rotator z = new Rotator();

		//attitude coverage:
		System.out.println("=========== Start Quaternion Math Coverage ==============");

		Angle heading = new Angle();
		Angle pitch = new Angle();
		Angle roll = new Angle();
		int digi = 10;

		//pole testing:
		System.out.println("=========== eulerRotate_kj");

		//north pole degenerate: roll equal 0d for default extraction...
		//o.w. heading == 0d and roll == -heading... or some combination?????

		roll = Angle.inPiRadians(0d); //One default degenerate condition.
		
		for(int k=-8; k<=8; k+=1)
		{
			heading = Angle.inPiRadians(k/8d); 

			for(int j=-15; j<=15; j+=1) //pitch
			{	
				pitch = Angle.inPiRadians( j/16d  ); //heading

				z = QuaternionMath.rotate_k(heading.codedPhase()) 
						.rotate_j(pitch.codedPhase())
						.rotate_i(roll.codedPhase());
				h = z.getEuler_k_kji();
				p = z.getEuler_j_kji();
				r = z.getEuler_i_kji();
				hd = z.getEuler_k_kj();
				pd = z.getEuler_j_kj();
				Vector3 e = z.getEuler_kji();

									System.out.println(" heading: "+heading.signedPrinciple().toDegreesString(digi)+" == :" + h.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
									System.out.println(" pitch  : "+pitch.signedPrinciple().toDegreesString(digi)+" == :" + p.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
									System.out.println(" roll   : "+roll.signedPrinciple().toDegreesString(digi)+" == :" + r.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
									System.out.println(z.getEuler_j_kj().angle().toDegreesString(digi) + "   " + z.getEuler_k_kj().angle().toDegreesString(digi) +" is normal kj: "+ z.isNormal_kj());
//									System.out.println(
//									" Euler  :\n      roll " + CodedPhase.encodes(e.getX()).angle().signedPrinciple().toDegreesString(digi) 
//								    + "\n      pitch " + CodedPhase.encodes(e.getY()).angle().signedPrinciple().toDegreesString(digi) 
//								    + "\n      head " + CodedPhase.encodes(e.getZ()).angle().signedPrinciple().toDegreesString(digi) 
//								); //.getDegrees()); //.toDegreesString(8));
									System.out.println(" ");


				//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
				//					   @MAS assertEquals(heading.signedPrinciple().getDegrees(),h.angle().signedPrinciple().getDegrees(),1e-12);
				assertEquals(new Angle(heading).subtract(hd.angle()).signedPrinciple().getDegrees(),0.0d,1e-10);
				//					
				assertEquals(pitch.signedPrinciple().getDegrees(),pd.angle().signedPrinciple().getDegrees(),1e-10);
				assertEquals(roll.signedPrinciple().getDegrees(),r.angle().signedPrinciple().getDegrees(),1e-12);

				//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
				//					   @MAS assertEquals(heading.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getZ()).angle().signedPrinciple().getDegrees(),1e-12);
//				assertEquals(new Angle(heading).subtract((CodedPhase.encodes(e.getZ()).angle())).signedPrinciple().getDegrees(),0.0d,1e-10);
				//
//				assertEquals(pitch.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getY()).angle().signedPrinciple().getDegrees(),1e-10);
//				assertEquals(roll.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getX()).angle().signedPrinciple().getDegrees(),1e-12);



			}
		} 

		
		
	}

	/**
	 * Test method for {@link rotation.Rotator#rotate_i(rotation.CodedPhase)}.
	 */
	@Test
	public void testExp() {
//		System.out.println("=============== TEST EXP =================");

//		Operator z = new Operator(0,(Angle.inDegrees(30).bisectorPrincipleAngle().getRadians()),0,0);
		Rotator z = new Rotator(0,(Angle.inDegrees(15).getRadians()),0,0);
		Vector3 v = new Vector3(1,0,0);
		CodedPhase theta =  Angle.inDegrees(30).codedPhase();
		
		z.set(1,0,0,0);
		z.rotate_i(CodedPhase.ZERO); //.half().getRadians());
//		System.out.println("exp i(0): "+z.multiply(z.getNormalizeFactor()).toString());
		assertTrue(z.isEquivalent(new Rotator(
				Quaternion.IDENTITY),1e-13));
		z.set(1,0,0,0);
		z.rotate_j(CodedPhase.ZERO); //.half().getRadians());
//		System.out.println("exp j(0): "+z.multiply(z.getNormalizeFactor()).toString());
		assertTrue(z.isEquivalent(new Rotator(1,0,0,0),1e-13));
		z.set(1,0,0,0);
		
		z.rotate_k(CodedPhase.ZERO); //.half().getRadians());
//		System.out.println("exp k(0): "+z.multiply(z.getNormalizeFactor()).toString());
		assertTrue(z.isEquivalent(new Rotator(1,0,0,0),1e-13));
		
		z.set(1,0,0,0);
		z.rotate_i(theta).multiply(z.getNormaliztionFactor()); //.half().getRadians());
//		System.out.println("exp i(30): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(
				StrictMath.cos(Angle.inDegrees(15).getRadians()),
				StrictMath.sin(Angle.inDegrees(15).getRadians()),
				0,
				0),1e-13));
		z.set(1,0,0,0);
		z.rotate_j(theta).multiply(z.getNormaliztionFactor()); //.half().getRadians());
//		System.out.println("Here exp j(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()), 0, StrictMath.sin(Angle.inDegrees(15)
						.getRadians()), 0), 1e-13));
		
		z.set(1,0,0,0);
		z.rotate_k(theta).multiply(z.getNormaliztionFactor()); //.half().getRadians());
		//System.out.println("exp k(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()), 0, 0, StrictMath.sin(Angle
						.inDegrees(15).getRadians())), 1e-13));

		theta = Angle.inDegrees(120).codedPhase();
		z.set(1,0,0,0);
		z.rotate_i(theta).multiply(z.getNormaliztionFactor()); //.half().getRadians());
//		System.out.println("exp i(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()), StrictMath.sin(Angle
						.inDegrees(60).getRadians()),0,0), 1e-13));
		
		z.set(1,0,0,0);
		z.rotate_j(theta).multiply(z.getNormaliztionFactor()); //.half().getRadians());
//		System.out.println("exp j(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()), 0, StrictMath.sin(Angle
						.inDegrees(60).getRadians()),0), 1e-13));
		
		z.set(1,0,0,0);
		z.rotate_k(theta).multiply(z.getNormaliztionFactor()); //.half().getRadians());
//		System.out.println("exp k(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()), 0, 0, StrictMath.sin(Angle
						.inDegrees(60).getRadians())), 1e-13));
			
//		System.out.println("=============== TEST EXP VECTOR =================");
		
		theta.set(CodedPhase.ZERO);
		z.set(1,0,0,0);
		v.set(1,0,0);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(0): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(1, 0, 0, 0), 1e-13));			
		theta = Angle.inDegrees(30).codedPhase();
		z.set(1,0,0,0);
		v.set(1,0,0);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()), StrictMath.sin(Angle.inDegrees(15)
						.getRadians()), 0, 0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,1,0);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector j(30): "+z.toString());		
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()), 0, StrictMath.sin(Angle.inDegrees(15)
						.getRadians()), 0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,0,1);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector k(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()), 0,0,StrictMath.sin(Angle.inDegrees(15)
						.getRadians())), 1e-13));
		
		theta = Angle.inDegrees(120).codedPhase();
		z.set(1,0,0,0);
		v.set(1,0,0);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()), StrictMath.sin(Angle.inDegrees(60)
						.getRadians()),0,0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,1,0);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector j(120): "+z.toString());		
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()),0, StrictMath.sin(Angle.inDegrees(60)
						.getRadians()),0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,0,1);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector k(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()),0, 0, StrictMath.sin(Angle.inDegrees(60)
						.getRadians())), 1e-13));
		
		
		theta.set(CodedPhase.ZERO);
		z.set(1,0,0,0);
		v.set(1,0,0);
		v.multiply(theta.angle().getRadians());
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(0): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(
						1,0, 0, 0), 1e-13));
		
		theta = Angle.inDegrees(30).codedPhase();
		z.set(1,0,0,0);
		v.set(1,0,0);
		v.multiply(theta.angle().getRadians());
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()),StrictMath.sin(Angle.inDegrees(15)
						.getRadians()),0,0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,1,0); 
		v.multiply(theta.angle().getRadians());
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector j(30): "+z.toString());		
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()),0,StrictMath.sin(Angle.inDegrees(15)
						.getRadians()),0), 1e-13));
		
		
		z.set(1,0,0,0);
		v.set(0,0,1);
		v.multiply(theta.angle().getRadians());
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector k(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()),0,0,StrictMath.sin(Angle.inDegrees(15)
						.getRadians())), 1e-13));
		
		theta = Angle.inDegrees(120).codedPhase();
		z.set(1,0,0,0);
//		z.multiply(theta.getRadians()*2);
		v.set(1,0,0);
		v.multiply(theta.angle().getRadians()*2);
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(120): "+z.toString());
		
		z.set(1,0,0,0);
		v.set(0,1,0);
		v.multiply(theta.angle().getRadians());
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector j(120): "+z.toString());		
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()),0, StrictMath.sin(Angle.inDegrees(60)
						.getRadians()), 0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,0,1);
		v.multiply(theta.angle().getRadians());
		z.rotate( theta ,v).multiply(z.getNormaliztionFactor());
//		System.out.println("vector k(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()),0,0, StrictMath.sin(Angle.inDegrees(60)
						.getRadians())), 1e-13));
		

		theta.set(CodedPhase.ZERO);
		z.set(1,0,0,0);
		v.set(1,0,0);
		v.multiply(theta.angle().getRadians());
		z.rotate( v ).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(0): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(
						1,0,0, 0), 1e-13));
		
		
		theta = Angle.inDegrees(30).codedPhase();
		z.set(1,0,0,0);
		v.set(1,0,0);
		v.multiply(theta.angle().signedPrinciple().getRadians()); 
		z.rotate( v ).unit(); //.multiply(z.getInverseAbs());
//		System.out.println("vector i(30): "+z.toString() + "  Theta (deg)= " + theta.getPiRadians()*180);
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()), StrictMath.sin(Angle.inDegrees(15)
						.getRadians()),0,0), 1e-13));
		
		
		z.set(1,0,0,0);
		v.set(0,1,0);
		v.multiply(theta.angle().signedPrinciple().getRadians()); 
		z.rotate( v ).multiply(z.getNormaliztionFactor());
		System.out.println("vector j(30): "+z.toString());		
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()),0, StrictMath.sin(Angle.inDegrees(15)
						.getRadians()),0), 1e-13));
		
		z.set(1,0,0,0);
		v.set(0,0,1);
		v.multiply(theta.angle().getRadians());
		z.rotate( v ).multiply(z.getNormaliztionFactor());
//		System.out.println("vector k(30): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(15)
						.getRadians()),0,0, StrictMath.sin(Angle.inDegrees(15)
						.getRadians())), 1e-13));
		
		theta = Angle.inDegrees(120).codedPhase();
		
		z.set(1,0,0,0);
		z.multiply(theta.angle().getRadians());
		v.set(1,0,0);
		v.multiply(theta.angle().getRadians());
		z.rotate( v ).multiply(z.getNormaliztionFactor());
//		System.out.println("vector i(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()), StrictMath.sin(Angle.inDegrees(60)
						.getRadians()),0,0), 1e-13));
		
		
		z.set(1,0,0,0);
		v.set(0,1,0);
		v.multiply(theta.angle().getRadians());
		z.rotate( v ).multiply(z.getNormaliztionFactor());
//		System.out.println("vector j(120): "+z.toString());		
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()),0, StrictMath.sin(Angle.inDegrees(60)
						.getRadians()),0), 1e-13));
		
		
		z.set(1,0,0,0);
		v.set(0,0,1);
		v.multiply(theta.angle().getRadians());
		z.rotate( v ).multiply(z.getNormaliztionFactor());
//		System.out.println("vector k(120): "+z.toString());
		assertTrue(z
				.isEquivalent(new Rotator(StrictMath.cos(Angle.inDegrees(60)
						.getRadians()),0,0, StrictMath.sin(Angle.inDegrees(60)
						.getRadians())), 1e-13));
		
		
		System.out.println("\nimage ej0 = :"+Vector3.UNIT_J.toString());
		
		CodedPhase wi =  Angle.inDegrees(30).codedPhase();
		Rotator pi = QuaternionMath.rotate_i(wi);
		
		int i;
		for(i=1; i<=12; i+=1)
		{
			System.out.println("\nimage ej"+i+" = :" + pi.getImage(Vector3.UNIT_J).toString());
			System.out.println("        "+i+" = :" + pi.getImage_j().toString());
			System.out.println("  "+ new Rotator(pi).getImage(new Quaternion(0,Vector3.UNIT_J)).toString());
			pi.rotate_i(wi);

		}

		System.out.println("\n=====================================");

		System.out.println("\nimage ek0 = :"+Vector3.UNIT_K.toString());
		
		CodedPhase wj =  Angle.inDegrees(60).codedPhase();
		Rotator pj = QuaternionMath.rotate_j(wj);
		
		int j;
		for(j=1; j<=6; j+=1)
		{
			System.out.println("\nimage ek"+j+" = :" + pj.getImage(Vector3.UNIT_K).toString());
			System.out.println("        "+j+" = :" + pj.getImage_k().toString());
			System.out.println("  "+ new Rotator(pj).getImage(new Quaternion(0,Vector3.UNIT_K)).toString());
			pj.rotate_j(wj);
		}
		
		System.out.println("\n=====================================");

		System.out.println("\nimage ei0 = :"+Vector3.UNIT_I.toString());
		
		CodedPhase wk =  Angle.inDegrees(45).codedPhase();
		Rotator pk = QuaternionMath.rotate_k(wk);
		
		int k;
		for(k=1; k<=8; k+=1)
		{
			System.out.println("\nimage ei"+k+" = :" + pk.getImage(Vector3.UNIT_I).toString());
			System.out.println("        "+k+" = :" + pk.getImage_i().toString());
			System.out.println("  "+ new Rotator(pk).getImage(new Quaternion(0,Vector3.UNIT_I)).toString());
			pk.rotate_k(wk);
		}
		
		
		//		fail("Not yet implemented");
	}

	/**
	 * Test method for {@link rotation.Rotator#flip_i()}.
	 */
	@Test
	public void testFlip() {
		Rotator z = new Rotator(0,0,1,0);
		z.flip_i();
//		System.out.println("a. flip i(j): "+z.toString());
//		System.out.println("b. flip i(j): "+z.multiply(z.getNormalizeFactor()).toString());
		assertTrue(z.isEquivalent(new Rotator(0,0,0,-1),1e-13));
		z.set(0,0,0,1);
		z.flip_j();
//		System.out.println("flip j(k): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,-1,0,0),1e-13));
		z.set(0,1,0,0);
		z.flip_k();
//		System.out.println("flip k(i): "+z.multiply(z.getNormalizeFactor()).toString());
		assertTrue(z.isEquivalent(new Rotator(0,0,-1,0),1e-13));
		
		Vector3 v = new Vector3(1,0,0);
		z.set(0,0,1,0);
		z.flip(v);
//		System.out.println("flip i(j): "+z.multiply(z.getNormalizeFactor()).toString());
		assertTrue(z.isEquivalent(new Rotator(0,0,0,-1),1e-13));
		
		v.set(0,1,0);
		z.set(0,0,0,1);
		z.flip(v);
//		System.out.println("flip j(k): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,-1,0,0),1e-13));
		
		
		v.set(0,0,1);
		z.set(0,1,0,0);
		z.flip(v);
//		System.out.println("flip k(i): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,0,-1,0),1e-13));
		

		//composite flip-checks...maintains order, changes signs...

		Quaternion q = new Quaternion(1,2,3,4);
		z.set(q);		
		z.flip_i().flip_j().flip_k();
		assertTrue(z.negate().equals(q));

		z.set(q);		
		z.flip_j().flip_k().flip_i();
		assertTrue(z.negate().equals(q));

		z.set(q);		
		z.flip_k().flip_i().flip_j();
		assertTrue(z.negate().equals(q));

		//		fail("Not yet implemented");
	}
	
	/**
	 * Test method for constructing Operator from Euler angles
	 * {@link rotation.Rotator#getEuler_k_kji()}.
	 */
	@Test
	public void testGetEuler() {

		CodedPhase h =  Angle.inDegrees(10).codedPhase();
		CodedPhase p =  Angle.inDegrees(30).codedPhase();
		CodedPhase r =  Angle.inDegrees(40).codedPhase();

		assertEquals(h.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
		assertEquals(p.angle().signedPrinciple().getDegrees(), 30d, 1e-13);
		assertEquals(r.angle().signedPrinciple().getDegrees(), 40d, 1e-13);

		Rotator z = new Rotator();
		
		//attitude coverage:
		System.out.println("=========== Start Attitude Coverage ==============");

		Angle heading = new Angle();
		Angle pitch = new Angle();
		Angle roll = new Angle();
		int digi = 10;
		
		//pole testing:
		System.out.println("=========== North Pole Testing");
		
		//north pole degenerate: roll equal 0d for default extraction...
		//o.w. heading == 0d and roll == -heading... or some combination?????

//		for(int i=0; i<=0; i+=1)
//		{
//			roll = Angle.inPiRadians(i/8d); 
			roll = Angle.inPiRadians(0d); //One default degenerate condition.
			for(int j=0; j<=1; j+=1)
			{	
				pitch = Angle.inPiRadians( 1/2d - j*0.0001d ); 
				for(int k=0; k<=16; k+=1)
				{
					heading = Angle.inPiRadians(k/8d); 

					z = QuaternionMath.rotate_k(heading.codedPhase()) 
							.rotate_j(pitch.codedPhase())
							.rotate_i(roll.codedPhase());
					h = z.getEuler_k_kji();
					p = z.getEuler_j_kji();
					r = z.getEuler_i_kji();
					Vector3 e = z.getEuler_kji();
					
//					System.out.println(" heading: "+heading.signedPrinciple().toDegreesString(digi)+" == :" + h.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" pitch  : "+pitch.signedPrinciple().toDegreesString(digi)+" == :" + p.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" roll   : "+roll.signedPrinciple().toDegreesString(digi)+" == :" + r.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" is dumped kj: "+ z.getDump_kj());
//					System.out.println(
//					" Euler  :\n      roll " + CodedPhase.encodes(e.getX()).angle().signedPrinciple().toDegreesString(digi) 
//				    + "\n      pitch " + CodedPhase.encodes(e.getY()).angle().signedPrinciple().toDegreesString(digi) 
//				    + "\n      head " + CodedPhase.encodes(e.getZ()).angle().signedPrinciple().toDegreesString(digi) 
//				); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" ");
					
					
//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
//					   @MAS assertEquals(heading.signedPrinciple().getDegrees(),h.angle().signedPrinciple().getDegrees(),1e-12);
					assertEquals(new Angle(heading).subtract(h.angle()).signedPrinciple().getDegrees(),0.0d,1e-10);
//					
					assertEquals(pitch.signedPrinciple().getDegrees(),p.angle().signedPrinciple().getDegrees(),1e-10);
					assertEquals(roll.signedPrinciple().getDegrees(),r.angle().signedPrinciple().getDegrees(),1e-12);
					
//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
//					   @MAS assertEquals(heading.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getZ()).angle().signedPrinciple().getDegrees(),1e-12);
					assertEquals(new Angle(heading).subtract((CodedPhase.encodes(e.getZ()).angle())).signedPrinciple().getDegrees(),0.0d,1e-10);
//
					assertEquals(pitch.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getY()).angle().signedPrinciple().getDegrees(),1e-10);
					assertEquals(roll.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getX()).angle().signedPrinciple().getDegrees(),1e-12);
					
					
					
				}
			} 
//		}
		System.out.println(" ");
		
		System.out.println("=========== South Pole Testing");
//		//south
//		for(int i=0; i<=0; i+=1)
//		{
//		roll = Angle.inPiRadians(i/1d); 		
		
			roll = Angle.inPiRadians(0d); //default degenerate condition...		
			for(int j=0; j<=1; j+=1)
			{	
				pitch = Angle.inPiRadians( -1/2d + j*0.0001d ); 
				for(int k=0; k<=16; k+=1)
				{
					heading = Angle.inPiRadians(k/8d); 

					z = QuaternionMath.rotate_k(heading.codedPhase()) 
							.rotate_j(pitch.codedPhase())
							.rotate_i(roll.codedPhase());
					h = z.getEuler_k_kji();
					p = z.getEuler_j_kji();
					r = z.getEuler_i_kji();
					Vector3 e = z.getEuler_kji();
					
//					System.out.println(" heading: "+heading.signedPrinciple().toDegreesString(digi)+" == :" + h.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" pitch  : "+pitch.signedPrinciple().toDegreesString(digi)+" == :" + p.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" roll   : "+roll.signedPrinciple().toDegreesString(digi)+" == :" + r.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" is dumped kj: "+ z.getDump_kj());
//					System.out.println( 
//					  " Euler  : roll " + CodedPhase.encodes(e.getX()).angle().signedPrinciple().toDegreesString(digi) 
//				    + " pitch " + CodedPhase.encodes(e.getY()).angle().signedPrinciple().toDegreesString(digi) 
//				    + " head " + CodedPhase.encodes(e.getZ()).angle().signedPrinciple().toDegreesString(digi) 
//				    ); //.getDegrees()); //.toDegreesString(8));					
//					System.out.println(" ");
					
//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
//					   @MAS assertEquals(heading.signedPrinciple().getDegrees(),h.angle().signedPrinciple().getDegrees(),1e-12);
					assertEquals(new Angle(heading).subtract(h.angle()).signedPrinciple().getDegrees(),0.0d,1e-10);
//					
					assertEquals(pitch.signedPrinciple().getDegrees(),p.angle().signedPrinciple().getDegrees(),1e-10);
					assertEquals(roll.signedPrinciple().getDegrees(),r.angle().signedPrinciple().getDegrees(),1e-12);
					
//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
//					   @MAS assertEquals(heading.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getZ()).angle().signedPrinciple().getDegrees(),1e-12);
					assertEquals(new Angle(heading).subtract((CodedPhase.encodes(e.getZ()).angle())).signedPrinciple().getDegrees(),0.0d,1e-10);
//
					assertEquals(pitch.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getY()).angle().signedPrinciple().getDegrees(),1e-10);
					assertEquals(roll.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getX()).angle().signedPrinciple().getDegrees(),1e-12);

					
				}
			} 
//		}
			System.out.println(" ");

		System.out.println("===========Normal Coverage Testing ");
		
		//hemisphere testing exclude north and south pole (pitch less than right angle and roll-dump):	
		// 1. NO test of roll flip (vertical complete inversion)!
		// 2. NO test of pitch obtuse (pole alignment or dumping)!
		for( int i=-31; i<32; i+=1) //NO roll flip of vertical!
		{
			roll = Angle.inPiRadians(i/32d); 
			
			for(int j=-15; j<16; j+=1) //NO pitch pole or dump!
			{	
				pitch = Angle.inPiRadians(j/32d); 
				
				for(int k=0; k<=64; k+=1)
				{
					heading = Angle.inPiRadians(k/32d); //no heading restirction!

					z = QuaternionMath.rotate_k(heading.codedPhase()) 
							.rotate_j(pitch.codedPhase())
							.rotate_i(roll.codedPhase());
					
					r = z.getEuler_i_kji(); //roll
					p = z.getEuler_j_kji(); //pitch
					h = z.getEuler_k_kji(); //heading
					
					Vector3 e = z.getEuler_kji(); //<r,p,h>
					
//					System.out.println(" heading: "+heading.signedPrinciple().toDegreesString(digi)+" == :" + h.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" pitch  : "+pitch.signedPrinciple().toDegreesString(digi)+" == :" + p.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" roll   : "+roll.signedPrinciple().toDegreesString(digi)+" == :" + r.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" is dumped kj: "+ z.getDump_kj());
//					System.out.println(
//						" Euler  : roll " + CodedPhase.encodes(e.getX()).angle().signedPrinciple().toDegreesString(digi) 
//					    + ";     pitch " + CodedPhase.encodes(e.getY()).angle().signedPrinciple().toDegreesString(digi) 
//					    + ";     head " + CodedPhase.encodes(e.getZ()).angle().signedPrinciple().toDegreesString(digi) 
//					); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" ");
					
					
//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
					assertEquals(new Angle(heading).subtract(h.angle()).signedPrinciple().getDegrees(),0.0d,1e-12);
//					
					assertEquals(pitch.signedPrinciple().getDegrees(),p.angle().signedPrinciple().getDegrees(),1e-12);
					assertEquals(roll.signedPrinciple().getDegrees(),r.angle().signedPrinciple().getDegrees(),1e-12);
					
//                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
					assertEquals(new Angle(heading).subtract((CodedPhase.encodes(e.getZ()).angle())).signedPrinciple().getDegrees(),0.0d,1e-12);
//
					assertEquals(pitch.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getY()).angle().signedPrinciple().getDegrees(),1e-12);
					assertEquals(roll.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getX()).angle().signedPrinciple().getDegrees(),1e-12);
				}
			} 
		}
		System.out.println("\n=========== End Normal Attitude Coverage ==============");
		
		System.out.println("===========Dumped Coverage Testing ");
		
		//hemisphere testing exclude north and south pole (pitch less than right angle and roll-dump):	
		// 1. NO test of roll flip (vertical complete inversion)!
		// 2. NO test of pitch obtuse (pole alignment or dumping)!
		for( int i=1; i<2; i+=1) //NO roll flip of vertical!
		{
			roll = Angle.inPiRadians(i/32d); 
			
			for(int j=32; j<64; j+=1) //NO pitch pole or dump!
			{	
				pitch = Angle.inPiRadians(j/32d); 
				
				for(int k=0; k<=64; k+=4)
				{
					heading = Angle.inPiRadians(k/32d); //no heading restirction!

					z = QuaternionMath.rotate_k(heading.codedPhase()) 
							.rotate_j(pitch.codedPhase())
							.rotate_i(roll.codedPhase());
					
					r = z.getEuler_i_kji(); //roll
					p = z.getEuler_j_kji(); //pitch
					h = z.getEuler_k_kji(); //heading
					
					Vector3 e = z.getEuler_kji(); //<r,p,h>
					
					System.out.println(" heading: "+heading.signedPrinciple().toDegreesString(digi)+" == :" + h.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
					System.out.println(" dump pitch  : "+pitch.signedPrinciple().toDegreesString(digi)+" == :" + p.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
					System.out.println(" roll   : "+roll.signedPrinciple().toDegreesString(digi)+" == :" + r.angle().toDegreesString(digi)); //.getDegrees()); //.toDegreesString(8));
					System.out.println(" is normal kj: "+ z.isNormal_kj());
//					System.out.println(
//						" Euler  : roll " + CodedPhase.encodes(e.getX()).angle().signedPrinciple().toDegreesString(digi) 
//					    + ";     pitch " + CodedPhase.encodes(e.getY()).angle().signedPrinciple().toDegreesString(digi) 
//					    + ";     head " + CodedPhase.encodes(e.getZ()).angle().signedPrinciple().toDegreesString(digi) 
//					); //.getDegrees()); //.toDegreesString(8));
//					System.out.println(" ");
					
					
////                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
//					assertEquals(new Angle(heading).subtract(h.angle()).signedPrinciple().getDegrees(),0.0d,1e-12);
////					
//					assertEquals(pitch.signedPrinciple().getDegrees(),p.angle().signedPrinciple().getDegrees(),1e-12);
//					assertEquals(roll.signedPrinciple().getDegrees(),r.angle().signedPrinciple().getDegrees(),1e-12);
//					
////                  Problems with discontinuity at straight angles:	re-phrase as angle difference assertion...				
//					assertEquals(new Angle(heading).subtract((CodedPhase.encodes(e.getZ()).angle())).signedPrinciple().getDegrees(),0.0d,1e-12);
////
//					assertEquals(pitch.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getY()).angle().signedPrinciple().getDegrees(),1e-12);
//					assertEquals(roll.signedPrinciple().getDegrees(),CodedPhase.encodes(e.getX()).angle().signedPrinciple().getDegrees(),1e-12);
				}
			} 
		}
		System.out.println("\n=========== End Dumped Attitude Coverage ==============");
		
		
		
//		heading = Angle.inDegrees(0d);
//		pitch = Angle.inDegrees(-90d);
//		roll = Angle.inDegrees(-172.5d);
//		
//		Quaternion z0 = QuaternionMath.rotate_k(heading.codedPhase()) 
//				.rotate_j(pitch.codedPhase())
//				.rotate_i(roll.codedPhase());
//		
//		heading = Angle.inDegrees(-172.5d);
//		pitch = Angle.inDegrees(-90d);
//		roll = Angle.inDegrees(0d);
//		
//		Quaternion z1 = QuaternionMath.rotate_k(heading.codedPhase()) 
//				.rotate_j(pitch.codedPhase())
//				.rotate_i(roll.codedPhase());
//		
//		heading = Angle.inDegrees(172.5d);
//		pitch = Angle.inDegrees(-90d);
//		roll = Angle.inDegrees(0d);
//		
//		Quaternion z2 = QuaternionMath.rotate_k(heading.codedPhase()) 
//				.rotate_j(pitch.codedPhase())
//				.rotate_i(roll.codedPhase());
//		
//		heading = Angle.inDegrees(0d);
//		pitch = Angle.inDegrees(90d);
//		roll = Angle.inDegrees(-172.5d);
//		
//		Quaternion z3 = QuaternionMath.rotate_k(heading.codedPhase()) 
//				.rotate_j(pitch.codedPhase())
//				.rotate_i(roll.codedPhase());
//		
//		heading = Angle.inDegrees(0d);
//		pitch = Angle.inDegrees(90d);
//		roll = Angle.inDegrees(-172.5d);
//		
//		Quaternion z4 = QuaternionMath.rotate_k(heading.codedPhase()) 
//				.rotate_j(pitch.codedPhase())
//				.rotate_i(roll.codedPhase());
//		
//		heading = Angle.inDegrees(172.5d);
//		pitch = Angle.inDegrees(90d);
//		roll = Angle.inDegrees(0d);
//		
//		Quaternion z5 = QuaternionMath.rotate_k(heading.codedPhase()) 
//				.rotate_j(pitch.codedPhase())
//				.rotate_i(roll.codedPhase());
//		
//		
//		
//		
//		h =  Angle.inDegrees(10).codedPhase();
//		p =  Angle.inDegrees(30).codedPhase();
//		r =  Angle.inDegrees(40).codedPhase();
//		
//		QuaternionMath.rotate_j(p);
//		System.out.println(z.toString());
//		z.unit();
//		System.out.println(z.toString());
//			
//		z = QuaternionMath.rotate_k(h).rotate_j(p).rotate_i(r);
//
//		h.set(z.getEuler_k_kji());
//		p.set(z.getEuler_j_kji());
//		r.set(z.getEuler_i_kji());
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 10d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), 30d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), 40d, 1e-13);
//
//		h = Angle.inDegrees(-20).codedPhase();
//		p = Angle.inDegrees(50).codedPhase();
//		r = Angle.inDegrees(10).codedPhase();
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), 50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
//
//		z = QuaternionMath.rotate_k(h).rotate_j(p).rotate_i(r);// .unit();
//
//		h.set(z.getEuler_k_kji());
//		p.set(z.getEuler_j_kji());
//		r.set(z.getEuler_i_kji());
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), 50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
//
//		h = Angle.inDegrees(-20).codedPhase();
//		p = Angle.inDegrees(-50).codedPhase();
//		r = Angle.inDegrees(10).codedPhase();
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), -50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
//
//		z = QuaternionMath.rotate_k(h).rotate_j(p).rotate_i(r);// .unit();
//
//		h.set(z.getEuler_k_kji());
//		p.set(z.getEuler_j_kji());
//		r.set(z.getEuler_i_kji());
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), -50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
//
//		h = Angle.inDegrees(-20).codedPhase();
//		p = Angle.inDegrees(50).codedPhase();
//		r = Angle.inDegrees(-10).codedPhase();
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), 50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), -10d, 1e-13);
//
//		z = QuaternionMath.rotate_k(h).rotate_j(p).rotate_i(r);// .unit();
//
//		h.set(z.getEuler_k_kji());
//		p.set(z.getEuler_j_kji());
//		r.set(z.getEuler_i_kji());
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), 50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), -10d, 1e-13);
//
//		h = Angle.inDegrees(-20).codedPhase();
//		p = Angle.inDegrees(50).codedPhase();
//		r = Angle.inDegrees(10).codedPhase();
//
//		Vector3 v = new Rotator(1, 0, 0, 0).getEuler_kji().multiply(
//				180 / StrictMath.PI);
//		assertTrue(v.isEquivalent(new Vector3(Vector3.ZERO), 1e-13));
//
//		v = z.getEuler_kji();
//
//		assertEquals(h.angle().unsignedPrinciple().getDegrees(), 340d, 1e-13);
//		assertEquals(p.angle().signedPrinciple().getDegrees(), 50d, 1e-13);
//		assertEquals(r.angle().signedPrinciple().getDegrees(), 10d, 1e-13);
//		
//		
//		h.set(CodedPhase.STRAIGHT);
//		h.negate();
//		System.out.println(h.angle().getDegrees());
//		h = Angle.inDegrees(-180).codedPhase();
//		System.out.println(h.angle().getDegrees());
//		p = Angle.inDegrees(89).codedPhase();
//		r = Angle.inDegrees(0).codedPhase();
//		z = QuaternionMath.rotate_k(h).rotate_j(p).rotate_i(r);// .unit();
//		h.set(z.getEuler_k_kji());
//		//p.set(z.getEuler_j_kji());
//		//r.set(z.getEuler_i_kji());
//		Angle w = Angle.inDegrees(-180);
//		System.out.println(w.getDegrees());
//		System.out.println(h.angle().getDegrees());
//		//System.out.println(p.angle().getDegrees());
//		//System.out.println(r.angle().getDegrees());
//		
/////		assertTrue(h.angle().getDegrees()==1d);
		

		// fail("Not yet implemented");
	}
	

	/**
	 * Test method for magnification to re-scale Operator
	 * {@link rotation.Rotator#getNormaliztionFactor()}.
	 */
	@Test
	public void testGetScaleFactor() {
		Rotator z = new Rotator(1, 2, 3, 4);
		assertEquals(z.getNormaliztionFactor(), 1.d / StrictMath.sqrt(1 + 4 + 9 + 16),
				1e-12);
		assertEquals(z.getImageNormalizationFactor(), 1d / (1 + 4 + 9 + 16), 1e-12);

		assertEquals(1 / z.getDeterminant(), z.getImageNormalizationFactor(), 1e-12);
		assertEquals(z.getNormaliztionFactor() * z.getNormaliztionFactor(), z.getImageNormalizationFactor(),
				1e-12);

		// fail("Not yet implemented");
	}

	/**
	 * Test method for {@link rotation.Rotator#getImage_i()}.
	 */
	@Test
	public final void testImagePreimage() {
		
		Rotator t = new Rotator(Quaternion.IDENTITY).rotate_i( 
				Angle.inDegrees(30).codedPhase());
//		System.out.println("==> image i"+new Rotator(t).getImage_i().unit().toString());
//		System.out.println("==> image j"+new Rotator(t).getImage_j().unit().toString());
//		System.out.println("==> image k"+new Rotator(t).getImage_k().unit().toString());
		assertTrue((new Rotator(t).getImage_i().unit()).isEquivalent(new Vector3(1d,0d,0d), 1e-11));
		assertTrue(new Rotator(t).getImage_j().unit().isEquivalent(new Vector3(0d,StrictMath.sqrt(3)/2,0.5), 1e-12));
		assertTrue(new Rotator(t).getImage_k().unit().isEquivalent(new Vector3(0,-.5,StrictMath.sqrt(3)/2), 1e-13));
		
//		System.out.println("==> pre-image i"+t.preImage_i().toString());
//		System.out.println("==> pre-image j"+t.preImage_j().toString());
//		System.out.println("==> pre-image k"+t.preImage_k().toString());
		assertTrue(t.getPreImage_i().unit().isEquivalent(new Vector3(1,0,0), 1e-13));
		assertTrue(t.getPreImage_j().unit().isEquivalent(new Vector3(0d,StrictMath.sqrt(3)/2,-0.5), 1e-13));
		assertTrue(t.getPreImage_k().unit().isEquivalent(new Vector3(0,.5,StrictMath.sqrt(3)/2), 1e-13));

		
//		System.out.println("==> pre-image i-hat"+t.preimage(new Vector3(1,0,0)).toString());
//		System.out.println("==> pre-image j-hat"+t.preimage(new Vector3(0,1,0)).toString());
//		System.out.println("==> pre-image k-hat"+t.preimage(new Vector3(0,0,1)).toString());
		assertTrue(t.getPreImage(new Vector3(1,0,0)).isEquivalent(new Vector3(1,0,0), 1e-13));
		assertTrue(t.getPreImage(new Vector3(0,1,0)).isEquivalent(new Vector3(0d,StrictMath.sqrt(3)/2,-0.5), 1e-13));
		assertTrue(t.getPreImage(new Vector3(0,0,1)).isEquivalent(new Vector3(0,.5,StrictMath.sqrt(3)/2), 1e-13));

//		System.out.println("a===> pre-image i-hat"+new Operator(t).preimage(new Quaternion(0,1,0,0)).toString());
//		System.out.println("b===> pre-image j-hat"+new Operator(t).preimage(new Quaternion(0,0,1,0)).toString());
//		System.out.println("c===> pre-image k-hat"+new Operator(t).preimage(new Quaternion(0,0,0,1)).toString());
		assertTrue(new Rotator(t).getPreImage(new Rotator(0,1,0,0)).isEquivalent(new Quaternion(0,1,0,0), 1e-13));
		assertTrue(new Rotator(t).getPreImage(new Rotator(0,0,1,0)).isEquivalent(new Rotator(0,0d,StrictMath.sqrt(3)/2,-0.5), 1e-13));
		assertTrue(new Rotator(t).getPreImage(new Rotator(0,0,0,1)).isEquivalent(new Rotator(0,0,.5,StrictMath.sqrt(3)/2), 1e-13));
	
		t.multiply(2);
//		System.out.println("==> 2*t image i"+t.image_i().toString());
//		System.out.println("==> 2*t image j"+t.image_j().toString());
//		System.out.println("==> 2*t image k"+t.image_k().toString());
		assertTrue(t.getImage_i().unit().isEquivalent(new Vector3(1,0,0), 1e-13));
		assertTrue(t.getImage_j().unit().isEquivalent(new Vector3(0,StrictMath.sqrt(3)/2,.5), 1e-13));
		assertTrue(t.getImage_k().unit().isEquivalent(new Vector3(0,-.5,StrictMath.sqrt(3)/2), 1e-13));
			//assertTrue(t.image_i().equals(new Vector3(1,0,0), 1e-13));

//		System.out.println("==> 2*t image i-hat"+t.image(new Vector3(1,0,0)).toString());
//		System.out.println("==> 2*t image j-hat"+t.image(new Vector3(0,1,0)).toString());
//		System.out.println("==> 2*t image k-hat"+t.image(new Vector3(0,0,1)).toString());
		assertTrue(t.getImage(new Vector3(1,0,0)).unit().isEquivalent(new Vector3(1,0,0), 1e-13));
		assertTrue(t.getImage(new Vector3(0,1,0)).unit().isEquivalent(new Vector3(0,StrictMath.sqrt(3)/2,.5), 1e-13));
		assertTrue(t.getImage(new Vector3(0,0,1)).unit().isEquivalent(new Vector3(0,-.5,StrictMath.sqrt(3)/2), 1e-13));

//		System.out.println("===> 2*t image i-hat"+new Operator(t).image(new Operator(0,1,0,0)).toString());
//		System.out.println("===> 2*t image j-hat"+new Operator(t).image(new Operator(0,0,1,0)).toString());
//		System.out.println("===> 2*t image k-hat"+new Operator(t).image(new Operator(0,0,0,1)).toString());
		assertTrue(new Rotator(t).getImage(new Rotator(0,1,0,0)).unit().isEquivalent(new Quaternion(0,1,0,0), 1e-13));
		assertTrue(new Rotator(t).getImage(new Rotator(0,0,1,0)).unit().isEquivalent(new Rotator(0,0,StrictMath.sqrt(3)/2,.5), 1e-13));
		assertTrue(new Rotator(t).getImage(new Quaternion(0,0,0,1)).unit().isEquivalent(new Quaternion(0,  0,-.5,StrictMath.sqrt(3)/2), 1e-13));
		
//		fail("Not yet implemented");
	}


	/**
	 * Test method for constructing operator from scalar and vector components
	 * {@link rotation.Rotator#Operator(double, rotation.Vector3)}.
	 */
	@Test
	public void testOperatorDoubleVector3() {
		double d = 1;
		Vector3 v = new Vector3(2, 3, 4);
		Rotator z = new Rotator(d, v);
		Rotator t = new Rotator(1, 2, 3, 4);
		assertTrue(z.isEquivalent(t, 1e-13));
		// fail("Not yet implemented");
	}


	/**
	 * Test method for constructing Operator from Quaternion
	 * {@link rotation.Rotator#Operator(rotation.Quaternion)}.
	 */
	@Test
	public void testOperatorQuaternion() {
		Rotator t = new Rotator(1, 2, 3, 4);
		Rotator z = new Rotator(t);
		assertTrue(z.isEquivalent(t, 1e-13));
		// fail("Not yet implemented");
	}





	/**
	 * Test method for {@link rotation.Rotator#turn_i()}.
	 */
	@Test
	public void testTurn() {
		Rotator z = new Rotator(0,0,1,0);
		z.turn_i().multiply(z.getNormaliztionFactor());
		System.out.println("turn i(j): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,0,StrictMath.sqrt(2)/2,StrictMath.sqrt(2)/(-2)),1e-12));
		z.set(0,0,0,1);
		z.turn_j().multiply(z.getNormaliztionFactor());
//		System.out.println("turn j(k): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,StrictMath.sqrt(2)/(2),0,StrictMath.sqrt(2)/-2),1e-13));
		z.set(0,1,0,0);
		z.turn_k().multiply(z.getNormaliztionFactor());
//		System.out.println("turn k(i): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,StrictMath.sqrt(2)/(2),StrictMath.sqrt(2)/(-2),0),1e-13));
		
		Vector3 v = new Vector3(1,0,0);
		z.set(0,0,1,0);
		z.turn(v).multiply(z.getNormaliztionFactor());
//		System.out.println("turn i(j): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,0,StrictMath.sqrt(2)/-2,StrictMath.sqrt(2)/(2)),1e-13));
		
		v.set(0,1,0);
		z.set(0,0,0,1);
		z.turn(v).multiply(z.getNormaliztionFactor());
//		System.out.println("turn j(k): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,StrictMath.sqrt(2)/(-2),0,StrictMath.sqrt(2)/2),1e-13));
		
		
		v.set(0,0,1);
		z.set(0,1,0,0);
		z.turn(v).multiply(z.getNormaliztionFactor());
//		System.out.println("turn k(i): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,StrictMath.sqrt(2)/(2),StrictMath.sqrt(2)/(-2),0),1e-13));

		v.set(Vector3.ZERO); //no axis to turn about...
		z.set(0,1,0,0);
		z.turn(v);
//		System.out.println("a. turn i(0): "+z.toString());
		z.multiply(z.getNormaliztionFactor());
//		System.out.println("b. turn i(0): "+z.toString());

		z.set(0,1,0,0);
		z.turn(v).multiply(z.getNormaliztionFactor());
//		System.out.println("c. turn i(0): "+z.toString());
		assertTrue(z.isEquivalent(new Rotator(0,1,0,0),1e-13));
		
		//composite flip-check...
		
		Quaternion q = new Quaternion(1,2,3,4);
		z.set(q);		
		z.turn_i().turn_j().turn_k().turn_i().turn_j().turn_k().multiply(1/8d);
		assertTrue(z.negate().equals(q));

		z.set(q);		
		z.turn_i().turn_i().turn_j().turn_j().turn_k().turn_k().multiply(1/8d);
		assertTrue(z.negate().equals(q));

		//		fail("Not yet implemented");
	}


}
