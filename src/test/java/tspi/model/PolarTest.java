/**
 * 
 */
package tspi.model;

import junit.framework.TestCase;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;

/**
 * @author mike
 *
 */
public class PolarTest extends TestCase {

	/**
	 * Test method for {@link tspi.model.Polar#Polar()}.
	 */
	public final void testPolar() {
		Polar p = new Polar();
		assertTrue(Double.isNaN(p.getRange()));
		assertTrue(Double.isNaN(p.getUnsignedAzimuth().getPiRadians()));
		assertTrue(Double.isNaN(p.getElevation().getPiRadians()));
//		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.Polar#Polar(double, rotation.Angle, rotation.Angle)}.
	 */
	public final void testPolarDoubleAngleAngle() {
		Polar p = new Polar(10000d, Angle.inDegrees(120d), Angle.inDegrees(30d));
		assertEquals(10000d,p.getRange(),0d);
		assertEquals(120d,p.getUnsignedAzimuth().getDegrees(),0d);
		assertEquals(30d,p.getElevation().getDegrees(),0d);

//		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.Polar#Polar(tspi.model.Polar)}.
	 */
	public final void testPolarPolar() {
		Polar q = new Polar(10000d, Angle.inDegrees(120d), Angle.inDegrees(30d));
		Polar p = new Polar(q);
		assertEquals(10000d,p.getRange(),0d);
		assertEquals(120d,p.getUnsignedAzimuth().getDegrees(),0d);
		assertEquals(30d,p.getElevation().getDegrees(),0d);
//		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.Polar#getUnsignedAzimuth()}.
	 */
	public final void testGetSetAzimuth() {
		//Unsigned principle Angle Factory.class..
		Polar p = new Polar(10000d, Angle.inDegrees(120d), Angle.inDegrees(30d));

		//Angle factory:
		Angle t = p.getUnsignedAzimuth();
		assertEquals(120d,t.getDegrees(),0);
		t.set(Angle.inDegrees(30));
		assertEquals(30d,t.getDegrees(),0);
		assertEquals(120d,p.getUnsignedAzimuth().getDegrees(),0d);

		//gets unsigned values:
		p.setAzimuth(Angle.inDegrees(-60));
		//assertEquals(-60d,p._azimuth.getDegrees(),0d);
		assertEquals(300d,p.getUnsignedAzimuth().getDegrees(),0d);
        //gets principle unsigned values:[0..360)
		p.setAzimuth(Angle.inDegrees(360));
		//assertEquals(360d,p._azimuth.getDegrees(),0d);
		//assertEquals(0d,p.getAzimuth().getDegrees(),0d);
		p.setAzimuth(Angle.inDegrees(-420));
		//assertEquals(-420d,p._azimuth.getDegrees(),0d);
		assertEquals(300d,p.getUnsignedAzimuth().getDegrees(),0d);


		//fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.Polar#getElevation()}.
	 */
	public final void testGetSetElevation() {
		//Unsigned principle Angle Factory.class..
		Polar p = new Polar(10000d, Angle.inDegrees(120d), Angle.inDegrees(30d));

		//Angle factory test:
		Angle t = p.getElevation();
		assertEquals(30d,t.getDegrees(),0);
		t.set(Angle.inDegrees(120));
		assertEquals(120d,t.getDegrees(),0);
		assertEquals(30d,p.getElevation().getDegrees(),0d);

		//gets signed values test:

		p.setElevation(Angle.inDegrees(-60));
		//assertEquals(-60d,p._elevation.getDegrees(),0d);
		assertEquals(-60d,p.getElevation().getDegrees(),0d);

		p.setElevation(Angle.inDegrees(-120));
		//assertEquals(-120d,p._elevation.getDegrees(),0d);
		assertEquals(-120d,p.getElevation().getDegrees(),0d);

		p.setElevation(Angle.inDegrees(60));
		//assertEquals(60d,p._elevation.getDegrees(),0d);
		assertEquals(60d,p.getElevation().getDegrees(),0d);

		p.setElevation(Angle.inDegrees(120));
		//assertEquals(120d,p._elevation.getDegrees(),0d);
		assertEquals(120d,p.getElevation().getDegrees(),0d);

        //test it gets principle unsigned values:[0..360)
		p.setElevation(Angle.inDegrees(360));
		//assertEquals(360d,p._elevation.getDegrees(),0d);
		assertEquals(0d,p.getElevation().getDegrees(),0d);

		p.setElevation(Angle.inDegrees(-420));
		//assertEquals(-420d,p._elevation.getDegrees(),0d);
		assertEquals(-60d,p.getElevation().getDegrees(),1e-13); //note roundoff loss...
		// lesson: controll magnitude of input angles when have fractional concern...

		//fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.Polar#getRange()}.
	 */
	public final void testGetSetRange() {

		Polar p = new Polar(10000d, Angle.inDegrees(120d), Angle.inDegrees(30d));
		//assertEquals(10000d,p._range,0d);
		assertEquals(10000d,p.getRange(),0d);
		assertEquals(120d,p.getUnsignedAzimuth().getDegrees(),0d);
		assertEquals(30d,p.getElevation().getDegrees(),0d);

		//note: negative and zero inputs allowed... even though may be nonsense.
		p.setRange(0d);
		//assertEquals(0d,p._range,0d);
		assertEquals(0d,p.getRange(),0d);
		p.setRange(-100d);
		//assertEquals(-100d,p._range,0d);
		assertEquals(-100d,p.getRange(),0d);

		//angles interpreted independent of sign of range!
		assertEquals(120d,p.getUnsignedAzimuth().getDegrees(),0d);
		assertEquals(30d,p.getElevation().getDegrees(),0d);

//		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.Polar#getTopocentric()}.
	 */
	public final void testGetSetTopocentric() {
		Polar r = new Polar();
		Polar q = new Polar();
		Vector3 v = new Vector3(Vector3.EMPTY);
		Vector3 w = new Vector3(Vector3.EMPTY);
		
		Polar p = new Polar(10000d, Angle.inDegrees(120d), Angle.inDegrees(30d));
		
		for ( int i = -8; i <= 8; i++){
			for ( int j = -8; j <= 8; j++){
				for ( int k = -8; k <= 8; k++){
					
					v.set(i,j,k);
					p.setTopocentric(v);
					
					System.out.println(
						v.toString(5)+"  "
					    + p.getRange() + " "
					    + p.getUnsignedAzimuth().getDegrees() + " "
					    + p.getElevation().getDegrees() 
					);
					
					q.set(p.getRange(), p.getUnsignedAzimuth(), p.getElevation());
					r.set(p);
					w.set(p.getTopocentric());
					//w.set(p.getTopocentric());
					//w.set(v);
					System.out.println(w.toString(5) );
					System.out.println(" ");
					assertTrue(v.isEquivalent(w,1e-14));	
					q.clear(q.getRange());
					q.clear();
					q.set(new Polar());
				}
				
			}
			
		}
		
		//fail("Not yet implemented"); // TODO
	}



}
