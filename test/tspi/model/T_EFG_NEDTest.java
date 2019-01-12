/**
 * 
 */
package tspi.model;

import junit.framework.TestCase;
import tspi.rotation.Angle;
import tspi.rotation.CodedPhase;
import tspi.rotation.Rotator;
import tspi.rotation.Vector3;

/**
 * @author mike
 *
 */
public class T_EFG_NEDTest extends TestCase {

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#T_EFG_NED(tspi.model.T_EFG_NED)}.
	 * Class that converts EFG geocentric to NED topocentric...
	 */
	public final void testT_EFG_NEDT_EFG_NED() {
		
		//construct nominal: Ellipsoid s1,s2
		double h1=1000d;
		double h2=h1;
		Ellipsoid s1 = new Ellipsoid();
		s1.setNorthLatitude(Angle.inDegrees(30));
		s1.setEastLongitude(Angle.inDegrees(240));
		s1.setEllipsoidHeight(1000d);
          assertEquals(s1.getEastLongitude().getDegrees(),240d,1e-13);
          assertEquals(s1.getNorthLatitude().getDegrees(),30,0d);
          assertEquals(s1.getEllipsoidHeight(),1000d,0);
  		
          Vector3 v2 = s1.getGeocentric(); //new Vector3(Vector3.EMPTY);
          System.out.println("Geocentric:"+v2.toString(8)); //verified s1 XYZ coordinates
          
        Ellipsoid s2 = new Ellipsoid();		
		Vector3 v1 = new Vector3(Vector3.EMPTY); //construct empty: Vector3 v1

		Angle lat1 =s1.getNorthLatitude();
		Angle lon1 = s1.getEastLongitude();
		//construct with static factory: Rotator q1
		Rotator q1 = T_EFG_NED.local(s1.getNorthLatitude(),s1.getEastLongitude());
		
		Vector3 geodeticEuler = q1.getEuler_kji();
		CodedPhase ak1 = q1.getEuler_k_kj();
		CodedPhase aj1 = q1.getEuler_j_kj();
		System.out.println("z:"+ak1.angle().toDegreesString(8));
		System.out.println("y:"+aj1.angle().toDegreesString(8));
		
		
		
		
		System.out.println("x:"+CodedPhase.encodes(geodeticEuler.getX()).angle().toDegreesString(8));
		System.out.println("y:"+CodedPhase.encodes(geodeticEuler.getY()).angle().toDegreesString(8));
		System.out.println("z:"+CodedPhase.encodes(geodeticEuler.getZ()).angle().toDegreesString(8));
		
		//create with Roator and double: T_EFG_NED p1
		T_EFG_NED p1 = new T_EFG_NED(q1,s1.getEllipsoidHeight());				
		     assertEquals(s1.getEllipsoidHeight(),p1.getLocalHeight(),0);
		     assertTrue(p1.getLocal().isEquivalent(q1,1e-20));
				
		//construct with static factory: Rotator q1
		q1 = T_EFG_NED.local(s1.getNorthLatitude().codedPhase(),s1.getEastLongitude().codedPhase());
		
		//create with Roator and double: T_EFG_NED p1
		p1 = new T_EFG_NED(q1,s1.getEllipsoidHeight());				
		     assertEquals(s1.getEllipsoidHeight(),p1.getLocalHeight(),0);
		     assertTrue(p1.getLocal().isEquivalent(q1,1e-20));

		//Extract Ellipsoid perspective and check member components for later comparisons:
		s1 = p1.getEllipsoid();
		    System.out.println("East : "+s1.getEastLongitude().toDegreesString(8));
		    System.out.println("North: "+s1.getNorthLatitude().toDegreesString(8));
		    System.out.println("height: "+s1.getEllipsoidHeight());
		
			 assertEquals(s1.getEastLongitude().getPiRadians(),4/3d,1e-13);
			 assertEquals(s1.getNorthLatitude().getPiRadians(),1/6d,1e-15);
			 assertEquals(s1.getEllipsoidHeight(),1000d,0);
		//Extract geocentric perspective for later comparisons:
		    v1 = p1.getGeocentric();
		     assertTrue(v1.isEquivalent(v2,1e-8));
		    
		//copy construct: T_EFG_NED p2	
		T_EFG_NED p2 = new T_EFG_NED(p1); //copy constructor		
			h2 = p2.getLocalHeight();	
			s2 = p2.getEllipsoid();  //get Ellipsoid
			v2 = p2.getGeocentric(); //get geocentric Vector3			
			assertTrue(v1.equals(v2));
			assertEquals(s2.getEastLongitude().getPiRadians(),s1.getEastLongitude().getPiRadians(),1e-13);
			assertEquals(s2.getNorthLatitude().getPiRadians(),s1.getNorthLatitude().getPiRadians(),1e-17);
			assertEquals(s2.getEllipsoidHeight(),s1.getEllipsoidHeight(),0);
			assertEquals(h1,h2,0d);

        //set with p1: T_EFG_NED p2
		p2.set(p1);
		h2 = p2.getLocalHeight();	
		s2 = p2.getEllipsoid();  //get Ellipsoid
		v2 = p2.getGeocentric(); //get geocentric Vector3			
			assertTrue(v1.equals(v2));
			assertEquals(s2.getEastLongitude().getPiRadians(),s1.getEastLongitude().getPiRadians(),1e-13);
			assertEquals(s2.getNorthLatitude().getPiRadians(),s1.getNorthLatitude().getPiRadians(),1e-17);
			assertEquals(s2.getEllipsoidHeight(),s1.getEllipsoidHeight(),0);
			assertEquals(h1,h2,0d);
		
		//clear
		p1 = new T_EFG_NED();
			assert(p1.getLocal().equals(Rotator.EMPTY));
			assertTrue(Double.isNaN(p1.getLocalHeight()));
		
		//set component members: T_EFG_NED p1
		p1.setLocalHeight(1000);
		p1.setLocal(T_EFG_NED.local(s1.getNorthLatitude(),s1.getEastLongitude()));;		
			assertTrue(v1.equals(v2));
			assertEquals(s2.getEastLongitude().getPiRadians(),s1.getEastLongitude().getPiRadians(),1e-13);
			assertEquals(s2.getNorthLatitude().getPiRadians(),s1.getNorthLatitude().getPiRadians(),1e-17);
			assertEquals(s2.getEllipsoidHeight(),s1.getEllipsoidHeight(),0);
			assertEquals(h1,h2,0d);

		//clear
		p1.clear();
			assert(p1.getLocal().equals(Rotator.EMPTY));
			assertTrue(Double.isNaN(p1.getLocalHeight()));
			
			
		//set with geocentric perspective: T_EFG_NED p1
		p1.clear(); //ensure wiped.
		p1.set(p2.getGeocentric()); //set with geocentric		    
		    assertTrue(p1.getGeocentric().isEquivalent(new Vector3(v2),1e-8));
		h1 = p1.getLocalHeight();
		v1 = p1.getGeocentric();
		s1 = p1.getEllipsoid();
    		assertTrue(v2.isEquivalent(v1, 1e-8));    		
			assertEquals(s2.getEastLongitude().getPiRadians(),s1.getEastLongitude().getPiRadians(),1e-13);
			assertEquals(s2.getNorthLatitude().getPiRadians(),s1.getNorthLatitude().getPiRadians(),1e-13);
			assertEquals(s2.getEllipsoidHeight(),s1.getEllipsoidHeight(),1e-8);
			assertEquals(h2,h1,1e-8);
		
		p1.clear();
		
        //set with ellipsoid perspective: T_EFG_NED p1
		p1.set(p2.getEllipsoid()); //set with Ellipsoid of that set geocentric
		s1 = p1.getEllipsoid();
		v1 = p1.getGeocentric();
		h1 = p1.getLocalHeight();
		assertTrue(v2.isEquivalent(v1,1e-8));
		assertEquals(s2.getEastLongitude().getPiRadians(),s1.getEastLongitude().getPiRadians(),1e-13);
		assertEquals(s2.getNorthLatitude().getPiRadians(),s1.getNorthLatitude().getPiRadians(),1e-15);
		assertEquals(s2.getEllipsoidHeight(),s1.getEllipsoidHeight(),0);
		assertEquals(h2,h1,0d);
		
		
		int cnt = 0;
		Ellipsoid geodetic = new Ellipsoid();
		Ellipsoid tgeodetic = new Ellipsoid();

		Double qHeight = Double.NaN;
		
		//Note: efg cannot handle dumping: ambiguous Euler assignments!
		
		Vector3 efg = new Vector3(Vector3.EMPTY);
		Vector3 tefg = new Vector3(Vector3.EMPTY);
		double qlat;
		double qlon;
		double qhgt;
//		for (int k = -1; k <= 1; k++) {
			double fudge = 0; //k * 0.00d;
			for (int i = -3; i <= 3; i++) { // latitudes [-90..90]
				double phi = i * 30.0d; // + 0.001d;
				for (int j = 0; j <= 12; j++) { // longitudes [-360..360]
					double lambda = j * 30.0d + fudge; // - 0.01d;
					for (int h = -1; h <= 2; h++) { // heights meters above, on, and below ellipsoid
						double hgt = h * 1000.0d + fudge; // - 0.1d;
//						System.out.print(phi+" "+lambda +" "+hgt);
//						System.out.print(Angle.inDegrees(phi).signedPrinciple()+" "+Angle.inDegrees(lambda).signedPrinciple() +" "+hgt);

//								+ " near rev = "
//						 + StrictMath.round(Angle.inDegrees(j*30).getRevolutions()));

						p1.setLocal(T_EFG_NED.local(Angle.inDegrees(phi).signedPrinciple(),Angle.inDegrees(lambda).signedPrinciple()));
						p1.setLocalHeight(hgt);

						geodetic.set(p1.getEllipsoid());	
//						System.out.print(p1.getEllipsoid().getNorthLatitude()+" "+p1.getEllipsoid().getEastLongitude()+" "+p1.getLocalHeight());

						System.out.print(geodetic.getNorthLatitude().toDegreesString(8)+" "+geodetic.getEastLongitude().toDegreesString(8)+" "+geodetic.getEllipsoidHeight());
						
						tgeodetic.setGeocentric(p1.getGeocentric());

						qlat = tgeodetic.getNorthLatitude().getDegrees();
//						assertEquals(qlat, phi, 1e-13);
						qlon = tgeodetic.getEastLongitude().unsignedPrinciple().getDegrees();
						qhgt = tgeodetic.getEllipsoidHeight();
//						System.out.print(String.format(" Qlat = %14.10f", qlat));
////						System.out.print(String.format(" phi = %14.10f", phi));
//						System.out.print(String.format(" Qlon = %15.10f", qlon));
////						System.out.print(String.format(" lambda = %15.10f", lambda));
////						System.out.print(String.format(" height = %8.3f", hgt));
//						System.out.print(String.format(" Qheight = %8.3f", qhgt));

//						assertEquals(Angle.inDegrees(qlon).unsignedPrinciple().getDegrees(), Angle.inDegrees(lambda).unsignedPrinciple().getDegrees(), 1e-13);
//						assertEquals(qhgt, -hgt, 1e-8);

						System.out.println();
						cnt = cnt + 1;

					}
				}
			}
//		}
		

		
		
		
		
	}


	/**
	 * Test method for {@link tspi.model.T_EFG_NED#getEllipsoid()}.
	 */
	public final void testGetEllipsoid() {
		fail("Not yet implemented"); // TODO
	}


	/**
	 * Test method for {@link tspi.model.T_EFG_NED#getGeocentric()}.
	 */
	public final void testGetGeocentric() {
		fail("Not yet implemented"); // TODO
	}



}
