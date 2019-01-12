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
public class LocationTest extends TestCase {

	public final void testWGS84WGS84() {
		int cnt = 0;
		Ellipsoid geodetic = new Ellipsoid();
		Ellipsoid tgeodetic = new Ellipsoid();

		Double qHeight = Double.NaN;
		Rotator q_EFG_NED = new Rotator(Rotator.EMPTY);
		Vector3 efg = new Vector3(Vector3.EMPTY);
		Vector3 tefg = new Vector3(Vector3.EMPTY);
		T_EFG_NED tNavigation = new T_EFG_NED();
		double qlat;
		double qlon;
		for (int i = 1; i <= 3; i++) { //lat seed -- phi	
			double phi = i * 30.0d; 
			
			for (int j = 0; j <= 12; j++) { //longitude seed -- lambda			
				double lambda = j * 30.0d;
				
				//for (int h = -1; h <= 2; h++) { //height seed -- hgt
				int h=2; {					
					double hgt = h * 1000.0d; //meters height above and below ellipsoid
					
					//set Ellipsoid object..: geodetic
//					geodetic.setNorthLatitude(Angle.inDegrees(phi));
//					geodetic.setEastLongitude(Angle.inDegrees(lambda));
					//System.out.println(phi+"  "+lambda + " near rev = "+StrictMath.round(Angle.inDegrees(j*30).getRevolutions()));
//					geodetic.setEllipsoidHeight(hgt);
					
					System.out.println("  lat "+ phi + "  lon "+ lambda);
					geodetic.set(Angle.inDegrees(phi),Angle.inDegrees(lambda),hgt);
					
					//set Geocentric with Ellipsoid object					
					efg.set(geodetic.getGeocentric());
					
					//set transform to navigation object with Ellipsoid coordinate object
					tNavigation.set(geodetic);
					
					tefg.set(tNavigation.getGeocentric());
					System.out.println("  geodetic lat"+ geodetic.getNorthLatitude().getDegrees() + "  lon " + geodetic.getEastLongitude().getDegrees() + " hgt " + geodetic.getEllipsoidHeight() );
					
					CodedPhase dumperi = tNavigation.getLocal().getEuler_i_kji();
					CodedPhase dumperj = tNavigation.getLocal().getEuler_j_kji();
					CodedPhase dumperk = tNavigation.getLocal().getEuler_k_kji();
					
					CodedPhase dumperjd = tNavigation.getLocal().getEuler_j_kj();
					CodedPhase dumperkd = tNavigation.getLocal().getEuler_k_kj();
					
System.out.println("J = "+ dumperj.angle().toDegreesString(10));					
System.out.println("K = "+ dumperk.angle().toDegreesString(10));					
System.out.println("dumper J = "+ dumperjd.angle().toDegreesString(10));					
System.out.println("dumper K = "+ dumperkd.angle().toDegreesString(10));					
					
					
					System.out.println("  navigation lat"+ tNavigation.getEllipsoid().getNorthLatitude().getDegrees() + "  lon " +  tNavigation.getEllipsoid().getEastLongitude().getDegrees() + " hgt " +  tNavigation.getEllipsoid().getEllipsoidHeight() );
					
					System.out.println(efg.toString(15)+tefg.toString(15)+"vector difference magnitude: " + new Vector3(efg).subtract(tefg).getAbs());
					assertTrue(efg.isEquivalent(tefg,1e-7));//to the .01 micrometer!
					
					//get rotator and translation components from transform to navigation object
					q_EFG_NED = tNavigation.getLocal(); 	//geodetic-tangent "level"
                    qHeight = tNavigation.getLocalHeight();		//geodetic-normal "vertical"
                    
					//output navigation transform's rotator norms and rotator
//					System.out.print(String.format("max = %5f; ",q_EFG_NED.getNormInf()));
//					System.out.print(String.format("abs = %5f; ",q_EFG_NED.getAbs()));
//					System.out.print(String.format(" q = " + q_EFG_NED.toString(15))); 
					assertTrue(1>=q_EFG_NED.getNormInf());
					assertTrue(2>=q_EFG_NED.getAbs());
					
                    //set Ellipsoid coordinate object with transform to navigation object
					tgeodetic.set(tNavigation.getEllipsoid());
					qlat = tgeodetic.getNorthLatitude().getDegrees();						
					qlon = tgeodetic.getEastLongitude().getDegrees();												
					System.out.print(String.format(" Qlat = %.10f", qlat ));
					System.out.print(String.format(" Qlon = %.10f", qlon ));
					System.out.print(String.format(" lambda = %.10f", lambda ));
					//System.out.println("HERE!!!!");
					assertEquals(qlat,phi,1e-13);
//					assertEquals(qlon,lambda % 360.0,1e-13);
					
					System.out.println();	
					cnt = cnt+1;
					
				}
			}
		}
		System.out.println("Test -0 < +0: " + (-0d < +0d) + "    Double.compare: -0 < +0: " + (Double.compare(-0d, +0d)<0d));
		System.out.println("Test -0 == +0: "+ (-0d == +0d) );
		
		System.out.println("Success of geo-location structures and earth coordinate transformations. Count = "+cnt);
	}


	/**
	 * Test method for {@link tspi.model.T_EFG_NED#WGS84(rotation.Vector3)}.
	 */
	public final void testWGS84Vector3() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#WGS84(rotation.Rotator, double)}.
	 */
	public final void testWGS84OperatorDouble() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#WGS84(rotation.Rotator, rotation.Vector3)}.
	 */
	public final void testWGS84OperatorVector3() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#getLocationXYZ()}.
	 */
	public final void testGetXYZ() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#getFromNEDtoEFG()}.
	 */
	public final void testGetFromNEDtoEFG() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#getNorthLatitude()}.
	 */
	public final void testGetAngleLatitude() {
		fail("Not yet implemented"); // TODO
	}


	/**
	 * Test method for {@link tspi.model.T_EFG_NED#putXYZ(rotation.Vector3)}.
	 */
	public final void testPutXYZ() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#set(rotation.Rotator, double)}.
	 */
	public final void testPut() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#putLatitude(rotation.Principle)}.
	 */
	public final void testPutLatitudePrinciple() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#putLatitude(rotation.Angle)}.
	 */
	public final void testPutLatitudeAngle() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#putLongitude(rotation.Principle)}.
	 */
	public final void testPutLongitudePrinciple() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#putLongitude(rotation.Angle)}.
	 */
	public final void testPutLongitudeAngle() {
		fail("Not yet implemented"); // TODO
	}

	/**
	 * Test method for {@link tspi.model.T_EFG_NED#putHeight(double)}.
	 */
	public final void testPutHeight() {
		fail("Not yet implemented"); // TODO
	}

}
