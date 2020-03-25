/**
 * 
 */
package tspi.filter;

import tspi.model.Ellipsoid;
import tspi.model.T_EFG_FRD;
import tspi.model.T_EFG_NED;
import tspi.rotation.Angle;

/**
 * @author mshields
 *
 */
public class TestApertureDirections {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		double h = 1000;
		double r = 10000;
		Angle latitude = Angle.inDegrees(10);
		Angle longitude = Angle.inDegrees(-60);
		Angle azimuth = Angle.inDegrees(0);
		Angle elevation = Angle.inDegrees(0);
				
		Ellipsoid pedLoc = new Ellipsoid();
		pedLoc.set(latitude,longitude,h);
		
		T_EFG_NED Q = new T_EFG_NED();
		Q.set(pedLoc);
		
		T_EFG_FRD P = new T_EFG_FRD();
		P.set(azimuth, elevation, Q.getLocal());
		
		System.out.println("deg Lat: " + latitude.toDegreesString(4) + "   deg Lon: " + longitude.toDegreesString(4));
		System.out.println("    Location Quaternion = " + Q.getLocal().unit().toString(13));
		System.out.println("__________________________________________________________________________");
		for (int k = 0; k < 2; k++) {
			
			
			elevation.setDegrees(k * 30d);
			
			for (int j = 0; j < 9; j++) {
				azimuth.setDegrees(j * 45d);
				
				P.set(azimuth, elevation, Q.getLocal());
				System.out.println(
						"\n  deg Az: " + azimuth.toDegreesString(4) + "   deg El: " + elevation.toDegreesString(4));
				System.out.println(P.getOrientation().unit().toString(13));
				System.out.println("    I_unit = " + P.getForwardUnit().unit().toString(13));
				System.out.println("    J_unit = " + P.getRightUnit().unit().toString(13));
				System.out.println("    K_unit = " + P.getDownUnit().unit().toString(13));
			}
		}

	}

}
