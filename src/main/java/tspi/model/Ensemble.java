package tspi.model;

import tspi.rotation.Vector3;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

/** A set of sensor pedestals working in concert. */
public class Ensemble extends ArrayList<Pedestal>{

	// TODO I think the origin should be a member of Ensemble, as well as origin relative operations...
	// TODO remove all non-UI stuff from PedestalModel to this class
	
	public Pedestal get(String systemId) {
		for(Pedestal pedestal : this)
			if(pedestal._systemId.equals(systemId))
				return pedestal;
		return null;
	}
	
	public Pedestal[] toArray() {
		Pedestal pedestal[] = new Pedestal[this.size()];
		return super.toArray(pedestal);
		//TODO I need to change the dependencies on Pedestal[] to Ensemble...
	}
	
	/** Point each of the sensors in the ensemble at the given point, including a perturbation generated from 
	 * the sensor's error model. */
	public void point(Vector3 efg, Random random) {
		for (Pedestal pedestal : this) {
			pedestal.pointToLocation(efg);
			Polar perturbed = pedestal.getPerturbedLocal(random);
			pedestal.pointDirection(perturbed.getUnsignedAzimuth(), perturbed.getElevation());
		}
	}
	
	/** Read an array of modeled pedestals from the given file */
	public static Ensemble load(File file) throws Exception {
		
		// use the pedestal model to parse the file
		PedestalModel model;
		model = new PedestalModel();
		model.load( file );
		ArrayList<Pedestal> list = model.asList();
		// TODO remove load() from the PedestalModel entirely and move it here!!
		
		//Assume first pedestal in file is filter origin
		Pedestal.setOrigin( list.get(0).getLocation() );
		System.out.println( "ORIGIN:" + Pedestal.getOrigin().toString(3));
		// TODO The origin should be a member of Ensemble instead of a static member of Pedestal!
				
		// convert the model to an ensemble
		Ensemble ensemble = new Ensemble();
		for (int n=1; n<list.size(); n++) {
			Pedestal pedestal = list.get(n);
			
			//Compute local coordinates wrt filter origin defined
			pedestal.setLocalOriginCoordinates();
			
			ensemble.add( pedestal );
		}
					
		//return list with pedestals located and filter origin defined:
		return ensemble;
	}
	
	// TODO save
	// TODO stream
	
	private static final long serialVersionUID = 1L;
}
