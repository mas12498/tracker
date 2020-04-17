package tspi.simulator;

import tspi.simulator.Kinematic;

public class TestKinematic {

	public static void main(String args[]) {
		Kinematic kinematic = new Kinematic(0.0, //t
				0.0,0.0,0.0, // p
				0.0,0.0,0.0, // v
				1.0,0.0,0.0 ); // a
		for (double t=0; t<10; t++) {
			System.out.println( "p="+kinematic.getPosition(t).toString() +
					"\tv="+kinematic.getVelocity(t).toString() +
					"\ta="+kinematic.getAcceleration(t).toString() +
					"\tS="+kinematic.getState(t) );
		}
	}
	
}
