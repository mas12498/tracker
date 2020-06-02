package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ensemble;
import tspi.rotation.Vector3;

import java.util.Iterator;

/** abstracts the source of observtions. */
public interface Observations extends Iterator<Ensemble> {

    /** @return the current time */
    public double getTime();

    /** @return the actual location of the target in EFG, or null if not available. */
    public Vector3 getTruth();

//    /** @return the actual kinematic state of the target or null if not available. Needed when testing filter accuracy. */
//    public RealVector getTruthState();

    /** @return an Ensemble of Pedestals pointed at a target */
    public Ensemble getEnsemble();

}