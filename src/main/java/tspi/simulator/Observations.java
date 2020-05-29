package tspi.simulator;

import tspi.model.Polar;
import tspi.rotation.Vector3;

import java.util.Iterator;

/** abstracts the source of observtions. */
public interface Observations extends Iterator<Double> {

    /** @return the current time */
    public double getTime();

    /** @return the actual location of the target, or null if not available. */
    public Vector3 getTruth();

    /** @return a vector of observations, whose order corresponds to the ensemble order */
    public Polar[] getObservations();

    // TODO should observations produce an ensemble?

    //TODO move mode into pedestal and change interface to provide iterator of Pedestals?
}