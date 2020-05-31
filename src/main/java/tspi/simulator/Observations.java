package tspi.simulator;

import tspi.model.Ensemble;
import tspi.model.Polar;
import tspi.rotation.Vector3;

import java.util.Iterator;

/** abstracts the source of observtions. */
public interface Observations extends Iterator<Ensemble> {

    /** @return the current time */
    public double getTime();

    /** @return the actual location of the target in EFG, or null if not available. */
    public Vector3 getTruth();

    /** @return an Ensemble of Pedestals pointed at a target */
    public Ensemble getEnsemble();
    //TODO instead expose an ensemble here
    // Make the iterator produce ensembles?
    // Create Observation class with time, truth, and measurements and have iterator produce that
    // Or drop iterator interface and write methods for explicitly advancing the stream of observations

    // TODO Ensemble
    // add more pointing routines to ensemble
    // add write routing to ensemble
    // add time to ensemble? no, prob not...

    // TODO TestFilter;
    //make testFilter use an ensemble instead of an array of Pedestals and PedestalModel
    //make test filter use Observations interface while driving filter.
    //add error diagnostic output to TestFilter

    // TODO ObservationsSimulator
    // Make observation simulator use Ensemble to point instead of pointing them individually

    //TODO move mode into pedestal and change interface to provide iterator of Pedestals?
}