package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ensemble;
import tspi.model.Polar;
import tspi.rotation.Vector3;

import javax.swing.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Iterator;

/** abstracts the source of observtions. */
public interface Observations extends Iterator<Double> {

    /** @return the current time */
    public double getTime();

    /** @return the actual location of the target, or null if not available. */
    public Vector3 getTruth();

    /** @return a vector of observations, whose order corresponds to the ensemble order */
    public Polar[] getObservations();

    //TODO how much do we need to expose?
//    /** @return the observation of the specified sensor */
//    public Polar getObservation(int index);
//
//    /** @return the mode */
//    public int getMode(int index);
}

//TODO move mode into pedestal and change interface to provide iterator of sensors?