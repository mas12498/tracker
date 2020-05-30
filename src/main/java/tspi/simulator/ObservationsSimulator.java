package tspi.simulator;

import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Vector3;
import tspi.util.TVector;

import java.util.Random;

class ObservationsSimulator implements Observations {

    // Observation state members, updated after the iterator is advanced
    Polar[] observations; // current observations
    Vector3 truth; // current true target position in EFG
    double time; // current time

    // Simulation members used to generate observations
    Ensemble sensors; // the pedestals generating observations
    Trajectory trajectory; // the parametric definition of the simulated target
    double t, dt; // start time and time increment
    int i, n; // progress count
    Random random;

    public ObservationsSimulator(
            Ensemble sensors, Trajectory trajectory,
            double start, double step, int count )
    {
        this.trajectory = trajectory;
        this.sensors = sensors;
        this.t = start;
        this.dt = step;
        this.n = count;
        this.i = -1;
        this.random = new Random(0L);
    }

    @Override
    public double getTime() { return time; }

    @Override
    public Vector3 getTruth() { return truth; }

    public Ensemble getSensors() { return sensors; }

    public Polar[] getObservations() { return observations; }
    @Override
    public boolean hasNext() { return i < n; }

    @Override
    public Double next() {

        // find the current trajectory position
        time = t + ((++i) * dt);
        truth = new TVector( trajectory.getState( time ) );

        // point the ensemble at the target
        sensors.point(truth, random);

        // gather an observation from each sensor in the ensemble
        observations = new Polar[ sensors.size() ];
        for (int m = 0; m< sensors.size(); m++)
            observations[m] = sensors.get(m).getLocal();

        return new Double( time );
    }
}
