package tspi.simulator;

import tspi.model.Ensemble;
import tspi.rotation.Vector3;
import tspi.util.TVector;

import java.util.Random;

class ObservationsSimulator implements Observations {

    // Observation state members, updated after the iterator is advanced
    Ensemble sensors; // the pedestals observing the target
    Vector3 truth; // current true target position in EFG
    double time; // current time

    // Simulation members used to generate observations
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

    @Override
    public Ensemble getEnsemble() { return sensors; }

//    public Polar[] getObservations() {
//        Polar[] observations = new Polar[ sensors.size() ];
//        for (int m = 0; m< sensors.size(); m++)
//            observations[m] = sensors.get(m).getLocal();
//        return observations;
//    }

    @Override
    public boolean hasNext() { return i < n; }

    @Override
    public Ensemble next() {

        // find the current trajectory position
        time = t + ((++i) * dt);
        truth = new TVector( trajectory.getState( time ) );

        // point the ensemble at the target
        sensors.point(truth, random);

        return sensors;
    }
}
