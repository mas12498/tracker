package tspi.simulator;

import tspi.model.Ellipsoid;
import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Vector3;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;

/** Generates a sequence of state vectors from a Trajectory and writes them to File.
 * Very similar to Test Measurements but has a different Trajectory */
public class ObservationsWriter {

    Observations observations;

    public ObservationsWriter(Observations observations) {
        this.observations = observations;
    }

    public void setObservations(Observations observations) {this.observations = observations;}
    public Observations getObservations() {return observations;}

    /** Print the required info into a CSV file with the following columns;
     * <pre>
timeSec,trackE,trackF,trackG,A_mode,A_rg,A_az,A_el,B_mode,B_rg,B_az,B_el,C_mode,C_rg,C_az,Cel
 0        1       2       3   4       5   6   7       8   9     10   11   12      13  14  15 </pre>
     * Also produces a target file for checking output with Increment III;
     * <pre>time, NLat, ELon, eHgt</pre>*/
    public void write(PrintStream targetObservations, PrintStream targetTruth) {

        // TODO do we really need truth with observations if we're writing a truth file? Or should we put LLH with observations?

        // TODO write the header of the files. Do I need an ensemble to get sensor IDs?
//        targetObservations.print( "timeSec" );
//        if (observations.hasTruth())
//            targetObservations.print(", trackE, trackF, trackG");
//        for (int m=0; m<observations.size(); m++) {
//            String id = "S"+m; // sensors.get(m).getSystemId();
//            targetObservations.print( ", "+id+"_mode, "+id+"_rg, "+id+"_az, "+id+"_el");
//        }
//        targetObservations.println();

        targetTruth.println("time, NLat, ELon, eHgt");

        while (observations.hasNext()) {
            observations.next();

            double time = observations.getTime();
            Vector3 truth = observations.getTruth();
            Polar[] measurements = observations.getObservations();
            // TODO figure out where to keep mode...
//            int[] modes = observations.getModes();

            // write a record in the observations file
            targetObservations.print(time);
            if (truth!=null)
                targetObservations.print(
                        ","+truth.getX()
                        +','+truth.getY()
                        +','+truth.getZ() );
            for (int n=0; n<measurements.length; n++)
                targetObservations.print(
                        ",0,"+measurements[n].getRange()
                        +","+measurements[n].getSignedAzimuth()
                        +","+measurements[n].getElevation() );
            targetObservations.println();

            // write a record in the target truth file
            Ellipsoid llh = new Ellipsoid();
            llh.setGeocentric(truth);
            targetTruth.println( time + ", "
                    + llh.getEastLongitude().getDegrees() + ", "
                    + llh.getNorthLatitude().getDegrees() + ", "
                    + llh.getEllipsoidHeight());
        }
    }

    /** Moves a target around a specified racetrack once while pointing an ensemble of sensors at it. Generates a CSV
     * describing the true target path, and a CSV describing the simulated sensor measurements, including error.
     * syntax;
     * <pre>$ racetrackWriter <C1(m)> <C2(m)> <radius(m)> <velocity(m/s)> <dt(s)> <pedestalinput> <ensembleoutput> <truthoutput></pre>
     * example:
     * <pre>0 0 10000 0 2000 10000 1000 223 0.02 "./data/pedestalsTest100.csv" "./data/TrajectoryTest/racetrack.csv" "./data/TrajectoryTest/target.csv"</pre>
     *  */
    public static void main(String[] args) {
        try {
            // parse arguments
            Vector3 c1 = new Vector3(Vector3.ZERO);
            c1.setX( Double.parseDouble(args[0]) );
            c1.setY( Double.parseDouble(args[1]) );
            c1.setZ( Double.parseDouble(args[2]) );
            Vector3 c2 = new Vector3(Vector3.ZERO);
            c2.setX( Double.parseDouble(args[3]) );
            c2.setY( Double.parseDouble(args[4]) );
            c2.setZ( Double.parseDouble(args[5]) );
            double radius = Double.parseDouble(args[6]);
            double velocity = Double.parseDouble(args[7]);
            double dt = Double.parseDouble(args[8]);
            double start = 0.0; // should this be an argument?

            // load an ensemble of sensors
            File pedestalFile = new File(args[9]);//"./data/pedestalsTest100.csv");
            Ensemble ensemble = Ensemble.load(pedestalFile);

            // set the origin to the first sensor
            Pedestal pedestal = ensemble.get(0);
            Ellipsoid origin = pedestal.getLocationEllipsoid();

            // create a racetrack above that origin
            Racetrack racetrack = new Racetrack( start, origin, c1, c2, radius, velocity);

            // create a simulation of the ensemble observing the trajectory
            int n = (int)Math.floor((racetrack.getPerimeter() / velocity) / dt); // one circuit of the racetrack
            ObservationsSimulator simulation = new ObservationsSimulator(ensemble, racetrack, start, dt, n);

            // write the trajectory profile to output
            ObservationsWriter writer = new ObservationsWriter( simulation );
            File file = new File( args[10] );
            PrintStream output = new PrintStream( new FileOutputStream(file) );
            File targetFile = new File(args[11]);
            PrintStream target = new PrintStream( new FileOutputStream(targetFile));
            writer.write( output, target );

        } catch(Exception exception) {
            exception.printStackTrace();
        }
    }
}
