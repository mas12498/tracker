package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.model.Ellipsoid;
import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Vector3;
import tspi.util.TVector;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.Random;

/** Generates a sequence of state vectors from a Trajectory and writes them to File.
 * Very similar to Test Measurements but has a different Trajectory */
public class ObservationsWriter {

    Trajectory trajectory;
    Ensemble sensors;
    TVector position;
    Random random;

    public ObservationsWriter(long seed) {
        this.random = new Random( seed );
        this.position = null;
    }

    public void setTrajectory(Trajectory trajectory) {this.trajectory = trajectory;}
    public void setSensors(Ensemble sensors) {this.sensors = sensors;}

    public Trajectory getTrajectory() {return trajectory;}
    public Ensemble getSensors() {return sensors;}
    public Vector3 getTarget() {return position;}

    /** Print the required info into a CSV file with the following columns;
     * <pre>
timeSec,trackE,trackF,trackG,A_mode,A_rg,A_az,A_el,B_mode,B_rg,B_az,B_el,C_mode,C_rg,C_az,Cel
 0        1       2       3   4       5   6   7       8   9     10   11   12      13  14  15 </pre>
     * Also produces a target file for checking output with Increment III;
     * <pre>time, NLat, ELon, eHgt</pre>*/
    public void write(double t0, double dt, int n, PrintStream output, PrintStream targets) {

        // write the header of the file
        output.print( "timeSec, trackE, trackF, trackG");
        for (int m=0; m<sensors.size(); m++) {
            String id = sensors.get(m).getSystemId();
            output.print( ", "+id+"_mode, "+id+"_rg, "+id+"_az, "+id+"_el");
        }
        output.println();

        targets.println("time, NLat, ELon, eHgt");

        // sample the time interval
        for (double i=0; i<n; i++) {
            double t = t0 + dt*i;

            // find the current trajectory position
            RealVector position = trajectory.getPosition(t);
            TVector efg = new TVector(position);

            // also write out a target file
            Ellipsoid llh = new Ellipsoid();
            llh.setGeocentric(efg);
            targets.println( t + ", "
                    + llh.getEastLongitude().getDegrees() + ", "
                    + llh.getNorthLatitude().getDegrees() + ", "
                    + llh.getEllipsoidHeight());

            // Print the trajectory info
            output.print( t+","
                    +efg.getX()+","
                    +efg.getY()+","
                    +efg.getZ()+"," );

            // for every sensor
            for (int m=0; m<sensors.size(); m++) {
                // measure the trajectory, including sensor error
                Pedestal pedestal = sensors.get(m);
                pedestal.pointToLocation(efg);
                Polar rae = pedestal.getPerturbedLocal(random);

                // print the sensor's measurment;
                int mode = 0; // should we add mode to the pedestal? the Trajectory?
                output.print( mode+","
                        +rae.getRange()+","
                        +rae.getSignedAzimuth().getDegrees()+","
                        +rae.getElevation().getDegrees()+"," );
            }

            output.println();
        }
    }

    // TODO break apart some functionality for easier comprehension and wider application
    public void update(double time) {
        // find the current trajectory position
        RealVector position = trajectory.getPosition(time);
        this.position = new TVector(position);

        // point each sensor at the target position
        for (int m=0; m<sensors.size(); m++) {
            // measure the trajectory, including sensor error
            Pedestal pedestal = sensors.get(m);
            pedestal.pointToLocation(this.position);
            //Polar rae = pedestal.getPerturbedLocal(random);
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

            // create the trajectory file writer for the given trajectory and ensemble
            ObservationsWriter writer = new ObservationsWriter( 0L ); // System.nanoTime() );
            writer.setTrajectory( racetrack );
            writer.setSensors( ensemble );

            // write the trajectory profile to output
            int n = (int)Math.floor((racetrack.getPerimeter() / velocity) / dt); // one circuit of the racetrack
            File file = new File( args[10] );
            PrintStream output = new PrintStream( new FileOutputStream(file) );
            File targetFile = new File(args[11]);
            PrintStream target = new PrintStream( new FileOutputStream(targetFile));
            writer.write( start, dt, n, output, target );

        } catch(Exception exception) {
            exception.printStackTrace();
        }
    }
}
