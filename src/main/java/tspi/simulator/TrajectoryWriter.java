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
public class TrajectoryWriter {

    Trajectory trajectory;
    Ensemble sensors;

    public TrajectoryWriter() {}

    public Trajectory getTrajectory() {return trajectory;}
    public Ensemble getSensors() {return sensors;}

    public void setTrajectory(Trajectory trajectory) {this.trajectory = trajectory;}
    public void setSensors(Ensemble sensors) {this.sensors = sensors;}

    /** Print the required info into a CSV file with the following columns;
     * timeSec,trackE,trackF,trackG,A_mode,A_rg,A_az,A_el,B_mode,B_rg,B_az,B_el,C_mode,C_rg,C_az,Cel
     *  0        1       2       3   4       5   6   7       8   9     10   11   12      13  14  15
     *  Also produces a target file for checking output with Increment III;
     *  time, NLat, ELon, eHgt*/
    public void write(double t0, double dt, int n, PrintStream output, PrintStream targets, Random random) {

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

    public static void main(String[] args) {
        try {
            // load an ensemble of sensors
            File pedestalFile = new File("./data/pedestalsTest100.csv");
            Ensemble ensemble = Ensemble.load(pedestalFile);

            // set the origin to the first sensor
            Pedestal pedestal = ensemble.get(0);
            Ellipsoid origin = pedestal.getLocationEllipsoid();

            // create a racetrack above that origin
            double start = 0.0;
            double length = 2000;
            double height = 10000;
            double radius = 1000.0;
            double velocity = 223;
            Vector3 c1 = new Vector3(0,0,height);
            Vector3 c2 = new Vector3(0,length,height);
            Racetrack racetrack = new Racetrack( start, origin, c1, c2, radius, velocity);

            // create the trajectory file writer
            TrajectoryWriter writer = new TrajectoryWriter();
            writer.setTrajectory( racetrack );
            writer.setSensors( ensemble );

            // write the trajectory profile to output
            double dt = 0.02; // 50 hertz
            int n = (int)Math.floor((racetrack.getPerimeter() / velocity) / dt); // one circuit
            Random random = new Random( 0L ); // System.nanoTime() );
            File file = new File("./data/TrajectoryTest/racetrack.csv");
            PrintStream output = new PrintStream( new FileOutputStream(file) ); // System.out;
            File targetFile = new File("./data/TrajectoryTest/target.csv");
            PrintStream target = new PrintStream( new FileOutputStream(targetFile));
            writer.write( start, dt, n, output, target, random);

        } catch(Exception exception) {
            exception.printStackTrace();
        }
    }

//    int steps;
//    double start, stop;
//    void setStart(double start) {this.start = start;}
//    void setStop(double stop) {this.stop = stop;}
//    void setSteps(int steps) {this.steps = steps;}
//    double getStart() {return start;}
//    double getStop() {return stop;}
//    int getSteps() {return steps;}
}
