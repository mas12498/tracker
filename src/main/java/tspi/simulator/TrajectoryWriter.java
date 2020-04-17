package tspi.simulator;

import org.apache.commons.math3.linear.RealVector;
import tspi.filter.Trajectory;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.util.TVector;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Random;

/** Generates a sequence of state vectors from a Trajectory and writes them to File */
public class TrajectoryWriter {

    Trajectory trajectory;
    ArrayList<Pedestal> sensors;

    public TrajectoryWriter(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.sensors = new ArrayList<Pedestal>();
    }

    public TrajectoryWriter addSensor(Pedestal pedestal) {
        this.sensors.add( pedestal );
        return this;
    }

    /** Print the required info into a CSV file with the following columns;
     * timeSec,trackE,trackF,trackG,A_mode,A_rg,A_az,A_el,B_mode,B_rg,B_az,B_el,C_mode,C_rg,C_az,Cel
     *  0        1       2       3   4       5   6   7       8   9     10   11   12      13  14  15 */
    public void write(double t0, double dt, int n, PrintStream output, Random random) {
        // sample the time interval
        for (double i=0; i<n; i++) {
            double t = t0 + n*dt;

            // find the current trajectory position
            RealVector position = trajectory.getPosition(t);
            TVector efg = new TVector(position);

            // Print the trajectory info
            output.print( t+','
                    +efg.getX()+','
                    +efg.getY()+','
                    +efg.getZ()+',' );

            // for every sensor
            for (int m=0; m<sensors.size(); m++) {
                // measure the trajectory, including sensor error
                Pedestal pedestal = sensors.get(m);
                pedestal.pointToLocation(efg);
                Polar rae = pedestal.getPerturbedLocal(random);

                // print the sensor's measurment;
                int mode = 0;// TODO : what is mode?
                output.print( mode+','
                        +rae.getRange()+','
                        +rae.getSignedAzimuth().getDegrees()+','
                        +rae.getElevation().getDegrees()+',' );
            }

            output.println();
        }
    }

    public static void main(String[] args) {
        Random random = new Random( System.nanoTime() );

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
