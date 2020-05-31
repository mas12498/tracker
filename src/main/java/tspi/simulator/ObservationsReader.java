package tspi.simulator;


import tspi.model.Ensemble;
import tspi.model.Pedestal;
import tspi.model.Polar;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

/** Provides Observations parsed from a csv observations file with the following header and columns;
 * time, [ trackE, trackF, trackG, ] (n_mode, n_rg, n_az, n_el)+
 * */
public class ObservationsReader implements Observations {

    // current state of the observers
    double time;
    Vector3 truth;
    Ensemble ensemble;

    // state of the file that observations are being read from?
    BufferedReader reader;
    String[] header;
    String[] row;
    File file;
    String separator = ",";
    int offset = 4;
    Exception exception;

    public ObservationsReader(File pedestals, File observations) throws Exception {
        // open the files
        ensemble = Ensemble.load( pedestals );
        reader = new BufferedReader( new FileReader(observations) );

        // read the header
        header = reader.readLine().split(separator);

        // make sure it has a time column
        if (!header[0].equalsIgnoreCase("timeSec"))
            throw new Exception( "first column of observation file should be time and must have the header \'timeSec\'" );

        // make sure it has columns for the target's true EFG coordinates
        if (header[1].trim().equalsIgnoreCase("trackE")
                && header[2].trim().equalsIgnoreCase("trackF")
                && header[3].trim().equalsIgnoreCase("trackG") )
            offset = 4;
        else offset = 1;

        // make sure there are the same number of pedestals in the ensemble and observation file
        if (header.length/4.0 != ensemble.size()+1.0)
            throw new Exception("There must be the same number of ensemble Pedestals and observations");

        // every pedestal in the ensemble should have 3 sensor measurements and a mode bitmask
        for (int k=offset; k<header.length; k+=4) {
            if (!header[k].contains("_mode"))
                throw new Exception("The first measurement of an observation should be mode and should contain \"mode\" in the title of column "+(k+1));
            if (!header[k+1].contains("_rg"))
                throw new Exception("The first measurement of an observation should be range and should contain \"rg\" in the title of column "+(k+2));
            if (!header[k+2].contains("_az"))
                throw new Exception("The first measurement of an observation should be azimuth and should contain \"az\" in the title of column "+(k+3));
            if (!header[k+3].contains("_el"))
                throw new Exception("The first measurement of an observation should be elevation and should contain \"el\" in the title of column "+(k+4));
            // TODO make sure header matches ensemble information
            // can they be in different orders?
        }
    }

    @Override
    public boolean hasNext() {
        try {
            String line = reader.readLine();
            if (line == null)
                return false;

            parse( line );
        } catch(Exception e) {
            e.printStackTrace();
            exception = e;
            return false;
        }
        return true;
    }

    @Override
    public Ensemble next() { return ensemble; }

    @Override
    public double getTime() { return time; }

    @Override
    public Vector3 getTruth() { return truth; }

    @Override
    public Ensemble getEnsemble() { return ensemble; }

    void parse(String line) throws Exception {
        row = line.split( separator );

        time = Double.parseDouble( row[0] );

        if (offset==4) {
            double e = Double.parseDouble( row[1] );
            double f = Double.parseDouble( row[2] );
            double g = Double.parseDouble( row[3] );
            truth = new Vector3( e, f, g );
        } else
            truth = null;

        for (int n=0; n<ensemble.size(); n++) {
            // read sensor measurements
            int mode = Integer.parseInt( row[offset + (4*n)] );
            double range = Double.parseDouble( row[offset + (4*n) + 1] );
            double azimuth = Double.parseDouble( row[offset + (4*n) + 2] );
            double elevation = Double.parseDouble( row[offset + (4*n) + 3] );

            // update the corresponding pedestal
            Pedestal pedestal = ensemble.get(n);
            pedestal.point(range, Angle.inDegrees(azimuth), Angle.inDegrees(elevation));
            pedestal.setMode(mode);
        }
    }

    public static void main( String[] args ) {
        File observations = new File( "./data/TrajectoryTest/racetrack.csv");
        File ensemble = new File("./data/pedestalsTest100.csv");
        try {
            ObservationsReader playback = new ObservationsReader(ensemble, observations);

            while (playback.hasNext()) {
                playback.next();
                System.out.println( playback.getTime() );
            }
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
}