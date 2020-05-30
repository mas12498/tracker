package tspi.simulator;


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

    // current state of the observer
    double time;
    Vector3 truth;
    int[] modes;
    Polar[] observations; // TODO should I instead keep an ensemble?

    // state of the file that observations are being read from?
    BufferedReader reader;
    String[] header;
    String[] row;
    File file;
    String separator = ",";
    int offset;
    Exception exception;

    public ObservationsReader(File csv, boolean hasTruth, int sensorCount) throws Exception {
        file = csv;
        reader = new BufferedReader( new FileReader(csv) );

        if (hasTruth)
            offset = 4;
        else offset = 1;

        header = new String[sensorCount + offset + (4*sensorCount)];
    }

    public ObservationsReader(File csv) throws Exception {
        file = csv;
        reader = new BufferedReader( new FileReader(csv) );
        header = reader.readLine().split(separator);

        if (!header[0].equalsIgnoreCase("timeSec"))
            throw new Exception( "first column of observation file should be time and must have the header \'timeSec\'" );

        if (header[1].trim().equalsIgnoreCase("trackE")
                && header[2].trim().equalsIgnoreCase("trackF")
                && header[3].trim().equalsIgnoreCase("trackG") )
            offset = 4;
        else offset = 1;

        if ((header.length - offset) % 4 != 0)
            throw new Exception( "Each sensor should provide 4 values; mode, range, azimuth, elevation" );

        for (int k=offset; k<header.length; k+=4) {
            if (!header[k].contains("_mode"))
                throw new Exception("The first measurement of an observation should be mode and should contain \"mode\" in the title of column "+(k+1));
            if (!header[k+1].contains("_rg"))
                throw new Exception("The first measurement of an observation should be range and should contain \"rg\" in the title of column "+(k+2));
            if (!header[k+2].contains("_az"))
                throw new Exception("The first measurement of an observation should be azimuth and should contain \"az\" in the title of column "+(k+3));
            if (!header[k+3].contains("_el"))
                throw new Exception("The first measurement of an observation should be elevation and should contain \"el\" in the title of column "+(k+4));
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
    public Double next() { return new Double(time); }

    @Override
    public double getTime() { return time; }

    @Override
    public Vector3 getTruth() { return truth; }

    @Override
    public Polar[] getObservations() { return observations; }

    public int[] getModes(int index) { return modes; }

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

        int count = (row.length-offset)/4;
        modes = new int[ count ];
        observations = new Polar[ count ];
        for (int n=0; n<count; n++) {
            modes[n] = Integer.parseInt( row[offset + (4*n)] );
            double range = Double.parseDouble( row[offset + (4*n) + 1] );
            double azimuth = Double.parseDouble( row[offset + (4*n) + 2] );
            double elevation = Double.parseDouble( row[offset + (4*n) + 3] );
            observations[n] = new Polar(range, Angle.inDegrees(azimuth), Angle.inDegrees(elevation));
        }
    }

    public static void main( String[] args ) {
        File file = new File( "./data/TrajectoryTest/racetrack.csv");
        try {
            ObservationsReader playback = new ObservationsReader(file);

            while (playback.hasNext()) {
                playback.next();
                System.out.println( playback.getTime() );
            }
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
}