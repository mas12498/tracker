package tspi.simulator;


import tspi.model.Polar;
import tspi.rotation.Vector3;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

/** Provides Observations parsed from a csv observations file with the following header and columns;
 * time, [ trackE, trackF, trackG, ] (n_mode, n_rg, n_az, n_el)+
 * */
public class RecordedObservations implements  Observations {

    BufferedReader reader;
    String[] header;
    String[] row;
    File file;
    String separator = ",";
    int offset;

    public RecordedObservations( File csv ) throws Exception {
        file = csv;
        reader = new BufferedReader( new FileReader(csv) );
        header = reader.readLine().split(separator);

        if (!header[0].equalsIgnoreCase("timeSec"))
            throw new Exception( "first column of observation file should be time and must have the header \'timeSec\'" );

        if (header[1].equalsIgnoreCase("trackE")
                && header[2].equalsIgnoreCase("trackF")
                && header[3].equalsIgnoreCase("trackG") )
            offset = 4;
        else offset = 1;

        if ((header.length - offset) % 4 != 0)
            throw new Exception( "Each sensor should provide 4 values; mode, range, azimuth, elevation" );

        for (int k=offset; k<header.length; k+=4) {
            if (!header[k].contains("_mode"))
                throw new Exception("The first measurement of an observation should be mode and should contain \"mode\" in the title of column "+k);
            if (!header[k].contains("_rg"))
                throw new Exception("The first measurement of an observation should be range and should contain \"rg\" in the title of column "+k);
            if (!header[k].contains("_az"))
                throw new Exception("The first measurement of an observation should be azimuth and should contain \"az\" in the title of column "+k);
            if (!header[k].contains("_el"))
                throw new Exception("The first measurement of an observation should be elevation and should contain \"el\" in the title of column "+k);
        }

        // read the data of the first row
        String line = reader.readLine();
        if (line!=null)
            row = line.split(separator);
    }

    @Override
    public double getTime() {

        return 0;
    }

    @Override
    public Vector3 getTruth() {
        return null;
    }

    @Override
    public Polar[] getObservations() {

    }

//    public int getMode(int index) {
//        return 0;
//    }

    @Override
    public boolean hasNext() {
        return row.length > 0;
    }

    @Override
    public Polar[] next() {
        double time
    }
}