package tspi.controller;

import tspi.model.*;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;

import java.io.*;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

// example test ensemble arguments;

//
//  RUN FIRST: GENERATE outputs ensembleFile
//
/* Test Arguments Generate Measurement Ensemble for ground pedestals */
// -generate
// pedestalFile.csv
// targetFile.csv
// ensembleFile.csv

//
//  SECOND: SOLVE single point plots solutionFile
//
/* Test Arguments Obtain Filtered Solution for Measurement Ensemble */
// -solve
// pedestalFile.csv
// ensembleFile.csv
// solutionFile.csv
// 4
// true

public class EnsembleTest {

	protected static SimpleDateFormat dayFormat = new SimpleDateFormat("yyyy.DDD");
	protected static SimpleDateFormat timeFormat = new SimpleDateFormat("HHmmss.SSS");
	protected static NumberFormat numberFormat = new DecimalFormat("0.00000000");
	protected static NumberFormat fTimeFormat = new DecimalFormat("00000000.000");
	protected static NumberFormat longitudeFormat = new DecimalFormat("0.000000000000");
	protected static NumberFormat latitudeFormat = new DecimalFormat("0.000000000000");
//	protected static NumberFormat longitudeFormat = new DecimalFormat("0.000000000000e;-0.000000000000w");
//	protected static NumberFormat latitudeFormat = new DecimalFormat("0.000000000000n;-0.000000000000s");
	protected static String Separator = ",";
	protected static String Line = "\r\n";
	
	public static void main(String[] args) {
		try {
			if(args.length>0 && args[0].equalsIgnoreCase("-generate")) {
				File pedestalFile = new File(args[1]);//"C:\\Users\\Casey\\Documents\\pedestals.csv");
				File targetFile = new File(args[2]);//"C:\\Users\\Casey\\Documents\\targets.csv");
				File ensembleFile = new File(args[3]);//"C:\\Users\\Casey\\Documents\\ensemble.csv");

				int trials = 1;
				
				PrintStream ensembleStream = //System.out;
						new PrintStream(
						new BufferedOutputStream(
						new FileOutputStream( ensembleFile )));
				
				EnsembleTest.generate(
						Calendar.getInstance().getTime(),
						100,
						pedestalFile,
						targetFile,
						trials,
						ensembleStream,
						true,
						0.0, 0.0,
						0.0, 0.0);////1.0, 1.0);
				
			} else if(args.length>0 && args[0].equalsIgnoreCase("-solve")) {
				File pedestalFile = new File(args[1]);//"C:\\Users\\Casey\\Documents\\pedestals.csv");
				File ensembleFile = new File(args[2]);//"C:\\Users\\Casey\\Documents\\ensemble.csv");
				File solutionFile = new File(args[3]);//"C:\\Users\\Casey\\Documents\\solution.csv");
				int solutionCount = Integer.parseInt(args[4]); //number of pedestals in soln
				boolean header = (args[5].equalsIgnoreCase("true")) ? true : false; //file has column names for header.
				
				PrintStream solutionStream = //System.out;
						new PrintStream(
						new BufferedOutputStream(
						new FileOutputStream( solutionFile )));
				
				EnsembleTest.solve(
						ensembleFile,
						pedestalFile,
						solutionCount, //4,
						solutionStream,
						header);//true);
			} else {
				System.out.println(
						"Usage:\n"
						+ "EnsembleTest -generate <input pedestal file-spec> <input targets file-spec> <output ensemble file-spec>\n"
						+ "or\n"
						+ "EnsembleTest -solve <input pedestal file-spec> <input ensemble file-spec> <output solution files-spec> <pedestal count solution> <header present={true|else}>\n"
								);
			}
			
		} catch(Exception exception) {
			exception.printStackTrace();
		}
	}

	public static void generate(
			Date time,
			int dt,
			File pedestalFile, // ensemble file's pedestal order will correspond to this pedestal file
			File targetFile,
			int trialsPerTarget,
			PrintStream stream,
			boolean header,
			double azBias, double elBias, //TODO add these values to Pedestal, and load them per pedestal!
			double azRange, double elRange )
			throws Exception
	{
		PedestalModel pedestals = new PedestalModel();
		pedestals.load( pedestalFile );
		TargetModel targets = new TargetModel();
		targets.load( targetFile );
		EnsembleTest.generate(
				time, dt, pedestals, targets, trialsPerTarget, stream, header, azBias, elBias, azRange, elRange );
	}
	
	public static void generate(
			Date time, // start time
			int dt, // trial time step in milliseconds
			PedestalModel pedestals,
			TargetModel targets,
			int trialsPerTarget,
			PrintStream stream,
			boolean header,
			double azBias, double elBias, //TODO add these values to Pedestal, and load them per pedestal!
			double azRange, double elRange)
	{
		// print header
		if( header ) {
			stream.append( "Day" ); stream.append( Separator );
			stream.append( "Time" ); stream.append( Separator );
			stream.append( "FTime" ); stream.append( Separator );
			for(Pedestal pedestal : pedestals) {
				stream.append( pedestal.getSystemId()+"_az" ); stream.append( Separator );
				stream.append( pedestal.getSystemId()+"_el" ); stream.append( Separator );
			}
			//TODO truths?
			stream.append(Line);
		}
		
		for( Target target : targets ) {
			for( int trial=0; trial<trialsPerTarget; trial++ ) {
			
				// increase time every trial?
				time.setTime( time.getTime() + dt);
				
				// write time columns
				stream.append( dayFormat.format(time) );
				stream.append( Separator );
				stream.append( timeFormat.format(time) );
				stream.append( Separator );
				stream.append( fTimeFormat.format( ftime(time) ) );
				stream.append( Separator );
				
				// TODO just generate pedestal mesurements in the same order they appear in the pedestal file.
//				// TODO we could set these using column headers instead of repeating them each record...
//				for( Pedestal pedestal : pedestals ) {
//					stream.append( pedestal.getSystemId() );		
//					stream.append( Separator );
//				} // or they could be specified by a pedestal file...
				
				for( Pedestal pedestal : pedestals ) {
					
					// point pedestal at target then perturb
					pedestal.pointToLocation( target.getGeocentricCoordinates() ); // TODO might want to cache instead of recomputing every trial...
					Angle azimuth = pedestal.getLocal().getUnsignedAzimuth();
					Angle elevation = pedestal.getLocal().getElevation();
					azimuth.add( Angle.inDegrees( azBias + (Math.random()*azRange) ) );
					elevation.add( Angle.inDegrees( elBias + (Math.random()*elRange) ) );
					pedestal.pointDirection(azimuth, elevation);
					// TODO get error model from pedestal instead of args!
					// TODO support gaussian error instead of just linear...
					// TODO polar or rectangular coords?
					
//					stream.append( numberFormat.format( pedestal.getRange() ) );		
//					stream.append( Separator );
					stream.append( numberFormat.format( pedestal.getLocal().getUnsignedAzimuth().getDegrees() ) );		
					stream.append( Separator );
					stream.append( numberFormat.format( pedestal.getLocal().getElevation().getDegrees() ) );		
					stream.append( Separator );
				}
				
				//TODO Eh? Use truth for which pedestal?
				
				stream.append( Line );
			}
		}
		stream.close();
	}
	
	public static void solve( 
			File ensembleFile,
			File pedestalFile,
			int pedestalCount,
			PrintStream stream,
			boolean header)
			throws Exception
	{
		PedestalModel pedestals = new PedestalModel();
		pedestals.load( pedestalFile );
		EnsembleTest.solve(
				ensembleFile,
				pedestals,
				pedestalCount,
				stream,
				header);
	}
	public static void solve( 
			File ensembleFile,
			PedestalModel pedestals, // pedestal order corresponds to pedestal order in ensemble file...
			int pedestalCount, // the first n columns are used in the solution
			PrintStream stream,
			boolean header)
			throws Exception
	{
		BufferedReader reader = new BufferedReader( new FileReader(ensembleFile) );
		String line = "";
		
		// consume ensemble header and output solution header
		if(header) {
			line = reader.readLine();
			stream.append( "Day" ); stream.append( Separator );
			stream.append( "Time" ); stream.append( Separator );
			stream.append( "FTime" ); stream.append( Separator );
			stream.append( "E" ); stream.append( Separator );
			stream.append( "F" ); stream.append( Separator );
			stream.append( "G" ); stream.append( Separator );
			stream.append( "long" ); stream.append( Separator );
			stream.append( "lat" ); stream.append( Separator );
			stream.append( "h" ); stream.append( Separator );
			stream.append( "condition" ); stream.append( Separator );
			for(Pedestal pedestal : pedestals) {
				stream.append( pedestal.getSystemId()+"_az" ); stream.append( Separator );
				stream.append( pedestal.getSystemId()+"_el" ); stream.append( Separator );
				stream.append( pedestal.getSystemId()+"_ra" ); stream.append( Separator );
			}
			stream.append(Line);
		}
		
		int count = 1;
		try {
			while( (line=reader.readLine()) != null) {
				String cols[] = line.split( Separator );
				if(cols.length==0 || (cols.length==1&&cols[0].equals("")))
					continue;
				
				// write the time to output
				String day = cols[0].trim();
				String time = cols[1].trim();
				Double ftime = Double.parseDouble(cols[2].trim());
				// TODO compute a time representation which is useful...
				stream.append( day );
				stream.append(Separator);
				stream.append( time );
				stream.append(Separator);
				stream.append( fTimeFormat.format( ftime ) );
				stream.append(Separator);
				
				// just get this info from the pedestal file and assume the pedestal order is the same!!!
//				// parse system names
//				if( pedestals.getColumnCount()==0 )
//					for(int index=0; index<pedestalCount; index++) {
				// TODO how do we get location? we have to load a pedestal file anyways!
//					Pedestal pedestal = new Pedestal(cols[3+index], Angle.inDegrees(0.0), Angle.inDegrees(0.0), 0.0);
//					pedestals.add(index, pedestal);
//				}
				
				// point the pedestals using the ensemble measurements
				for( int n=0; n<pedestals.getRowCount(); n++) {
					Pedestal pedestal = pedestals.getPedestal(n);
					Angle azimuth = Angle.inDegrees( Double.parseDouble( cols[3+n*2].trim() ) );
					Angle elevation = Angle.inDegrees( Double.parseDouble( cols[4+n*2].trim() ) );
//					double range = Double.parseDouble( cols[3+n].trim() ); // TODO number of sensor will have to come from the pedestal file as well...
					pedestal.pointDirection(/*range,*/ azimuth, elevation);
				}
				
				// solve the system
				Vector3 origin = pedestals.getPedestal(0).getLocation(); //default first pedestal as origin...
				Pedestal.setOrigin(origin);
				ArrayList<Pedestal> list = new ArrayList<Pedestal>();
				for( int n=0; n<pedestalCount; n++  ) {
					pedestals.getPedestal(n).setLocalOriginCoordinates();
			    	list.add(pedestals.getPedestal(n));					
				}
				Solution solution = new Solution( list );
				Ellipsoid coordinate = new Ellipsoid();
				coordinate.setGeocentric( solution._position_EFG );
				double condition = solution._condition;
				
				// write the solution coordinates
				stream.append( numberFormat.format( solution._position_EFG.getX() ) ); // todo might want to make formats for longitude and latitude
				stream.append( Separator );
				stream.append( numberFormat.format( solution._position_EFG.getY() ) );
				stream.append( Separator );
				stream.append( numberFormat.format( solution._position_EFG.getZ() ) );
				stream.append( Separator );
				stream.append( longitudeFormat.format( coordinate.getEastLongitude().getDegrees() ) ); // todo might want to make formats for longitude and latitude
				stream.append( Separator );
				stream.append( latitudeFormat.format( coordinate.getNorthLatitude().getDegrees() ) );
				stream.append( Separator );
				stream.append( numberFormat.format( coordinate.getEllipsoidHeight() ) );
				stream.append( Separator );
				stream.append( numberFormat.format( condition ) );
				stream.append( Separator );
				
				// compute the pedestal az and el given the solution
				for(Pedestal pedestal : pedestals) {
					pedestal.pointToLocation( solution._position_EFG );
					double azimuth = pedestal.getLocal().getUnsignedAzimuth().getDegrees();
					double elevation = pedestal.getLocal().getElevation().getDegrees();
					double range = pedestal.getLocal().getRange();
					
					// add solution direction to output
					stream.append( numberFormat.format( azimuth ) );
					stream.append( Separator );
					stream.append( numberFormat.format( elevation ) );
					stream.append( Separator );
					stream.append( numberFormat.format( range ) );
					stream.append( Separator );
				}
				
				stream.append( Line );
				
				count++;
			}
		} catch(Exception exception) {
			exception.printStackTrace();
			throw new Exception( "Error, line "+count+"("+line+") : "+exception.getMessage() );
		} finally {
			reader.close();
			stream.close();
		}
	}
	
	public static double ftime( Date time ) {
		Calendar c = Calendar.getInstance();
		c.setTime( time );
		double millis = 0.0;
		millis += c.get(Calendar.HOUR_OF_DAY)*3600000;
		millis += c.get(Calendar.MINUTE)*60000;
		millis += c.get(Calendar.SECOND)*1000;
		millis += c.get(Calendar.MILLISECOND);
		return millis/1000.0;
	}
}
