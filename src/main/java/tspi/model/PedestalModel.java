package tspi.model;

import tspi.rotation.Angle;

import javax.swing.*;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.TableModel;
import java.awt.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Iterator;

/** A model which represents a list of Pedestals. Also provides a file load and save, as well as a CellRenderer appropriate for the model. */
@SuppressWarnings("serial")
public class PedestalModel extends AbstractTableModel implements Iterable<Pedestal> {
    boolean isEllipsoid;
	protected ArrayList<Pedestal> pedestals;
	public static final int ID=0, MAPRG=1, MAPAZ=2, MAPEL=3, LAT=4, LON=5, H=6, DRG=7, DAZ=8, DEL=9, SDRG=10, SDAZ=11, SDEL=12, R= 13, AZ=14, EL=15, COORD=16;
	public static final int GEOCENTRIC=1, ELLIPSOIDAL=2;
	protected int system = ELLIPSOIDAL;

	
	public PedestalModel() {
		this.pedestals = new ArrayList<Pedestal>();
	}
	
	public ArrayList<Pedestal> asList() { return pedestals; }
	
	public Pedestal getPedestal(int row) {
		return pedestals.get(row);
	}
	
	public Pedestal getPedestal(String systemId) {
		for(Pedestal pedestal : this.pedestals)
			if(pedestal._systemId.equals(systemId))
				return pedestal;
		return null;
	} // might want to index if this is a common operation...
	
	public void add(int index, Pedestal pedestal) {
		this.pedestals.add(index, pedestal);
		this.fireTableRowsInserted(index, index);
	}
	
	public void remove(int index) {
		this.pedestals.remove(index);
		this.fireTableRowsDeleted(index, index);
	}
	
	public void clearOrientations() {
		for(Pedestal pedestal : pedestals)
			pedestal._local.clear(Double.NaN);
	}
	
	public void setCooordinateSystem(int system) {
		if(system == ELLIPSOIDAL || system == GEOCENTRIC) {
			this.system = system;
			this.fireTableStructureChanged();
		}
	}
	
	@Override
	public Iterator<Pedestal> iterator() { return this.pedestals.iterator(); }
	
	@Override
	public int getColumnCount() { return 16; }

	@Override
	public int getRowCount() { return pedestals.size(); }

	@Override
	public String getColumnName(int col) {
		if( this.system == ELLIPSOIDAL ) {
			switch(col) {
			case ID: return "System";
			case MAPRG: return "RG?";
			case MAPAZ: return "AZ?";
			case MAPEL: return "EL?";
			case LAT: return "N Latitude";
			case LON: return "E Longitude";
			case H: return "Height";
			case DRG: return "Bias R*";			
			case DAZ: return "Bias AZ";
			case DEL: return "Bias EL";
			case SDRG: return "Dev R*";			
			case SDAZ: return "Dev AZ";
			case SDEL: return "Dev EL";
			case R: return "Range";
			case AZ: return "Azimuth";
			case EL: return "Elevation";
			}
		} else if( this.system == GEOCENTRIC ) {
			switch(col) {
			case ID: return "System";
			case MAPRG: return "RG?";
			case MAPAZ: return "AZ?";
			case MAPEL: return "EL?";
			case LAT: return "E";
			case LON: return "F";
			case H: return "G";
			case DRG: return "Bias R";			
			case DAZ: return "Bias AZ";
			case DEL: return "Bias EL";
			case SDRG: return "Dev R";			
			case SDAZ: return "Dev AZ";
			case SDEL: return "Dev EL";
			case R: return "Range";
			case AZ: return "Azimuth";
			case EL: return "Elevation";
			}
		}
		return "";
	}  
	
	@Override
	public Class<?> getColumnClass(int col) {
		switch(col) {
		case ID: return String.class;
		case MAPRG: return Boolean.class;
		case MAPAZ: return Boolean.class;
		case MAPEL: return Boolean.class;
		case LAT: return Double.class;
		case LON: return Double.class;
		case H: return Double.class;
		case DRG: return Double.class;			
		case DAZ: return Double.class;
		case DEL: return Double.class;
		case SDRG: return Double.class;			
		case SDAZ: return Double.class;
		case SDEL: return Double.class;
		case R: return Double.class;
		case AZ: return Double.class;
		case EL: return Double.class;
		default: return Object.class;
		}
	}
	
	@Override
	public boolean isCellEditable(int row, int col) {
		if( col==R || col==AZ || col==EL )
			return false;
		return true;
	} 
	
	@Override
	public Object getValueAt(int row, int col) {
		Pedestal pedestal = pedestals.get(row);
		if( this.system == ELLIPSOIDAL ) {
			switch(col) {
			case ID: return pedestal.getSystemId();
			case MAPRG: return pedestal.getMapRG();
			case MAPAZ: return pedestal.getMapAZ();
			case MAPEL: return pedestal.getMapEL();
			case LAT:
				return pedestal.getLocationEllipsoid().getNorthLatitude().getDegrees();
			case LON:
				return pedestal.getLocationEllipsoid().getEastLongitude().signedPrinciple().getDegrees();
			case H:
				return pedestal.getLocationEllipsoid().getEllipsoidHeight();
			case DRG:
				Double mu_r = pedestal.getBiasRG();
				if (!mu_r.isNaN())
					return mu_r;
				break;
			case DAZ:
				Double mu_az = pedestal.getBiasAZ().getDegrees();
				if (!mu_az.isNaN())
					return mu_az;
				break;
			case DEL:
				Double mu_el = pedestal.getBiasEL().getDegrees();
				if (!mu_el.isNaN())
					return mu_el;
				break;
			case SDRG:
				Double sigma_r = pedestal.getDeviationRG();
				if (!sigma_r.isNaN())
					return sigma_r;
				break;
			case SDAZ:
				Double sigma_az = pedestal.getDeviationAZ().getDegrees();
				if (!sigma_az.isNaN())
					return sigma_az;
				break;
			case SDEL:
				Double sigma_el = pedestal.getDeviationEL().getDegrees();
				if (!sigma_el.isNaN())
					return sigma_el;
				break;
			case R:
				Double r = pedestal.getLocal().getRange();
				if( !r.isNaN() ) 
					return r;
				break;
			case AZ:
				Double az = pedestal.getLocal().getUnsignedAzimuth().getDegrees();
				if( !az.isNaN() ) 
					return az;
				break;
			case EL:
				Double el = pedestal.getLocal().getElevation().getDegrees();
				if( !el.isNaN() ) 
					return el;
				break;
			}
		} else if( this.system == GEOCENTRIC ) {
			switch(col) {
			case ID: return pedestal.getSystemId();
			case MAPRG: return pedestal.getMapRG();
			case MAPAZ: return pedestal.getMapAZ();
			case MAPEL: return pedestal.getMapEL();
			case LAT: return pedestal.getLocation().getX();
			case LON: return pedestal.getLocation().getY();
			case H: return pedestal.getLocation().getZ();
			case DRG:
				Double mu_r = pedestal.getBiasRG();
				if (!mu_r.isNaN())
					return mu_r;
				break;
			case DAZ:
				Double mu_az = pedestal.getBiasAZ().getDegrees();
				if (!mu_az.isNaN())
					return mu_az;
				break;
			case DEL:
				Double mu_el = pedestal.getBiasEL().getDegrees();
				if (!mu_el.isNaN())
					return mu_el;
				break;
			case SDRG:
				Double sigma_r = pedestal.getDeviationRG();
				if (!sigma_r.isNaN())
					return sigma_r;
				break;
			case SDAZ:
				Double sigma_az = pedestal.getDeviationAZ().getDegrees();
				if (!sigma_az.isNaN())
					return sigma_az;
				break;
			case SDEL:
				Double sigma_el = pedestal.getDeviationEL().getDegrees();
				if (!sigma_el.isNaN())
					return sigma_el;
				break;
			case R:
				Double r = pedestal.getLocal().getRange();
				if( !r.isNaN() ) return r;
				break;
			case AZ:
				Double az = pedestal.getLocal().getUnsignedAzimuth().getDegrees();
				if( !az.isNaN() ) return az;
				break;
			case EL:
				Double el = pedestal.getLocal().getElevation().getDegrees();
				if( !el.isNaN() ) return el;
				break;
			}
		}
		return null;
	}
	
	@Override
	public void setValueAt(Object value, int row, int col) {
		Pedestal pedestal = pedestals.get(row);
		if( this.system == ELLIPSOIDAL ) {
			switch(col) {
			case ID:
				pedestal.setSystemId((String) value);
				break;
			case MAPRG:
				pedestal.setMapRG((Boolean) value);
				break;
			case MAPAZ:
				pedestal.setMapAZ((Boolean) value);
				break;
			case MAPEL:
				pedestal.setMapEL((Boolean) value);
				break;
			case LAT:
				pedestal.locateLatitude(Angle.inDegrees((Double) value));
				break;
			case LON:
				pedestal.locateLongitude(Angle.inDegrees((Double) value));
				break;
			case H:
				pedestal.locateEllipsoidHeight((Double) value);
				break;
			case DRG: 
				pedestal.setBiasRG((Double) value);
				break;
			case DAZ: 
				pedestal.setBiasAZ(Angle.inDegrees((Double) value));
				break;
			case DEL: 
				pedestal.setBiasEL(Angle.inDegrees((Double) value));
				break;
			case SDRG: 
				pedestal.setDeviationRG((Double) value);
				break;
			case SDAZ: 
				pedestal.setDeviationAZ(Angle.inDegrees((Double) value));
				break;
			case SDEL: 
				pedestal.setDeviationEL(Angle.inDegrees((Double) value));
				break;
			case R:
				pedestal.pointRange((Double) value);
				break;
			case AZ:
				pedestal.pointAzimuth(Angle.inDegrees((Double) value));
				break;
			case EL:
				pedestal.pointElevation(Angle.inDegrees((Double) value));
				break;
			}
		} else if (this.system == GEOCENTRIC) {
			switch (col) {
			case ID:
				pedestal.setSystemId((String) value);
				break;
			case MAPRG:
				pedestal.setMapRG((Boolean) value);
				break;
			case MAPAZ:
				pedestal.setMapAZ((Boolean) value);
				break;
			case MAPEL:
				pedestal.setMapEL((Boolean) value);
				break;
			case LAT:
				pedestal.locateX((Double) value);
				break;
			case LON:
				pedestal.locateY((Double) value);
				break;
			case H:
				pedestal.locateZ((Double) value);
				break;
			case DRG: 
				pedestal.setBiasRG((Double) value);
				break;
			case DAZ: 
				pedestal.setBiasAZ(Angle.inDegrees((Double) value));
				break;
			case DEL: 
				pedestal.setBiasEL(Angle.inDegrees((Double) value));
				break;
			case SDRG: 
				pedestal.setDeviationRG((Double) value);
				break;
			case SDAZ: 
				pedestal.setDeviationAZ(Angle.inDegrees((Double) value));
				break;
			case SDEL: 
				pedestal.setDeviationEL(Angle.inDegrees((Double) value));
				break;
			case R:
				pedestal.pointRange((Double) value);
				break;
			case AZ:
				pedestal.pointAzimuth(Angle.inDegrees((Double) value));
				break;
			case EL:
				pedestal.pointElevation(Angle.inDegrees((Double) value));
				break;
			}
		}
	}
	
	static final String PedestalHeader = "id,mapRG,mapAZ,mapEL,lat(deg),lon(deg),h(m),mu_r,mu_az,mu_el,sigma_r,sigma_az,sigma_el,r,az,el";

	// TODO replace this with something standard!!!!!!!!!!!	
	public void load(File file) throws Exception {
		pedestals.clear();
		BufferedReader reader = new BufferedReader( new FileReader(file) );
		String line = "";
		int n=1;
		try {
			// read the header row...
			reader.readLine(); 
			// read the data rows...
			while( (line=reader.readLine()) != null) {
				String cols[] = line.split(",");
				if(cols.length==0 || (cols.length==1&&cols[ID].equals("")))
					continue;
				
				String id = cols[ID].trim();
				
				//Sensor map
				Boolean hasRG = Boolean.parseBoolean(cols[MAPRG].trim());
				Boolean hasAZ = Boolean.parseBoolean(cols[MAPAZ].trim());
				Boolean hasEL = Boolean.parseBoolean(cols[MAPEL].trim());
				
				//location
				Double lat = Double.parseDouble(cols[LAT].trim());
				Double lon = Double.parseDouble(cols[LON].trim());
				Double h = Double.parseDouble(cols[H].trim());

				Pedestal pedestal = new Pedestal(id, hasRG, hasAZ, hasEL, Angle.inDegrees(lat), Angle.inDegrees(lon), h);

				Double mu_r = Double.parseDouble(cols[DRG].trim());
				Double mu_az = Double.parseDouble(cols[DAZ].trim());
				Double mu_el = Double.parseDouble(cols[DEL].trim());
				pedestal.setBias(new Polar(mu_r, Angle.inDegrees(mu_az), Angle.inDegrees(mu_el)));
				
				Double sigma_r = Double.parseDouble(cols[SDRG].trim());
				Double sigma_az = Double.parseDouble(cols[SDAZ].trim());
				Double sigma_el = Double.parseDouble(cols[SDEL].trim());				
				pedestal.setDeviation(new Polar(sigma_r, Angle.inDegrees(sigma_az), Angle.inDegrees(sigma_el)));
				
				//positioning (aperture pointing)
				Double az = Double.parseDouble(cols[AZ].trim());
				Double el = Double.parseDouble(cols[EL].trim());	
				//positioning rg gate
				Double r = Double.parseDouble(cols[R].trim());				
				if(Double.isNaN(az)&&Double.isNaN(az)&&Double.isNaN(el)) { //Increment program usage...temp by-pass...
					pedestal.setMapSensors(hasRG, hasAZ, hasEL);
				}else {				
					pedestal.setMapSensors(hasRG, hasAZ, hasEL);
					pedestal.point(new Polar(r, Angle.inDegrees(az), Angle.inDegrees(el)));
				}
				pedestals.add(pedestal);
				n++;
			}
		} catch(Exception exception) {
			throw new Exception( "Error, line "+n+" : Format should be \""+PedestalHeader+"\". Stopped Loading." );
		} finally {
			reader.close();
			this.fireTableDataChanged();
		}
	}
	
	// TODO replace this with something that doesn't suck.
	public void save(File file) throws Exception {
		this.system = ELLIPSOIDAL;
		PrintWriter writer = new PrintWriter(file);
		writer.append( PedestalHeader );
		writer.println();
		
		for(Pedestal pedestal : pedestals) {
			// id
			writer.append(pedestal.getSystemId());
			writer.append(",");
			
			//sensor map
			writer.append(((Boolean) pedestal.getMapRG()).toString());
			writer.append(",");
			writer.append(((Boolean) pedestal.getMapAZ()).toString());
			writer.append(",");
			writer.append(((Boolean) pedestal.getMapEL()).toString());
			writer.append(",");
			
			// geocentric coordinates
			Ellipsoid wgs84 = pedestal.getLocationEllipsoid();
			writer.append(wgs84.getNorthLatitude().toDegreesString(7));
			writer.append(",");
			writer.append(wgs84.getEastLongitude().signedPrinciple().toDegreesString(7));
			writer.append(",");
			writer.append(Double.toString(wgs84.getEllipsoidHeight()));
			writer.append(",");
						
			// measurement bias in the polar directions
			Polar bias = pedestal.getBias();
			writer.append(Double.toString(bias.getRange()) );
			writer.append(",");
			writer.append(bias.getSignedAzimuth().toDegreesString(7));
			writer.append(",");
			writer.append(bias.getElevation().toDegreesString(7));
			writer.append(",");
			
			// measurement deviation in the polar directions
			Polar deviation = pedestal.getDeviation();
			writer.append(Double.toString(deviation.getRange()) );
			writer.append(",");
			writer.append(deviation.getSignedAzimuth().toDegreesString(7));
			writer.append(",");
			writer.append(deviation.getElevation().toDegreesString(7));
			writer.append(",");
			
			// current orientation
			Polar local = pedestal.getLocal();
			writer.append(Double.toString(local.getRange()) );
			writer.append(",");
			writer.append(local.getUnsignedAzimuth().toDegreesString(7));
			writer.append(",");
			writer.append(local.getElevation().toDegreesString(7));
			
			writer.println();
		}
		writer.close();
	}
	
	public static class CellRenderer extends DefaultTableCellRenderer {
		
		DecimalFormat meters = new DecimalFormat("#0.##;-#0.##");
		DecimalFormat degrees = new DecimalFormat("#0.00#####;-#0.00#####");
		
		@Override
		public Component getTableCellRendererComponent(JTable table, Object value,
				boolean isSelected, boolean hasFocus, int row, int col) {
			
			// cell renderers avoid creating new objects by reusing the same component and just adjusting it's properties
			super.getTableCellRendererComponent(
					table, value, isSelected, hasFocus, row, col);
			
			// check if the field holds a double
			if(value!=null) {
				TableModel model = table.getModel();
				
				if(model.getColumnClass(col) == Double.class)
					if(col==R || col==DRG || col==H ) {
					this.setValue( meters.format(value) );
					} else if(col==AZ || col==EL || col==DAZ || col==DEL || col==LAT || col==LON ) {
						this.setValue( degrees.format(value) );
					} else {
						
					}
			}
			
			
			
			if(col==DRG || col==AZ || col==EL || col== R ){
				this.setEnabled(false);
				this.setForeground(Color.blue);
			} else {
				this.setEnabled(true);
				this.setForeground(Color.black);
			}
			
			return this;
		}
	}
	
	// test program
	public static void main(String args[]) {
		File input = new File("C:\\Users\\shiel\\Documents\\workspace\\tracker\\data\\pedestalsExample.csv");
		File output = new File("C:\\Users\\shiel\\Documents\\workspace\\tracker\\data\\pedestalsExampleEcho.csv");
		PedestalModel model = new PedestalModel();
		try {
			model.load(input);
			model.save(output);
		} catch( Exception exception ) {
			exception.printStackTrace();
		}
	}
}
