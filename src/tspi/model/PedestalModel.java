package tspi.model;

import java.awt.Color;
import java.awt.Component;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;

import javax.swing.JTable;
//import javax.swing.ListSelectionModel;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.DefaultTableCellRenderer;

import rotation.Angle;

/** A model which represents a list of Pedestals. Also provides a file load and save, as well as a CellRenderer appropriate for the model. */
@SuppressWarnings("serial")
public class PedestalModel extends AbstractTableModel implements Iterable<Pedestal> {

	protected ArrayList<Pedestal> pedestals;
	public static final int ID=0, MAPAZ=1, MAPEL=2, LAT=3, LON=4, H=5, AZ=6, EL=7, R= 8, DAZ=9, DEL=10, DRG=11;
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
	public int getColumnCount() { return 12; }

	@Override
	public int getRowCount() { return pedestals.size(); }

	@Override
	public String getColumnName(int col) {
		if( this.system == ELLIPSOIDAL ) {
			switch(col) {
			case ID: return "System";
			case MAPAZ: return "Maps AZ";
			case MAPEL: return "Maps EL";
			case LAT: return "North Latitude";
			case LON: return "East Longitude";
			case H: return "Height";
			case AZ: return "Azimuth";
			case EL: return "Elevation";
			case R: return "Range";
			case DAZ: return "Bias AZ";
			case DEL: return "Bias EL";
			case DRG: return "Bias R*";			
			}
		} else if( this.system == GEOCENTRIC ) {
			switch(col) {
			case ID: return "System";
			case MAPAZ: return "Maps AZ";
			case MAPEL: return "Maps EL";
			case LAT: return "E";
			case LON: return "F";
			case H: return "G";
			case AZ: return "Azimuth";
			case EL: return "Elevation";
			case R: return "Range";
			case DAZ: return "Bias AZ";
			case DEL: return "Bias EL";
			case DRG: return "Bias R*";			
			}
		}
		return "";
	}  
	
	@Override
	public Class<?> getColumnClass(int col) {
		switch(col) {
		case ID: return String.class;
		case MAPAZ: return Boolean.class;
		case MAPEL: return Boolean.class;
		case LAT: return Double.class;
		case LON: return Double.class;
		case H: return Double.class;
		case AZ: return Double.class;
		case EL: return Double.class;
		case R: return Double.class;
		case DAZ: return Double.class;
		case DEL: return Double.class;
		case DRG: return Double.class;			
		default: return Object.class;
		}
	}
	
	@Override
	public boolean isCellEditable(int row, int col) {
		if(col==DRG || col==AZ || col==EL || col==R)
			return false;
		return true;
	} 
	
	@Override
	public Object getValueAt(int row, int col) {
		Pedestal pedestal = pedestals.get(row);
		if( this.system == ELLIPSOIDAL ) {
			switch(col) {
			case ID: return pedestal.getSystemId();
			case MAPAZ: return pedestal.getMapAZ();
			case MAPEL: return pedestal.getMapEL();
			case LAT: return pedestal.getLocationEllipsoid().getNorthLatitude();
			case LON: return pedestal.getLocationEllipsoid().getEastLongitude().signedPrinciple();
			case H: return pedestal.getLocationEllipsoid().getEllipsoidHeight();
			case AZ:
				Double az = pedestal.getLocal().getAzimuth().getDegrees();
				if( !az.isNaN() ) return az;
				break;
			case EL:
				Double el = pedestal.getLocal().getElevation().getDegrees();
				if( !el.isNaN() ) return el;
				break;
			case R:
				Double r = pedestal.getLocal().getRange();
				if( !r.isNaN() ) return r;
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
			case DRG:
				Double mu_r = pedestal.getBiasRG();
				if (!mu_r.isNaN())
					return mu_r;
				break;
			}
		} else if( this.system == GEOCENTRIC ) {
			switch(col) {
			case ID: return pedestal.getSystemId();
			case MAPAZ: return pedestal.getMapAZ();
			case MAPEL: return pedestal.getMapEL();
			case LAT: return pedestal.getLocation().getX();
			case LON: return pedestal.getLocation().getY();
			case H: return pedestal.getLocation().getZ();
			case AZ:
				Double az = pedestal.getLocal().getAzimuth().getDegrees();
				if( !az.isNaN() ) return az;
				break;
			case EL:
				Double el = pedestal.getLocal().getElevation().getDegrees();
				if( !el.isNaN() ) return el;
				break;
			case R:
				Double r = pedestal.getLocal().getRange();
				if( !r.isNaN() ) return r;
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
			case DRG:
				Double mu_r = pedestal.getBiasRG();
				if (!mu_r.isNaN())
					return mu_r;
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
			case AZ:
				pedestal.pointAzimuth(Angle.inDegrees((Double) value));
				break;
			case EL:
				pedestal.pointElevation(Angle.inDegrees((Double) value));
				break;
			case R:
				pedestal.pointRange((Double) value);
				break;
			case DAZ: 
				pedestal.setBiasAZ(Angle.inDegrees((Double) value));
				break;
			case DEL: 
				pedestal.setBiasEL(Angle.inDegrees((Double) value));
				break;
			case DRG: 
				pedestal.setBiasRG((Double) value);
				break;
			}
		} else if (this.system == GEOCENTRIC) {
			switch (col) {
			case ID:
				pedestal.setSystemId((String) value);
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
			case AZ:
				pedestal.pointAzimuth(Angle.inDegrees((Double) value));
				break;
			case EL:
				pedestal.pointElevation(Angle.inDegrees((Double) value));
				break;
			case R:
				pedestal.pointRange((Double) value);
				break;
			case DAZ: 
				pedestal.setBiasAZ(Angle.inDegrees((Double) value));
				break;
			case DEL: 
				pedestal.setBiasEL(Angle.inDegrees((Double) value));
				break;
			case DRG: 
				pedestal.setBiasRG((Double) value);
				break;
			}
		}
	}
	
	static final String PedestalHeader = "id,mapAZ,mapEL,lat(deg),lon(deg),h(m),az,el,r,mu_az,mu_el,mu_r,sigma_az,sigma_el,sigma_r";

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
				if(cols.length==0 || (cols.length==1&&cols[0].equals("")))
					continue;
				
				String id = cols[0].trim();
				
				//Sensor map
				Boolean hasAZ = Boolean.parseBoolean(cols[1].trim());
				Boolean hasEL = Boolean.parseBoolean(cols[2].trim());
				
				//location
				Double lat = Double.parseDouble(cols[3].trim());
				Double lon = Double.parseDouble(cols[4].trim());
				Double h = Double.parseDouble(cols[5].trim());

				Pedestal pedestal = new Pedestal(id, hasAZ, hasEL, Angle.inDegrees(lat), Angle.inDegrees(lon), h);

				Double mu_az = Double.parseDouble(cols[9].trim());
				Double mu_el = Double.parseDouble(cols[10].trim());
				Double mu_r = Double.parseDouble(cols[11].trim());
				
				pedestal.setBias( new Polar(mu_r, Angle.inDegrees(mu_az), Angle.inDegrees(mu_el)) );				
				
				//positioning (aperture pointing)
				Double az = Double.parseDouble(cols[6].trim());
				Double el = Double.parseDouble(cols[7].trim());				
				if(Double.isNaN(az)&&Double.isNaN(el)) { //Increment program usage...temp by-pass...
					pedestal.setMapSensors(false, hasAZ, hasEL);
				}else {				
					Double r = Double.parseDouble(cols[8].trim());
					Double sigma_az = Double.parseDouble(cols[12].trim());
					Double sigma_el = Double.parseDouble(cols[13].trim());
					Double sigma_r = Double.parseDouble(cols[14].trim());

					pedestal.setMapSensors(false, hasAZ, hasEL);
					pedestal.point(new Polar(r, Angle.inDegrees(az), Angle.inDegrees(el)));
					pedestal.setBias(new Polar(mu_r, Angle.inDegrees(mu_az), Angle.inDegrees(mu_el)));
					pedestal.setDeviation(new Polar(sigma_r, Angle.inDegrees(sigma_az), Angle.inDegrees(sigma_el)));
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
			writer.append(((Boolean) pedestal.getMapAZ()).toString());
			writer.append(",");
			writer.append(((Boolean) pedestal.getMapEL()).toString());
			writer.append(",");
			
			// geocentric coordinates
			writer.append(pedestal.getLocationEllipsoid().getNorthLatitude().toDegreesString(7));
			writer.append(",");
			writer.append(pedestal.getLocationEllipsoid().getEastLongitude().signedPrinciple().toDegreesString(7));
			writer.append(",");
			writer.append(Double.toString(pedestal.getLocationEllipsoid().getEllipsoidHeight()));
			writer.append(",");
			
			// current orientation
			Polar local = pedestal.getLocal();
			writer.append(local.getAzimuth().toDegreesString(7));
			writer.append(",");
			writer.append(local.getElevation().toDegreesString(7));
			writer.append(",");
			writer.append(Double.toString(local.getRange()) );
			writer.append(",");
			
			// measurement bias in the polar directions
			writer.append(pedestal.getBiasAZ().toDegreesString(7));
			writer.append(",");
			writer.append(pedestal.getBiasEL().toDegreesString(7));
			writer.append(",");
			writer.append(Double.toString(pedestal.getBiasRG()) );
			writer.append(",");
			
			// measurement deviation in the polar directions
			Polar deviation = pedestal.getDeviation();
			writer.append(deviation.getAzimuth().toDegreesString(7));
			writer.append(",");
			writer.append(deviation.getElevation().toDegreesString(7));
			writer.append(",");
			writer.append(Double.toString(deviation.getRange()) );
			
			writer.println();
		}
		writer.close();
	}
	
	public static class CellRenderer extends DefaultTableCellRenderer {
		@Override
		public Component getTableCellRendererComponent(JTable table, Object value,
				boolean isSelected, boolean hasFocus, int row, int col) {
			// get a default label for the cell
			Component cell = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, col);
			// this is from back when we were only fusing two sensors
			// turn off the selection if it is not the first selected or last selected
			// this communicates to the user only two rows can be selected
//			ListSelectionModel selections = table.getSelectionModel();
//			if( row == selections.getMaxSelectionIndex()
//					|| row == selections.getMinSelectionIndex() )
//				cell = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, col);
//			else cell = super.getTableCellRendererComponent(table, value, false, hasFocus, row, col);
			
			if(col==DRG || col==AZ || col==EL || col== R ){
				cell.setEnabled(false);
				cell.setForeground(Color.blue);
			} else {
				cell.setEnabled(true);
				cell.setForeground(Color.black);
			}	
			return cell;
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
