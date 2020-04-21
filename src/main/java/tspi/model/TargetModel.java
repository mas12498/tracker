package tspi.model;

import javax.swing.*;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.DefaultTableCellRenderer;
import java.awt.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Iterator;

/** Represents a set of Targets and adapts them to a table view. Also provides a file load and save, as well as a CellRenderer appropriate for the model. */
@SuppressWarnings("serial")
public class TargetModel extends AbstractTableModel implements Iterable<Target> {

	protected ArrayList<Target> targets = new ArrayList<Target>();
	public static final int TIME=0, LAT=1, LON=2, H=3, RANK=4, COND=5, ERR=6, DX=7, DY=8, DZ=9, ODAZ=10, ODEL=11, ODRG=12;
	public static final int GEOCENTRIC=1, ELLIPSOIDAL=2;
	protected int system = ELLIPSOIDAL;
	private static final int DIGITS=14; 
	public TargetModel() {
		this.targets = new ArrayList<Target>();
	}

	public Target getTarget(int row) {
		return targets.get(row);
	}
	
	public void add(int index, Target target) {
		this.targets.add(index, target);
		this.fireTableRowsInserted(index, index);
	}
	
	public void remove(int index) {
		this.targets.remove(index);
		this.fireTableRowsDeleted(index, index);
	}
	
	public void clearSolutions() {
		for(Target target : targets)
			target.solution = null;
	}
	
	public void setCoordinateSystem(int system) {
		if(system == ELLIPSOIDAL || system == GEOCENTRIC) {
			this.system = system;
			this.fireTableStructureChanged();
		}
	}
	
	@Override
	public Iterator<Target> iterator() { return this.targets.iterator(); }
	
	@Override
	public int getColumnCount() { return 13; }

	@Override
	public int getRowCount() { return targets.size(); }

//	DX=6, DY=7, DZ=8, oDAZ=9, oDEL=10, oDRG=11;	
	
	@Override
	public String getColumnName(int col) {
		if(this.system==ELLIPSOIDAL) {
			switch(col) {
			case TIME: return "Time";
			case LAT: return "North Latitude";
			case LON: return "East Longitude";
			case H: return "Height";
			case RANK: return "Rank Sol";
			case COND: return "Condition";
			case ERR: return "Err";
			case DX: return "Err X";
			case DY: return "Err Y";
			case DZ: return "Err Z";
			case ODAZ: return "origin DAZ";
			case ODEL: return "origin DEL";
			case ODRG: return "origin DRG";			
			}
		} else if( system==GEOCENTRIC ) {
			switch(col) {
			case TIME: return "Time";
			case LAT: return "E";
			case LON: return "F";
			case H: return "G";
			case RANK: return "Rank sol";
			case COND: return "Condition";
			case ERR: return "Error";
			case DX: return "Err X";
			case DY: return "Err Y";
			case DZ: return "Err Z";
			case ODAZ: return "origin DAZ";
			case ODEL: return "origin DEL";
			case ODRG: return "origin DRG";
			}
		}
		return "";
	}
	
	@Override
	public Class<?> getColumnClass(int col) {
		switch(col) {
		case TIME: return Long.class;
		case LAT: return Double.class;
		case LON: return Double.class;
		case H: return Double.class;
		case RANK: return Integer.class;
		case COND: return Double.class;
		case ERR: return Double.class;
		case DX: return Double.class;
		case DY: return Double.class;
		case DZ: return Double.class;
		case ODAZ: return Double.class;
		case ODEL: return Double.class;
		case ODRG: return Double.class;

		default: return Object.class;
		}
	}
	
	@Override
	public Object getValueAt(int row, int col) {
		Target target = targets.get(row);
		if( system==ELLIPSOIDAL ) {
			switch(col) {
			case TIME: return target.getTime();
			case LAT: return target.getLatitude().getDegrees();
			case LON: return target.getLongitude().getDegrees();
			case H: return target.getHeight();
			case RANK:
				if(target.solution!=null && !Double.isNaN(target.solution._error))
					return target.solution._rank;
				break;
			case COND:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._condition;
				break;
			case ERR:
				if(target.solution!=null && !Double.isNaN(target.solution._error))
					return target.solution._error;
				break;
			
			case DX:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_EFG.getX();
				break;
			case DY:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_EFG.getY();
				break;
			case DZ:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_EFG.getZ();
				break;
			case ODAZ:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_RAE.getSignedAzimuth().getDegrees();
				break;
			case ODEL:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_RAE.getElevation().getDegrees();
				break;
			case ODRG:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_RAE.getRange();
				break;
			}			
			
		} else if( system==GEOCENTRIC ) {
			switch(col) {
			case TIME: return target.getTime();
			case LAT: return target.getE();
			case LON: return target.getF();
			case H: return target.getG();
			case RANK:
				if(target.solution!=null && !Double.isNaN(target.solution._error))
					return target.solution._rank;
				break;
			case COND:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._condition;
				break;
			case ERR:
				if(target.solution!=null && !Double.isNaN(target.solution._error))
					return target.solution._error;
				break;
			
			case DX:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_EFG.getX();
				break;
			case DY:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_EFG.getY();
				break;
			case DZ:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_EFG.getZ();
				break;
			case ODAZ:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_RAE.getSignedAzimuth().getDegrees();
				break;
			case ODEL:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_RAE.getElevation().getDegrees();
				break;
			case ODRG:
				if(target.solution!=null && !Double.isNaN(target.solution._condition))
					return target.solution._delta_RAE.getRange();
				break;
			}
		}
		return null;
	}

	@Override
	public boolean isCellEditable(int row, int col) {
		if(col>=ERR )
			return false;
		return true;
	}
	
	@Override
	public void setValueAt(Object value, int row, int col) {
		Target target = targets.get(row);
		if( system==ELLIPSOIDAL) {
			switch(col) {
			case TIME: target.setTime( (Long)value ); break;
			case LAT: target.setLatitude( (Double)value ); break;
			case LON: target.setLongitude( (Double)value ); break;
			case H: target.setHeight( (Double)value ); break;
			case RANK: 
			case COND: 
			case ERR: 
			case DX: 
			case DY: 
			case DZ: 
			case ODAZ: 
			case ODEL: 
			case ODRG: break;
			}
		} else if( system==GEOCENTRIC ) {
			switch(col) {
			case TIME: target.setTime( (Long)value ); break;
			case LAT: target.setE( (Double)value ); break;
			case LON: target.setF( (Double)value ); break;
			case H: target.setG( (Double)value ); break;
			case RANK: 
			case COND: 
			case ERR: 
			case DX: 
			case DY: 
			case DZ: 
			case ODAZ: 
			case ODEL: 
			case ODRG: break;
			}
		}
	}
	
	// TODO replace this with something standard!!!!!!!!!!!
	public void load(File file) throws Exception {
		targets.clear();
		BufferedReader reader = new BufferedReader( new FileReader(file) );
		final String header = "time, NLat, ELon, eHgt";
		String line = "";
		int n=0;
		try {
			while( (line=reader.readLine()) != null) {
				n++;
				if (line.equalsIgnoreCase(header))
					continue;
				String cols[] = line.split(",");
				if(cols.length==0 || (cols.length==1&&cols[0].equals("")))
					continue;
				Double time = Double.parseDouble(cols[0].trim());
				Double lat = Double.parseDouble(cols[1].trim());
				Double lon = Double.parseDouble(cols[2].trim());
				Double h = Double.parseDouble(cols[3].trim());
				Target target = new Target(time, lat, lon, h);
				targets.add(target);
			}
		} catch(Exception exception) {
			
			exception.printStackTrace();
			throw new Exception( "Error, line "+n
					+" : First 4 columns should be time, lat, lon, h\n\""+line+"\"\nStopped loading." );
		} finally {
			reader.close();
			this.fireTableDataChanged();
		}
	}
	
	// TODO replace this with something which isn't lame.
	public void save(File file) throws Exception {
		PrintWriter writer = new PrintWriter(file);
		for(Target target : targets) {
			writer.append(target.getTime().toString());
			writer.append(",");
			writer.append(target.getLatitude().toDegreesString(DIGITS));
			writer.append(",");
			writer.append(target.getLongitude().toDegreesString(DIGITS));
			writer.append(",");
			writer.append(Double.toString(target.getHeight()));
			
//			NumberFormat localFmt = NumberFormat.getInstance();
//			localFmt.setMaximumFractionDigits(6);
			
			if(target.solution!=null) {
				writer.append(",");
				writer.append(Double.toString(target.solution._error));
//				writer.append(localFmt.format(target.solution._error));
				writer.append(",");
				writer.append(Double.toString(target.solution._condition));
//				writer.append(localFmt.format(target.solution._condition));
				// TODO there will be more complex measures of error...
			}
			writer.println();
		}
		writer.close();
	}
	
	public static class CellRenderer extends DefaultTableCellRenderer {
		
		@Override
		public Component getTableCellRendererComponent(JTable table, Object value,
				boolean isSelected, boolean hasFocus, int row, int col) {
			
			// turn off the selection if it is not the first selected or last selected
			// this communicates to the user only two rows can be selected
			ListSelectionModel selections = table.getSelectionModel();
			if (row == selections.getMaxSelectionIndex() || row == selections.getMinSelectionIndex())
				super.getTableCellRendererComponent(table, value, true, hasFocus, row, col);
			else
				super.getTableCellRendererComponent(table, value, false, hasFocus, row, col);

			// get the table model so we can look up information
//			TableModel model = table.getModel();
			DecimalFormat meters = new DecimalFormat("#0.###;-#0.###");
			DecimalFormat degrees = new DecimalFormat("#0.0000000;-#0.0000000");
			
			// conditionally format the value
			if (value==null)
				setValue("");
			else if (col==H || col==COND || col==ERR || col== DX || col== DY || col== DZ || col== ODRG )
				setValue(meters.format(value));
			else if (col==LAT || col==LON || col==ODAZ || col== ODEL  )
				setValue(degrees.format(value));
			else
				setValue( value.toString() );
			
			// prevent user from editing fields which are generated
			if (col >= ERR)
				setEnabled(false);
			else
				setEnabled(true);

			return this;
		}
	}
}
