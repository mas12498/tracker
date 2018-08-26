package tspi.model;
import java.awt.Component;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;

import javax.swing.JTable;
import javax.swing.ListSelectionModel;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.DefaultTableCellRenderer;

import rotation.Angle;

/** Represents a set of Targets and adapts them to a table view. Also provides a file load and save, as well as a CellRenderer appropriate for the model. */
@SuppressWarnings("serial")
public class TargetModel extends AbstractTableModel implements Iterable<Target> {

	protected ArrayList<Target> targets = new ArrayList<Target>();
	public static final int TIME=0, LAT=1, LON=2, H=3, ERR = 4, COND =5;
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
	public int getColumnCount() { return 6; }

	@Override
	public int getRowCount() { return targets.size(); }

	@Override
	public String getColumnName(int col) {
		if(this.system==ELLIPSOIDAL) {
			switch(col) {
			case TIME: return "Time";
			case LAT: return "North Latitude";
			case LON: return "East Longitude";
			case H: return "Height";
			case ERR: return "Error";
			case COND: return "Condition";
			}
		} else if( system==GEOCENTRIC ) {
			switch(col) {
			case TIME: return "Time";
			case LAT: return "E";
			case LON: return "F";
			case H: return "G";
			case ERR: return "Error";
			case COND: return "Condition";
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
		case ERR: return Double.class;
		case COND: return Double.class;
		default: return Object.class;
		}
	}
	
	@Override
	public Object getValueAt(int row, int col) {
		Target target = targets.get(row);
		if( system==ELLIPSOIDAL ) {
			switch(col) {
			case TIME: return target.getTime();
			case LAT: return target.getLatitude();
			case LON: return target.getLongitude();
			case H: return target.getHeight();
			case ERR:
				if(target.solution!=null && !Double.isNaN(target.solution.error))
					return target.solution.error;
				break;
			case COND:
				if(target.solution!=null && !Double.isNaN(target.solution.condition))
					return target.solution.condition;
				break;
			}
		} else if( system==GEOCENTRIC ) {
			switch(col) {
			case TIME: return target.getTime();
			case LAT: return target.getE();
			case LON: return target.getF();
			case H: return target.getG();
			case ERR:
				if(target.solution!=null && !Double.isNaN(target.solution.error))
					return target.solution.error;
				break;
			case COND:
				if(target.solution!=null && !Double.isNaN(target.solution.condition))
					return target.solution.condition;
				break;
			}
		}
		return null;
	}

	@Override
	public boolean isCellEditable(int row, int col) {
		if(col==ERR || col==COND)
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
			case ERR: case COND: break;//target.setError( (Double)value ); break;
			}
		} else if( system==GEOCENTRIC ) {
			switch(col) {
			case TIME: target.setTime( (Long)value ); break;
			case LAT: target.setE( (Double)value ); break;
			case LON: target.setF( (Double)value ); break;
			case H: target.setG( (Double)value ); break;
			case ERR: case COND: break;//target.setError( (Double)value ); break;
			}
		}
	}
	
	// TODO replace this with something standard!!!!!!!!!!!
	public void load(File file) throws Exception {
		targets.clear();
		BufferedReader reader = new BufferedReader( new FileReader(file) );
		String line = "";
		int n=1;
		try {
			while( (line=reader.readLine()) != null) {
				String cols[] = line.split(",");
				if(cols.length==0 || (cols.length==1&&cols[0].equals("")))
					continue;
				Long time = Long.parseLong(cols[0].trim());
				Double lat = Double.parseDouble(cols[1].trim());
				Double lon = Double.parseDouble(cols[2].trim());
				Double h = Double.parseDouble(cols[3].trim());
				Target target = new Target(time, lat, lon, h);
				targets.add(target);
				n++;
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
			writer.append(Long.toString(target.getTime()));
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
				writer.append(Double.toString(target.solution.error));
//				writer.append(localFmt.format(target.solution.error));
				writer.append(",");
				writer.append(Double.toString(target.solution.condition));
//				writer.append(localFmt.format(target.solution.condition));
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
			Component cell;
			ListSelectionModel selections = table.getSelectionModel();
			if( row == selections.getMaxSelectionIndex()
					|| row == selections.getMinSelectionIndex() )
				cell = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, col);
			else cell = super.getTableCellRendererComponent(table, value, false, hasFocus, row, col);
			
			if(col==ERR || col==COND)
				cell.setEnabled(false);
			else
				cell.setEnabled(true);
						
			return cell;
		}
	}
}
