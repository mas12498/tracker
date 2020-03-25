package tspi.controller;

import tspi.model.*;
import tspi.rotation.Angle;
import tspi.rotation.Vector3;

import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.event.TableModelEvent;
import javax.swing.event.TableModelListener;
import javax.swing.table.DefaultTableCellRenderer;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.ArrayList;

/** A controller for manipulating a set of pedestals and their targets. Meant 
 * to demonstrate the deliverables of Increment 1 of the Predictive TSPI 
 * project. */
@SuppressWarnings("serial")
public class Increment3 extends JFrame 
implements ActionListener, ListSelectionListener, TableModelListener {
	protected PedestalModel pedestals;
	protected TargetModel targets;
	protected JTable pedestalTable;
	protected JTable targetTable;
	protected JMenuItem setOrigin;
	protected JMenuItem addPedestal;
	protected JMenuItem addTarget;
	protected JMenuItem removePedestal;
	protected JMenuItem removeTarget;
	protected JMenuItem loadPedestals;
	protected JMenuItem loadTargets;
	protected JMenuItem savePedestals;
	protected JMenuItem saveTargets;
	protected JMenuItem coordGeocentric;
	protected JMenuItem coordEllipsoidal;
	protected JMenuItem about;
	protected DefaultTableCellRenderer cell;
	
	private static final int DIGITS = 14;
	
	
	
	public Increment3() {
		
		
		
		//pedestals model table
		pedestals = new PedestalModel();
		pedestals.addTableModelListener( this );		
		
		pedestalTable = new JTable(pedestals);
		pedestalTable.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
		pedestalTable.setRowSelectionAllowed(true);
		pedestalTable.setColumnSelectionAllowed(false);
		pedestalTable.createDefaultColumnsFromModel();
		pedestalTable.getSelectionModel().addListSelectionListener( this );	
		
		this.cell = new PedestalModel.CellRenderer();
		pedestalTable.setDefaultRenderer(Double.class, cell);
		pedestalTable.setDefaultRenderer(String.class, cell);
		//pedestalTable.getColumnModel().getColumn(0).setCellRenderer(cell);
		
		
		
		//targets model table
		targets = new TargetModel();
		targets.addTableModelListener( this );
		
		targetTable = new JTable( targets );
		targetTable.setSelectionMode( ListSelectionModel.SINGLE_SELECTION );
		targetTable.setRowSelectionAllowed( true );
		targetTable.setColumnSelectionAllowed( false );
		targetTable.createDefaultColumnsFromModel();
		targetTable.getSelectionModel().addListSelectionListener( this );
		
		this.cell = new TargetModel.CellRenderer();
		targetTable.setDefaultRenderer(Double.class, cell);
		targetTable.setDefaultRenderer(String.class, cell);
		//targetTable.getColumnModel().getColumn(0).setCellRenderer(cell);
		
		
		//pedestals pnlPedestals pane
		JPanel pnlPedestals = new JPanel( new BorderLayout() );
		JLabel lblPedestals = new JLabel("Pedestals");
		lblPedestals.setHorizontalAlignment(JLabel.CENTER);
		JScrollPane scrPedestals = new JScrollPane(pedestalTable);
		pnlPedestals.add(lblPedestals, BorderLayout.NORTH);
		pnlPedestals.add(scrPedestals, BorderLayout.CENTER);
		
		
		//targets pnlTarget pane
		JPanel pnlTarget = new JPanel( new BorderLayout() );
		JLabel lblTarget = new JLabel("Targets");
		lblTarget.setHorizontalAlignment(JLabel.CENTER);
		JScrollPane scrTarget = new JScrollPane(targetTable);
		pnlTarget.add(lblTarget, BorderLayout.NORTH);
		pnlTarget.add(scrTarget, BorderLayout.CENTER);
		
		//split panes layout
		JSplitPane split = new JSplitPane(
				JSplitPane.VERTICAL_SPLIT, true, pnlPedestals, pnlTarget);
		split.setResizeWeight(0.25);
		split.setDividerLocation(0.25);

		//file menu drop down
		loadPedestals = new JMenuItem( "Load Pedestals" );
		loadTargets = new JMenuItem( "Load Targets" );
		loadPedestals.addActionListener(this);
		loadTargets.addActionListener(this);
		savePedestals = new JMenuItem( "Save Pedestals" );
		saveTargets = new JMenuItem( "Save Targets" );
		savePedestals.addActionListener(this);
		saveTargets.addActionListener(this);
		JMenu file = new JMenu("File");
		file.add(loadPedestals);
		file.add(loadTargets);
		file.add(savePedestals);
		file.add(saveTargets);
		
		//edit menu drop down
		setOrigin = new JMenuItem("Set Origin");
		addPedestal = new JMenuItem("Add Pedestal");
		addTarget = new JMenuItem("Add Target");
		removePedestal = new JMenuItem("Remove Pedestal");
		removeTarget = new JMenuItem("Remove Target");
		setOrigin.addActionListener(this);
		addPedestal.addActionListener(this);
		addTarget.addActionListener(this);
		removePedestal.addActionListener(this);
		removeTarget.addActionListener(this);
		JMenu edit = new JMenu("Edit");
		edit.add(setOrigin);
		edit.add(addPedestal);
		edit.add(addTarget);
		edit.add(removePedestal);
		edit.add(removeTarget);
		
		//coordinates menu drop down
		coordEllipsoidal = new JMenuItem("Ellipsoidal");
		coordEllipsoidal.addActionListener(this);
		coordGeocentric = new JMenuItem("Geocentric");
		coordGeocentric.addActionListener(this);
		JMenu coordinates = new JMenu("Coordinates");
		coordinates.add(coordEllipsoidal);
		coordinates.add(coordGeocentric);
		
		//about menu popup
		about = new JMenuItem("About");
		about.addActionListener(this);
		
		//TODO add or remove systems and targets
		//JMenu edit = new JMenu("Edit");
		
		
		//menu bar 
		JMenuBar bar = new JMenuBar();
		bar.add(file);
		bar.add(edit);
		bar.add(coordinates);
		bar.add(about);
		
		//application gui layout
		this.setLayout( new BorderLayout() );
		this.add(bar, BorderLayout.NORTH);
		this.add(split, BorderLayout.CENTER);
		this.setTitle("TSPI Predictor; Increment 3");
		this.setBounds(100, 100, 1000, 600);
		this.setDefaultCloseOperation(EXIT_ON_CLOSE);
	}
	
	/** Usecase 1: Update all pedestal's vector states to reference point-out of the selected target*/
	public void ComputeDirections(Target target, PedestalModel pedestals) {
		Vector3 geo = target.getGeocentricCoordinates();
		if(geo==null) { //pedestal location not created[?]
			//System.out.println("ComputeDirections(): Invalid Target Coordinates( "+target.getTime()+", "+target.getLatitude()+", "+target.getLongitude()+", "+target.getHeight()+")");
			return;
		}
		for(Pedestal pedestal : pedestals)
			pedestal.pointToLocation( geo );
	}
	

	
	/**
	 * Usecase 2: Updates errors computed in location of all targets using only
	 * assigned (checked) sensors of two or more selected pedestals.
	 */
	public void ComputeError(ArrayList<Pedestal> selected, TargetModel targets) {
		/**
		 * The location errors of fused sensor solutions are defined with respect to an
		 * arbitrary pedestal coordinate frame origin location. In this application, we
		 * define the origin location to be defined as the first pedestal given in the
		 * pedestals table list -- And, this origin pedestal need not provide any
		 * sensors contributing to the fused solution of target locations.
		 * 
		 */
		// for each target
		for(Target target : targets) {
			Vector3 geo = target.getGeocentricCoordinates();
			if(geo==null) {
				//System.out.println("ComputeError(): Invalid Target Coordinates( "+target.getTime()+", "+target.getLatitude()+", "+target.getLongitude()+", "+target.getHeight()+")");
				continue;
			}
			Ellipsoid ellipsoid = target.getEllipsoidalCoordinates().getEllipsoid();
			
			//Debug information to console:
			System.out.println( "\nTarget "+target.getTime()+" : " 
					+ " lon="+ellipsoid.getEastLongitude().signedPrinciple().toDegreesString(DIGITS)
					+ " lat="+ellipsoid.getNorthLatitude().toDegreesString(DIGITS)
					+ " h="+ellipsoid.getEllipsoidHeight());
			
			// point pedestals to each target
			for(Pedestal pedestal : selected)
				pedestal.pointToLocation(geo);
			
//			// TODO obtain the origin from reference of a distinct input ...instead of just the first pedestal in list!
//			Vector3 origin = new Vector3( selected.get(0).getLocation() );
			
			// compute measured target location fused from pedestal model
			Solution fused = new Solution( selected );
			
			// compute measured target location error
			fused.measureError( target.getGeocentricCoordinates() );
			
			//output to targets
			target.setSolution(fused);
			
//			// TODO display more conditioning and error ellipse info, maybe even the calculated target location.
		}
	}
	
	/** Pedestal and Target table selection listener. */
	@Override
	public void valueChanged(ListSelectionEvent event) {
		//don't do anything until the user is done highlighting
		if( event.getValueIsAdjusting() )
			return;
		
		// Pedestals were selected
		// The bounds of the selected interval are used as the input pedestals to compute error
		if( event.getSource() == this.pedestalTable.getSelectionModel() ) {
						
           
			/* Before selection...check if origin was changed... AND fix local coordinates: */
			
			//Clear old "Origin: " prefix if below first...
			int pNum = pedestals.getRowCount();
			for (int p = 1; p < pNum; p++) { //start below origin position...
				String idPedStr =  pedestals.getPedestal(p).getSystemId();
				int idPedStrLen = idPedStr.length();
				if (idPedStrLen >= 8) {
					if (idPedStr.startsWith("Origin:")) {
						pedestals.getPedestal(0);
						if(idPedStrLen<8) {
							pedestals.getPedestal(p).setSystemId("o"+p);							
						}else {
							pedestals.getPedestal(p).setSystemId(idPedStr.substring(8, idPedStrLen));
						}						

					}
				}
				
			}	
			
			// Force the "Origin: " prefix...
			String idPedStr = pedestals.getPedestal(0).getSystemId();
			if (idPedStr.startsWith("Origin:")) {
				// need do nothing
				// System.out.println(idPedStr + " "+origin.toString(14));
			} else {
				pedestals.getPedestal(0).setSystemId("Origin: " + idPedStr);
				// System.out.println("Origin Assigned: "+idPedStr+" "+origin.toString(14));
			}

			//Check if origin location matches pedestal(0) location...
			if (!(pedestals.getPedestal(0).getLocation().equals(Pedestal.getOrigin()))) {
				Pedestal.setOrigin(pedestals.getPedestal(0).getLocation());
				pNum = pedestals.getRowCount();
				for (int p = 0; p < pNum; p++) {
					pedestals.getPedestal(p).setLocalOriginCoordinates();
				}
			}	
			
			
            /* pedestal selection collection...pedestals used to resolve fused solutions for targets: */			
			int rows[] = pedestalTable.getSelectedRows();
			ArrayList<Pedestal> list = new ArrayList<Pedestal>();
			for(int row : rows) {
				if(row==-1) continue;
				Pedestal pedestal = pedestals.getPedestal(row); //pedestalTable.getRowSorter().convertRowIndexToModel(row) );
				list.add(pedestal);
				//System.out.println("local coordinates: "+pedestals.getPedestal(row).getLocalCoordinates().toString(14));
			}
			
			// deselect the target table, and clear the error deltas
			//targetTable.getSelectionModel().clearSelection();
			targets.clearSolutions();
			
			// make sure enough pedestals are selected for a solution
			Vector3 origin = new Vector3(Pedestal.getOrigin());
			if(list.size() >= 2)
				ComputeError( list, targets);
			
			// clear pedestal heading data
			this.pedestals.clearOrientations();
			
			// notify listeners the data changed
			this.targets.fireTableDataChanged(); // this listener update method also unselects previous selections
			this.pedestals.fireTableRowsUpdated(0, pedestals.getRowCount()-1);

		// a target was selected
		} else if( event.getSource() == this.targetTable.getSelectionModel() ) {
			// deselect the pedestal table
			//pedestalTable.getSelectionModel().clearSelection();
			
			// get the currently selected target
			int row = targetTable.getSelectionModel().getMinSelectionIndex();
			//row = targetTable.getRowSorter().convertRowIndexToModel(row);
			if(row==-1)
				return;
			Target target = targets.getTarget(row);
			
			//System.out.println("Point:\n"+target+"\n\n");
			
			// point all pedestals to the selected target
			ComputeDirections(target, pedestals);
			
			// clear error info
			this.targets.clearSolutions();
			
			// notify all listeners
			this.targets.fireTableRowsUpdated(0, targets.getRowCount()-1);
			this.pedestals.fireTableDataChanged(); // this listener update method also unselects previous selections

		}
	}
	
	/** table edit listener */
	@Override
	public void tableChanged(TableModelEvent event) {
		// TODO for avoiding incorrect numbers that have to be fixed by clicking around;
		// on target edit, should recompute pedestal angles
		// on pedestal angle, should clear target error
	}
	
	/** Menu Item listener. */
	@Override
	public void actionPerformed(ActionEvent event) {
		try{
			if( event.getSource() == this.setOrigin ) {			
				int index = this.pedestalTable.getSelectedRow();
				if(index==-1)
					index = 0;
				Ellipsoid o = new Ellipsoid(pedestals.getPedestal(index).getLocationEllipsoid());
				Pedestal pedestal = new Pedestal("Origin: " + pedestals.getPedestal(index).getSystemId(), false, false,
						false, o.getNorthLatitude(), o.getEastLongitude(), o.getEllipsoidHeight());
				this.pedestals.add(0, pedestal);
				this.pedestalTable.setRowSelectionInterval(index+1, index+1);
				Pedestal.setOrigin(pedestals.getPedestal(0).getLocation());
				int pNum = pedestals.getRowCount();
				for (int p = 0; p < pNum; p++) {
					pedestals.getPedestal(p).setLocalOriginCoordinates();
				}
				
			} else if( event.getSource() == this.addPedestal ) {
				int index = this.pedestalTable.getSelectedRow();
				if(index==-1)
					index = this.pedestals.getRowCount();
				Pedestal pedestal = new Pedestal("",false,false,false,Angle.ZERO,Angle.ZERO,0.0);
				this.pedestals.add(index, pedestal);
				this.pedestalTable.setRowSelectionInterval(index, index);
				
			} else if( event.getSource() == this.addTarget ) {
				int index = this.targetTable.getSelectedRow();
				if(index==-1)
					index = this.targets.getRowCount();
				Target target = new Target(0L,0.0,0.0,0.0);
				this.targets.add(index, target);
				this.targetTable.setRowSelectionInterval(index, index);
			
			} else if( event.getSource() == this.removePedestal ) {
				int index = this.pedestalTable.getSelectedRow();
				if(index!=-1) {
					this.pedestals.remove(index);
					this.pedestalTable.clearSelection();
				} else
					JOptionPane.showMessageDialog(this, "Please select a pedestal.", "Selection needed", JOptionPane.INFORMATION_MESSAGE);

			} else if( event.getSource() == this.removeTarget ) {
				int index = this.targetTable.getSelectedRow();
				if(index!=-1) {
					this.targets.remove(index);
					this.targetTable.clearSelection();
				} else
					JOptionPane.showMessageDialog(this, "Please select a target.", "Selection needed", JOptionPane.INFORMATION_MESSAGE);

			} else if( event.getSource() == this.loadPedestals ) {
				JFileChooser chooser = new JFileChooser();
				if(JFileChooser.APPROVE_OPTION == chooser.showOpenDialog(this)) {
					File file = chooser.getSelectedFile();
					this.pedestals.load(file);
				}
				
			} else if( event.getSource() == this.loadTargets ) {
				JFileChooser chooser = new JFileChooser();
				if(JFileChooser.APPROVE_OPTION == chooser.showOpenDialog(this)) {
					File file = chooser.getSelectedFile();
					this.targets.load(file);
				}
				
			} else if( event.getSource() == this.savePedestals ) {
				JFileChooser chooser = new JFileChooser();
				if(JFileChooser.APPROVE_OPTION == chooser.showSaveDialog(this)) {
					File file = chooser.getSelectedFile();
					this.pedestals.save(file);
				}
				
			} else if( event.getSource() == this.saveTargets ) {
				JFileChooser chooser = new JFileChooser();
				if(JFileChooser.APPROVE_OPTION == chooser.showSaveDialog(this)) {
					File file = chooser.getSelectedFile();
					this.targets.save(file);
				}
				
			} else if( event.getSource()==this.coordEllipsoidal) {
				this.pedestals.setCooordinateSystem(PedestalModel.ELLIPSOIDAL);
				this.targets.setCoordinateSystem(TargetModel.ELLIPSOIDAL);
				
			} else if( event.getSource()==this.coordGeocentric) {
				this.pedestals.setCooordinateSystem(PedestalModel.GEOCENTRIC);
				this.targets.setCoordinateSystem(TargetModel.GEOCENTRIC);
			} else if( event.getSource()==this.about) {
				JOptionPane.showMessageDialog(this, 
						"TSPI Predictor Increment 3\n"
						+ "Mike Shields : Quaternion Library, Regresion Design\n"
						+ "CaseyShields : UI\n\n"
						+ "Case 1 : Compute pedestals' reference azimuth, elevation and range from target location\n"
						+ "   Select one target to aim all pedestals at it.\n\n"
						+ "Case 2 : Compute targets' location from Pedestals' reference azimuth and elevation.\n"
						+ "   Select two or more pedestals. Selected pedestals will be aimed at a target, \n"
						+ "then the target will be re-derived solely from pedestals' location, azimuth and elevation.\n"
						+ "The target solution's error magnitude and condition number is returned.\n"
						+ "This process is repeated for every target in the target table.\n"
						+ "Since no perturbations are introduced, error should only depend on conditioning and round-off errors.");
			}
		} catch(Exception exception) {
			JOptionPane.showMessageDialog(this, exception.getMessage());
		}
	}
	
	public static void main(String args[]) {
		Increment3 frame = new Increment3();
		if(args.length==2) {
			
			// try to load a default pedestal file using the first argument as a path
			File pedestalFile = new File( args[0] );
			try { frame.pedestals.load( pedestalFile ); }
			catch(Exception exception) {
				JOptionPane.showMessageDialog(frame, "Failed to load "+args[1]+" as a pedestal csv:\n"+ exception.getMessage());
			}
			
			// try to load a default target file using the second argument as a path
			File targetFile = new File( args[1] );
			try { frame.targets.load( targetFile ); }
			catch(Exception exception) {
				JOptionPane.showMessageDialog(frame, "Failed to load "+args[1]+" as a taget csv:\n" + exception.getMessage());
			}
		}
		frame.setVisible(true);
	}

	
}
