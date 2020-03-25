package tspi.model;

import org.apache.commons.math3.linear.*;
import tspi.rotation.Vector3;

import java.util.List;

// TODO Single Point Solution...NO prior...
public class SinglePoint {
	public Vector3 _position_EFG = new Vector3(Vector3.EMPTY);
	public Vector3 _positionBiased_EFG = new Vector3(Vector3.EMPTY);
	public Vector3 _delta_EFG = new Vector3(Vector3.EMPTY);
	public Polar _delta_RAE = new Polar();
	public double _condition;
	public double _error;
	public int _rank;
	private static final int DIGITS = 14;	
	public SinglePoint(List<Pedestal> pedestals) {
		int pedSensorCnt =0;  //count pedestal sensors active in fuzed solution
		//pedSensorCnt = pedestals.size()*2; //assumes always two valid sensors per pedestal...
		for(Pedestal pedestal : pedestals) {
			pedSensorCnt += 3; // begin with case (by 3) for rg,az,el of pedestal... need not always be 3...
			Ellipsoid ellipsoid = pedestal.getLocationEllipsoid(); //.getWGS84().getEllipsoid();
			System.out.println( "Pedestal "+pedestal.getSystemId()+" : "
//					+ "EFG=" + pedestal.getGeocentricCoordinates().toString(5)
					+ " lon="+ellipsoid.getEastLongitude().signedPrinciple().toDegreesString(DIGITS)
					+ " lat="+ellipsoid.getNorthLatitude().toDegreesString(DIGITS)
					+ " h="+ellipsoid.getEllipsoidHeight()
					+ " "+pedestal.getMapRG()+"_rg=" + pedestal._local.getRange()
					+ " "+pedestal.getMapAZ()+"_az=" + pedestal._local.getUnsignedAzimuth().toDegreesString(7)
					+ " "+pedestal.getMapEL()+"_el=" + pedestal._local.getElevation().toDegreesString(7));
		}					
		Vector3 _projection = new Vector3(Double.NaN,Double.NaN,Double.NaN);		
		double [] rhs = new double [pedSensorCnt]; //max row dimension possible for initialization
		double [][] matrixData = new double [pedSensorCnt][3]; //max row dimension possible for initialization
		int i = 0;
		int iAz = 0; 
		int iEl = 0;
		int iRG = 0;
		for(Pedestal pedestal : pedestals) {	//Assuming three {az,el,rg} axial sensors per pedestal...
			if (pedestal.getMapRG()) { // range axis: dircos axial line of sight (polarization twist)				
				_projection = pedestal._apertureFrame._orientation.getImage_i().unit();
					rhs[i] =  pedestal._local.getRange() + pedestal._localCoordinates.getInnerProduct(_projection);
					matrixData[i][0] = _projection.getX(); matrixData[i][1] = _projection.getY(); matrixData[i][2] = _projection.getZ();								
				i += 1; iRG += 1;
			}			
			if (pedestal.getMapAZ()) { // Horz: axial EL dircos
				_projection = pedestal._apertureFrame._orientation.getImage_j().unit();
					rhs[i] = pedestal._localCoordinates.getInnerProduct(_projection);
					matrixData[i][0] = _projection.getX(); matrixData[i][1] = _projection.getY(); matrixData[i][2] = _projection.getZ();
				i += 1; iAz += 1;
			}
			if (pedestal.getMapEL()) { // Vert: axial AZ dircos
				_projection = pedestal._apertureFrame._orientation.getImage_k().unit();
					rhs[i] = pedestal._localCoordinates.getInnerProduct(_projection);				
					matrixData[i][0] = _projection.getX(); matrixData[i][1] = _projection.getY(); matrixData[i][2] = _projection.getZ();
				i += 1; iEl += 1;
			}
		}		
		RealMatrix a = new Array2DRowRealMatrix(matrixData);  //REFERENCE	ideal pedestal Solution:
//			System.out.println("Pedestals in solution: "+pedSensorCnt/2);
//			System.out.println("Sensors in solution: "+iAz+" AZ + "+iEl+" EL = "+i+" Total.");
			//System.out.println(a.getColumnDimension()); // 3
		SingularValueDecomposition svd = new SingularValueDecomposition(a.getSubMatrix(0,i-1,0, a.getColumnDimension()-1));
		RealVector b = new ArrayRealVector(rhs);
		RealVector y = svd.getSolver().solve(b.getSubVector(0, i));
		Vector3 vector_EFG = new Vector3(y.getEntry(0), y.getEntry(1), y.getEntry(2)); //angles-only solution -- no error
		Polar polar_EFG = Polar.commandLocal(vector_EFG, Pedestal.getOriginFrame()._local);
//			System.out.println("Pedestals in solution: "+pedSensorCnt/2);
//			System.out.println("Sensors in solution: "+iAz+" AZ + "+iEl+" EL = "+i+" Total.");
			System.out.println("COL rank: "+a.getColumnDimension()); // 3						
		System.out.println("Pedestals in solution: "+pedSensorCnt/2);
		System.out.println("Sensors in solution: "+iAz+" AZ + "+iEl+" EL + "+iRG +" RG = "+i+" Total.");
//		System.out.println(a.getColumnDimension()); // 3	
//		RealMatrix covariance = svd.getCovariance(1e-9);
//		RealMatrix vSVD = svd.getV();
//		RealMatrix utSVD = svd.getUT();
//		RealMatrix sSVD = svd.getS();
//		double normSVD = svd.getNorm();
		_rank = svd.getRank();
//		OLSMultipleLinearRegression fit = new OLSMultipleLinearRegression();
//		fit.newSampleData(rhs, matrixData);
//		fit.setNoIntercept(true);	
//        double[] parm = fit.estimateRegressionParameters(); //y    
//		double R2Adjusted = fit.calculateAdjustedRSquared();
//		double mse = fit.estimateErrorVariance();
//		double[] parmSTD = fit.estimateRegressionParametersStandardErrors();		
		this._position_EFG.set( new Vector3(vector_EFG).add(Pedestal.getOrigin()));
		this._condition = svd.getConditionNumber();		
		System.out.println("SVD rank: "+_rank); //need  3				
	System.out.println("Condition number : "+formatted(this._condition,5));
//	System.out.println( this._position_EFG.toString(5) );
	}
		
	public void measureError(Vector3 truth) {
		Vector3 targetPrime;
		if( truth==null )
			targetPrime = new Vector3(0.0,0.0,0.0);
		else targetPrime = new Vector3(this._positionBiased_EFG);
		targetPrime.subtract( truth );
		this._error = targetPrime.getAbs();
	}
	
	public String formatted(double value, int decimals){
		String fmt = " %."+decimals+"f ";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, value);
	}
	
}
