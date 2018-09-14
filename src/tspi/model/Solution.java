package tspi.model;

import java.util.List;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import rotation.Rotator;
import rotation.Vector3;

// TODO this needs to be fleshed out with system matrices, error models, etc.
// and it probably needs a much more specific name than 'Solution'
// I think Pedestal.computeTarget() actually belongs here too...

public class Solution {
	public Vector3 position_EFG = new Vector3(Vector3.EMPTY);
	public Vector3 delta_EFG = new Vector3(Vector3.EMPTY);
	public Polar delta_RAE = new Polar();
	public double condition;
	public double error;
	private static final int DIGITS = 14;
	
	public Solution(List<Pedestal> pedestals) {		
		
		
		//count pedestal sensors active in fuzed solution
		int pedSensorCnt =0;
		//pedSensorCnt = pedestals.size()*2; //assumes always two valid sensors per pedestal...
		for(Pedestal pedestal : pedestals) {
			// begin with case (by 2) for az,el of pedestal... need not always be 2...
			pedSensorCnt += 2;
			Ellipsoid ellipsoid = pedestal.getLocationEllipsoid(); //.getWGS84().getEllipsoid();
			//Debug
			System.out.println( "Pedestal "+pedestal.getSystemId()+" : "
//					+ "EFG=" + pedestal.getGeocentricCoordinates().toString(5)
					+ " lon="+ellipsoid.getEastLongitude().signedPrinciple().toDegreesString(DIGITS)
					+ " lat="+ellipsoid.getNorthLatitude().toDegreesString(DIGITS)
					+ " h="+ellipsoid.getEllipsoidHeight()
					+ " "+pedestal.getMapAZ()+"_az=" + pedestal._local.getAzimuth().toDegreesString(7)
					+ " "+pedestal.getMapEL()+"_el=" + pedestal._local.getElevation().toDegreesString(7));
		}	
				
		Vector3 row = new Vector3(Double.NaN,Double.NaN,Double.NaN);
		Vector3 rowBiased = new Vector3(Double.NaN,Double.NaN,Double.NaN);
		double [] rhs = new double [pedSensorCnt]; //max row dimension possible for initialization
		double [][] matrixData = new double [pedSensorCnt][3]; //max row dimension possible for initialization
		double [] rhsBiased = new double [pedSensorCnt]; //max row dimension possible for initialization
		double [][] matrixDataBiased = new double [pedSensorCnt][3]; //max row dimension possible for initialization
		Rotator biasedPed = new Rotator();
		Polar biasedLocal = new Polar();
		int i = 0;
		int iAz = 0; 
		int iEl = 0;
		for(Pedestal pedestal : pedestals) {	
			System.out.println("Pedestal "+pedestal.getSystemId()+": q_NG="+pedestal._localFrame.getLocal().toString(5)+"  q_AG = "+pedestal._apertureFrame._orientation.toString(5));
			
			//Assuming two axial sensors per pedestal...			
			if (pedestal.getMapEL()) {
				//reference pedestal solution matrix
				row = pedestal._apertureFrame._orientation.getImage_k().unit();// Vert: axial AZ dircos
				matrixData[i][0] = row.getX();
				matrixData[i][1] = row.getY();
				matrixData[i][2] = row.getZ();
				rhs[i] = pedestal._localCoordinates.getInnerProduct(row);
				//perturbed by biases pedestal, estimated solution matrix
				biasedLocal.set(pedestal.getBiasedLocal());
				rowBiased.set(new Rotator(pedestal._localFrame._local).rotate_k(biasedLocal.getAzimuth().codedPhase())
						.rotate_j(biasedLocal.getElevation().codedPhase()).getImage_k().unit());
				matrixDataBiased[i][0] = rowBiased.getX();
				matrixDataBiased[i][1] = rowBiased.getY();
				matrixDataBiased[i][2] = rowBiased.getZ();
				rhsBiased[i] = pedestal._localCoordinates.getInnerProduct(rowBiased);
				
				i += 1;
				iEl += 1;
			}
			if (pedestal.getMapAZ()) {
				//reference pedestal solution matrix
				row = pedestal._apertureFrame._orientation.getImage_j().unit();// Horz: axial EL dircos
				matrixData[i][0] = row.getX();
				matrixData[i][1] = row.getY();
				matrixData[i][2] = row.getZ();
				rhs[i] = pedestal._localCoordinates.getInnerProduct(row);
				//perturbed by biases, estimated pedestal solution matrix
				biasedLocal.set(pedestal.getBiasedLocal());
				rowBiased.set(new Rotator(pedestal._localFrame._local).rotate_k(biasedLocal.getAzimuth().codedPhase())
						.rotate_j(biasedLocal.getElevation().codedPhase()).getImage_k().unit());
				matrixDataBiased[i][0] = rowBiased.getX();
				matrixDataBiased[i][1] = rowBiased.getY();
				matrixDataBiased[i][2] = rowBiased.getZ();
				rhsBiased[i] = pedestal._localCoordinates.getInnerProduct(rowBiased);
								
				i += 1;
				iAz += 1;
			}
		}
		
		
		RealMatrix a = new Array2DRowRealMatrix(matrixData);
	System.out.println("Pedestals in solution: "+pedSensorCnt/2);
	System.out.println("Sensors in solution: "+iAz+" AZ + "+iEl+" EL = "+i+" Total.");
	System.out.println(a.getColumnDimension()); // 3
		SingularValueDecomposition svd = new SingularValueDecomposition(a.getSubMatrix(0,i-1,0, a.getColumnDimension()-1));
		RealVector b = new ArrayRealVector(rhs);
		RealVector y = svd.getSolver().solve(b.getSubVector(0, i));
		
		RealMatrix aPerturbed = new Array2DRowRealMatrix(matrixDataBiased);
	System.out.println("Pedestals in solution: "+pedSensorCnt/2);
	System.out.println("Sensors in solution: "+iAz+" AZ + "+iEl+" EL = "+i+" Total.");
	System.out.println(a.getColumnDimension()); // 3
		SingularValueDecomposition svdPerturbed = new SingularValueDecomposition(aPerturbed.getSubMatrix(0,i-1,0, aPerturbed.getColumnDimension()-1));
		RealVector bPerturbed = new ArrayRealVector(rhsBiased);
		RealVector yPerturbed = svdPerturbed.getSolver().solve(bPerturbed.getSubVector(0, i));
		
		RealMatrix covariance = svd.getCovariance(1e-9);
		RealMatrix vSVD = svd.getV();
		RealMatrix utSVD = svd.getUT();
		RealMatrix sSVD = svd.getS();
		double normSVD = svd.getNorm();
		int rankSVD = svd.getRank();

//		OLSMultipleLinearRegression fit = new OLSMultipleLinearRegression();
//		fit.newSampleData(rhs, matrixData);
//		fit.setNoIntercept(true);
//		
//        double[] parm = fit.estimateRegressionParameters(); //y
//        
//		double R2Adjusted = fit.calculateAdjustedRSquared();
//		double mse = fit.estimateErrorVariance();
//		double[] parmSTD = fit.estimateRegressionParametersStandardErrors();
			
		//angles only solution
		Vector3 vector_EFG = new Vector3(y.getEntry(0), y.getEntry(1), y.getEntry(2));
		Polar polar_EFG = Polar.commandLocal(vector_EFG, Pedestal.getOriginFrame()._local);
		
		Vector3 vectorBiased_EFG = new Vector3(yPerturbed.getEntry(0), yPerturbed.getEntry(1), yPerturbed.getEntry(2));
		Polar polarBiased_EFG = Polar.commandLocal(vectorBiased_EFG, Pedestal.getOriginFrame()._local);
		
		this.delta_RAE.set(polarBiased_EFG.getRange()-polar_EFG.getRange() 
				,polarBiased_EFG.getSignedAzimuth().subtract(polar_EFG.getSignedAzimuth()).signedPrinciple()
				,polarBiased_EFG.getElevation().subtract(polar_EFG.getElevation()).signedPrinciple()
				);
		this.delta_EFG.set( new Vector3(vectorBiased_EFG).subtract(vector_EFG));
		this.position_EFG.set( new Vector3(vector_EFG).add(Pedestal.getOrigin()));

		this.condition = svd.getConditionNumber();
				
	System.out.println("Condition number : "+formatted(this.condition,5));
	System.out.println( this.position_EFG.toString(5) );
	}
	
	
	
	
	
	public void measureError(Vector3 truth) {
		Vector3 targetPrime;
		if( truth==null )
			targetPrime = new Vector3(0.0,0.0,0.0);
		else targetPrime = new Vector3(this.position_EFG);
		targetPrime.subtract( truth );
		this.error = targetPrime.getAbs();
	}
	
	public String formatted(double value, int decimals){
		String fmt = " %."+decimals+"f ";
		//System.out.println("FORMAT STRING = "+fmt);
		return String.format(fmt, value);
	}
	
}
