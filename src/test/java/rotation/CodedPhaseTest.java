package rotation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
//import static org.junit.Assert.fail;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import rotation.Angle;
import rotation.CodedPhase;

public class CodedPhaseTest {

	@Test
	public void main(){	
	double i;
	Angle a = Angle.inDegrees(Double.NaN);
	for (int h = -32; h <= 32; h++) {
		i=h*22.5;
		a = Angle.inDegrees(i);
		CodedPhase c = new CodedPhase(a.codedPhase());
//		System.out.println("("+i+")"
//		        +" Unsigned Angle = "+a.unsignedPrincipleComponent().toDegreesString(17)
//				+"    Signed Angle = "+a.signedPrincipleComponent().toDegreesString(17)
//				+" near rev = "+StrictMath.round(Angle.inDegrees(i).getRevolutions())
//				);			

//		System.out.println("("+i+")"
//		        +" Unsigned Principle Angle = "+c.angle().signedPrinciple().unsignedPrinciple().toDegreesString(17)
//				+"    Signed Principle Angle = "+c.angle().signedPrinciple().signedPrinciple().toDegreesString(17)
//				+" near rev = "+StrictMath.round(Angle.inDegrees(i).getRevolutions())
//				);			
	}
	System.out.println("********End of AngleTest exercises *******************");

	}
	
	
	@Test
	public void testAdd() {
		Angle a = Angle.inDegrees(0);
		Angle b = Angle.inDegrees(30);
		CodedPhase p1 = new CodedPhase(a.codedPhase());
		CodedPhase p2 = b.codedPhase();
		
		p1 = new CodedPhase(a.codedPhase());
		p2 = b.codedPhase();
		
        //System.out.println(p1.signedAngle().getDegrees()-0d);
		assertEquals(p2.angle().signedPrinciple().getDegrees(),30d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()-30d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),30d,1e-13);
       
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()-60d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),60d,1e-13);
       
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()-90d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),90d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()-120d);
		assertEquals(p1.angle().getDegrees(),120d,1e-13);
        
        p1.add(p2).angle();


        //System.out.println(p1.angle().getDegrees());
		//System.out.println(p1.sin());
		//System.out.println(p1.cos());
        //System.out.println(p2.angle().getDegrees());
		//System.out.println(p2.sin());
		//System.out.println(p2.cos());
        assertEquals(p1.angle().getDegrees(),150d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.angle().getDegrees()-180d);
		//assertEquals(p1.angle().signedPrinciple().getDegrees(),-180d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+150d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-150d,1e-13);
       
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+120d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-120d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+90d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-90d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+60d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-60d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+30d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-30d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+0d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),0d,1e-13);
        
		p1.add(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()-30d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),30d,1e-13);

		double n=0d;
		a = Angle.inRadians(n);
		p1 =  a.codedPhase();
		b = Angle.inRadians(StrictMath.atan(1)*(30d/45d));
		p2 =  b.codedPhase();
		for (int i = 0; i < 20; i++) {
			a = Angle.inRadians(n);
		    //System.out.println("Summed Signed Principle Angle: " + a.getRadians());
		    //System.out.println("Summed Signed Principle Angle: " + a.signedPrinciple().getRadians());
		    //System.out.println("Signed Principle Angle: " + p1.angle().signedPrinciple().getRadians());
		    //System.out.println("Signed Principle Angle: " + p1.angle().signedPrinciple().getRadians());
		    //System.out.println("Straight Principle Angle: " + Angle.STRAIGHT.signedPrinciple().getRadians());
            //(a.codedPhase().angle()) 
		    if(0==i%6){
		        assertEquals(StrictMath.abs(p1.angle().signedPrinciple().getRadians()), StrictMath.abs(a.codedPhase().angle().signedPrinciple().getRadians()) ,1e-13);
		    } else {
			    assertEquals(p1.angle().signedPrinciple().getRadians(), a.codedPhase().angle().signedPrinciple().getRadians() ,1e-13);
		    }
		    //fails for roundoff...:assertEquals(p1.angle().signedPrinciple().getRadians(), a.signedPrinciple().getRadians() ,1e-13);
			n += b.getRadians();
			p1.add(p2);
		}
		
		//test Angle and CodedPhase casts and additions with clock arithmetic.
		Angle c = Angle.inPiRadians(0);
		CodedPhase p3 = c.codedPhase();
		for (int i = -8; i < 8; i++) {
			
			a = Angle.inPiRadians(i/4d);
			p1 = a.codedPhase();
			
			for (int j = -12; j < 12; j++) {
				
				b = Angle.inPiRadians(i/6d);
				p2 = b.codedPhase();
				
				c = new Angle(a).add(b);				
				p3 = new CodedPhase(p1).add(p2);
				
			    assertEquals(c.codedPhase().angle().getPiRadians(), p3.angle().getPiRadians() ,1e-15);
			}
		}
		
		
		//test add right orthogonal with clock arithmetic...
		b = Angle.inPiRadians(1/2d);
		p2 = b.codedPhase();			
		for (int i = -12; i < 12; i++) {
			a = Angle.inPiRadians(i/12d);
			p1 = a.codedPhase();
				c = new Angle(a).add(b);
				p3 = new CodedPhase(p1).addRight();
			    assertEquals(c.codedPhase().angle().getPiRadians(), p3.angle().getPiRadians() ,1e-15);
		}
		
		//test subtract right orthogonal with clock arithmetic...
		b = Angle.inPiRadians(1/2d);
		//System.out.println("Signed PiRad Angle b: " + b.getDegrees());
		p2 = b.codedPhase();			
		for (int i = -12; i < 12; i++) {
			a = Angle.inPiRadians(i/12d); 
			c = Angle.inPiRadians((i-6)/12d);
			p1 = a.codedPhase();
			p3 = new CodedPhase(p1).subtractRight();
			//System.out.println("Angle      p1: " + p1.angle().getDegrees());
			//System.out.println("CodedAngle p3: " + p3.angle().getDegrees());
			    if(Double.isInfinite(p3.tanHalf())){
					//System.out.println("Signed PiRad Angle c: " + c.codedPhase().angle().getPiRadians() + "  " + i);
					//System.out.println("PiRad Angle p3 raw: " + p3.tanHalf() + "  " + i);
					//System.out.println("Angle      p1: " + p1.angle().getDegrees());
					//System.out.println("CodedAngle p3: " + p3.angle().getDegrees());
				assertEquals(c.codedPhase().angle().negate().getPiRadians(), p3.angle().getPiRadians() ,1e-15);
			    }else{
					assertEquals(c.codedPhase().angle().getPiRadians(), p3.angle().getPiRadians() ,1e-15);			    	
			    }
		}
		
		//test add straight with clock arithmetic...
		b = Angle.inPiRadians(1d);
		p2 = b.codedPhase();			
		for (int i = -12; i < 12; i++) {
			a = Angle.inPiRadians(i/12d);
			p1 = a.codedPhase();
				c = new Angle(a).add(b);
				p3 = new CodedPhase(p1).addStraight();
				assertEquals(c.codedPhase().angle().signedPrinciple().getPiRadians(), p3.angle().signedPrinciple().getPiRadians() ,1e-14);
		}
		
		b = Angle.inPiRadians(1.0e-7); //near zero positive number...
		p2 = b.codedPhase();
		p1 = new CodedPhase(p2).addStraight();
			assertEquals(p1.angle().getPiRadians(),-1d,1e-7); //near opposite within perturbation...

	}


	@Test
	public void testCos() {
		Angle t=new Angle(Angle.EMPTY); 
		CodedPhase w = new CodedPhase(CodedPhase.EMPTY);
		for (int i = -360; i < 361; i++) {
			t.setPiRadians(i / 360);
			w.set(t.codedPhase());
			if (StrictMath.abs(t.getPiRadians()) < 1) {
				assertEquals(StrictMath.cos(t.getRadians()), w.cos(), 0);
			} else {
				//System.out.println();
				//System.out.println();
				//System.out.println(w.angle().getPiRadians());				
				//System.out.println(t.getPiRadians());
				
				//System.out.println();				
				//System.out.println(StrictMath.cos(t.getRadians()));
				//System.out.println(w.tanHalf());
				//System.out.println(w.cos());
				assertEquals(StrictMath.cos(t.getRadians()), w.cos(), 0);
			}
		}
		
		Angle a = Angle.inDegrees(0);
		CodedPhase p = new CodedPhase(a.codedPhase());
		
		//System.out.println(p.tan());
		
		//System.out.println(p.cos());
		assertEquals(p.cos(),1,1e-15);
		
		a.setDegrees(60d);
		p = a.codedPhase();
		//System.out.println(p.cos()+ " Cosine of 60 degrees: "+ StrictMath.cos(StrictMath.toRadians(60)));
		assertEquals(p.cos(),.5d,1e-14);
		
		a.setDegrees(90d);
		p = a.codedPhase();
		//System.out.println(p.angle().getDegrees());
		//System.out.println(p.cos());
		assertEquals(p.cos(),0d,1e-15);
		
		a.setDegrees(180d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),-1d,1e-15);
			
		a.setDegrees(-60d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),.5d,1e-15);
		
		a.setDegrees(-90d);
		p = a.codedPhase();
		//System.out.println(p.cos()+" Cosine -90 degrees: " + StrictMath.cos(StrictMath.toRadians(-90)));
		assertEquals(p.cos(),0d,1e-15);
		
		a.setDegrees(-180d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),-1d,1e-15);
		
		//a.setDegrees(120d);
		a.setRadians(StrictMath.PI*(2d/3));
		p = a.codedPhase();
		//System.out.println(a.getDegrees() + " Degrees. "+ p.signedAngle().getDegrees());
		//System.out.println(p.cos());
		assertEquals(p.cos(),-.5d,1e-13);
		
		a.setDegrees(-120d);
		p = a.codedPhase();
		//System.out.println(p.cos()+ " of Degrees. "+ p.unsignedAngle().getDegrees());
		assertEquals(p.cos(),-.5d,1e-15);			
		
		a.setDegrees(240d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),-.5d,1e-15);

		a.setDegrees(60d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),.5d,1e-15);
		
		a.setDegrees(-60d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),.5d,1e-15);
		
		a.setDegrees(300d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),.5d,1e-15);
		
		a.setDegrees(300d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),.5d,1e-15);
		
		a.setDegrees(89.99999999999999d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),StrictMath.cos(StrictMath.toRadians(89.99999999999999d)),1e-15);

		a.setDegrees(90d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),StrictMath.cos(StrictMath.toRadians(90.0d)),1e-15);
		
		a.setDegrees(90.00000000000001d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),StrictMath.cos(StrictMath.toRadians(90.00000000000001d)),1e-15);
		
		a.setDegrees( 0.000001d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),StrictMath.cos(StrictMath.toRadians(0.000001d)),1e-15);

		a.setDegrees(-0.000001d);
		p = a.codedPhase();
		//System.out.println(p.cos());
		assertEquals(p.cos(),StrictMath.cos(StrictMath.toRadians(-0.000001d)),1e-15);

		
	}


	@Test
	public void testGetSetAnglePrinciple() {
		Angle a = Angle.inDegrees(30);
		Angle b = Angle.inDegrees(60);

		CodedPhase p = new CodedPhase(a.codedPhase());
		assertEquals(p.angle().signedPrinciple().getDegrees(),30,1e-14);

		CodedPhase t = new CodedPhase(p);
		assertEquals(t.angle().signedPrinciple().getDegrees(),30,1e-14);

		p = b.codedPhase();  //angle
		assertEquals(p.angle().signedPrinciple().getDegrees(),60,1e-14);

		p.set(t); //principle angle	
		assertEquals(p.angle().signedPrinciple().getDegrees(),30,1e-14);

	  //check factory casts back to Angle class
		

		CodedPhase c = Angle.inPiRadians(2).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),0,0);
		c.equals(Angle.inPiRadians(2).codedPhase());
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),0,0);
		c.equals(Angle.inPiRadians(2).codedPhase());
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(-0d) );

		c = Angle.inPiRadians(-2).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(-0d) );
		
		c = Angle.inPiRadians(0).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(-0d) );

		c = Angle.inPiRadians(-0).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),0,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),0,0);
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==0d );
		assertTrue( c.angle().unsignedPrinciple().getPiRadians()==-0d );
		assertTrue( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(0d) );
		assertFalse( Double.valueOf(c.angle().unsignedPrinciple().getPiRadians()).equals(-0d) );

		c = Angle.inPiRadians(-1).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),-1,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),1,0);
		
		c = Angle.inPiRadians(1).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),-1,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),1,0);
		
		c = Angle.inPiRadians(-3).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),-1,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),1,0);
		
		c = Angle.inPiRadians(3).codedPhase();
		assertEquals(c.angle().signedPrinciple().getPiRadians(),-1,0);
		assertEquals(c.angle().unsignedPrinciple().getPiRadians(),1,0);
		
		
		t = Angle.inDegrees(-30).codedPhase(); 
		assertEquals(t.angle().signedPrinciple().getDegrees(),-30,1e-14);
		assertEquals(t.angle().unsignedPrinciple().getDegrees(),330,1e-13);

		t = Angle.inDegrees(-150).codedPhase(); 
		assertEquals(t.angle().signedPrinciple().getDegrees(),-150,1e-13);
		assertEquals(t.angle().unsignedPrinciple().getDegrees(),210,1e-13);

		t = Angle.inDegrees(-180).codedPhase(); 
    	assertEquals(t.angle().signedPrinciple().getDegrees(),-180,1e-13);
    	//System.out.println("-180 degrees"+Angle.inPiRadians(t.getPiRadians()).getCodedPrincipleDouble());
		assertEquals(t.angle().signedPrinciple().getDegrees(),-180,1e-13);
		assertEquals(t.angle().signedPrinciple().signedPrinciple().getDegrees(),-180,1e-13);
		assertEquals(t.angle().unsignedPrinciple().getDegrees(),180,1e-13);

		t = Angle.inDegrees(180).codedPhase(); 
    	assertEquals(t.angle().signedPrinciple().getDegrees(),-180,1e-13);
    	//System.out.println("+180 degrees"+Angle.inPiRadians(t.getPiRadians()).getCodedPrincipleDouble());
		assertEquals(t.angle().signedPrinciple().getDegrees(), -180,1e-13);
		assertEquals(t.angle().unsignedPrinciple().getDegrees(),180,1e-13);
		
		Angle aa = Angle.inDegrees(180);
		//System.out.println( aa.getPiRadians() );
		//System.out.println( aa.signedPrinciple().getPiRadians() );
		//System.out.println( aa.unsignedPrinciple().getPiRadians() );
		//System.out.println( aa.codedPhase().tanHalf() );
	
		aa = Angle.inDegrees(-180);
		//System.out.println( aa.getPiRadians() );
		//System.out.println( aa.signedPrinciple().getPiRadians() );
		//System.out.println( aa.unsignedPrinciple().getPiRadians() );
		//System.out.println( aa.codedPhase().tanHalf() );
	
		aa = Angle.inDegrees(360);
		//System.out.println( aa.getPiRadians() );
		//System.out.println( aa.signedPrinciple().getPiRadians() );
		//System.out.println( aa.unsignedPrinciple().getPiRadians() );
		//System.out.println( aa.codedPhase().tanHalf() );
	
		aa = Angle.inDegrees(-360);
		//System.out.println( aa.getPiRadians() );
		//System.out.println( aa.signedPrinciple().getPiRadians() );
		//System.out.println( aa.unsignedPrinciple().getPiRadians() );
		//System.out.println( aa.codedPhase().tanHalf() );
	
		t = Angle.inDegrees(-0).codedPhase();
    	assertEquals(t.angle().signedPrinciple().getDegrees(),0,1e-13);
		assertEquals(t.angle().signedPrinciple().getDegrees(),0,1e-13);

		t = Angle.inDegrees(0).codedPhase();
    	assertEquals(t.angle().signedPrinciple().getDegrees(),0,1e-13);
		assertEquals(t.angle().signedPrinciple().getDegrees(),0,1e-13);
	
	}

	@Test
	public void testIsAcute() {
		Angle angle = Angle.inRadians(Double.MIN_VALUE);
		CodedPhase test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); //lost last sig bit of Radians

		//angle.set(Angle.inRadians(StrictMath.scalb(StrictMath.PI,-1)));
		angle.set(Angle.inPiRadians(1/2d));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(StrictMath.nextUp(StrictMath.scalb(StrictMath.PI,-1))));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(-StrictMath.scalb(StrictMath.PI,-1)));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(-StrictMath.nextUp(StrictMath.scalb(StrictMath.PI,-1))));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 


		
		angle.set(Angle.inRadians(7));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

		angle.set(Angle.inRadians(6));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

		angle.set(Angle.inRadians(5));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

		angle.set(Angle.inRadians(4));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(3));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(2));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(1));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

		angle.set(Angle.inRadians(-1));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

		angle.set(Angle.inRadians(-2));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		
		angle.set(Angle.inRadians(-3));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 

		angle.set(Angle.inRadians(-4));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.FALSE); 
		
		angle.set(Angle.inRadians(-5));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 
		
		angle.set(Angle.inRadians(-6));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

		angle.set(Angle.inRadians(-7));
		test= angle.codedPhase();
		assertEquals(test.isAcute(),Boolean.TRUE); 

	}
		
		
		
	

	@Test
	public void testIsEqualTo() {
		Angle angle = Angle.inRadians(3);
		CodedPhase test=new CodedPhase(angle.codedPhase());
		CodedPhase tryit = new CodedPhase(test);
		CodedPhase tol = new CodedPhase(CodedPhase.ZERO);
		CodedPhase e = new CodedPhase(CodedPhase.EMPTY);
		
		assertEquals(tol.codedMagnitude(),0d,1e-15);
		
		assertTrue(new CodedPhase(CodedPhase.EMPTY).isEmpty());
		
		assertFalse(test.isEmpty());
		
		assertFalse(tol.equals(test.isEmpty()));
		assertTrue(e.equals(CodedPhase.EMPTY));
		
		double test1 = tol.magnificationRotator();
		double test2 = test.magnificationRotator();
		assertEquals(test1,1,1e-15);

		
	}

	@Test
	public void testIsPositive() {
		Angle angle = Angle.inRadians(Double.MIN_VALUE);
		CodedPhase test= angle.codedPhase();
		assertEquals(test.isPositive(),Boolean.TRUE); //lost last sig bit of Radians
	
		//show zero case
		angle.set(Angle.inRadians(0.0));		
		test= angle.codedPhase();
		assertEquals(test.isPositive(),Boolean.TRUE);

		//show negative zero becomes zero case
		angle.set(Angle.inRadians(-0.0));		
		test= angle.codedPhase();
		assertEquals(test.isPositive(),Boolean.TRUE);

		
		//show greatest value less than negative-zero computes as zero by loss of bit representation...	
		angle.setRadians(-Double.MIN_VALUE);
		test= angle.codedPhase();
		assertEquals(test.isPositive(),Boolean.TRUE); //lost last sig bit of Radians
	
		//show value largest value not to test positive...	
		angle.setPiRadians(-2*Double.MIN_VALUE);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.FALSE); //neg last sig bit of Radians now visible.
		assertEquals(test.isPositive(),Boolean.FALSE); //neg last sig bit of Radians now visible.
	
		
	}

	@Test
	public void testIsRight() {
		CodedPhase test=CodedPhase.RIGHT; //new Principle(angle).setRight();
		assertEquals(test.isAcute(),Boolean.FALSE); 
		assertEquals(test.isOrthogonal(),Boolean.TRUE); 
        test.trueRound();
		assertEquals(test.isOrthogonal(),Boolean.TRUE);
		
		test=Angle.inPiRadians(StrictMath.scalb(1, -27)).add(Angle.RIGHT).codedPhase(); //new Principle(angle).setRight();
		assertEquals(test.isOrthogonal(),Boolean.FALSE); 
        test.trueRound();
		assertEquals(test.isOrthogonal(),Boolean.FALSE);
		

		test=Angle.inPiRadians(StrictMath.scalb(1, -28)).negate().add(Angle.RIGHT).codedPhase(); //new Principle(angle).setRight();
		assertEquals(test.isOrthogonal(),Boolean.FALSE); 
        test.trueRound();
		assertEquals(test.isOrthogonal(),Boolean.TRUE);
		
		
		test.negate();
		assertEquals(test.isAcute(),Boolean.FALSE); 
		assertEquals(test.isObtuse(),Boolean.FALSE); 
		assertEquals(test.isOrthogonal(),Boolean.TRUE); 
        test.trueRound();
		assertEquals(test.isOrthogonal(),Boolean.TRUE);
		
		
		assertEquals(new CodedPhase(test).add(test).isObtuse(),Boolean.TRUE); 
		
	}

	@Test
	public void testIsStraight() {
		CodedPhase test=CodedPhase.STRAIGHT; //new Principle(angle).setStraight();
		assertEquals(test.isAcute(),Boolean.FALSE); 
		assertEquals(test.isStraight(),Boolean.TRUE); 
		assertEquals(test.isObtuse(),Boolean.TRUE); 
		test.negate();
		assertEquals(test.isAcute(),Boolean.FALSE); 
		assertEquals(test.isStraight(),Boolean.TRUE); 
		assertEquals(test.isObtuse(),Boolean.TRUE);
		
		test= Angle.inPiRadians(1e-9).add(Angle.STRAIGHT).codedPhase(); //new Principle(angle).setRight();
		//assertEquals(test.isStraight(),Boolean.FALSE); 
        test.trueRound();
        System.out.println(test.tanHalf());
		assertEquals(test.isStraight(),Boolean.TRUE);

		
		
		
	}


	@Test
	public void testIsZero() {
		Angle angle = Angle.inDegrees(0.0);		
		CodedPhase test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.TRUE);
        test.trueRound();
		assertEquals(test.isZero(),Boolean.TRUE);

		
		//show min possible representation lost a bit...
	
		angle.setPiRadians(-2*Double.MIN_VALUE);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.FALSE);
        test.trueRound();
		assertEquals(test.isZero(),Boolean.TRUE);
		
		angle.setPiRadians(Double.MIN_VALUE / 2);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.TRUE);
		
		angle.setRadians(-Double.MIN_VALUE);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.TRUE); //lost last sig bit of Radians
	
		angle.setRadians(Double.MIN_VALUE);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.TRUE); //lost last sig bit of Radians
	
		angle.setPiRadians(2*Double.MIN_VALUE);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.FALSE);
		
		angle.setPiRadians(2*StrictMath.scalb(1d, -27));
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.FALSE);
				
		angle.setPiRadians(Double.MIN_NORMAL);
		test= angle.codedPhase();
		assertEquals(test.isZero(),Boolean.FALSE);		
		
	}

	@Test
	public void testNegate() {
		Angle a = Angle.inDegrees(30);
		CodedPhase p = new CodedPhase(a.codedPhase());
        p.negate();
        //System.out.println(p.toSignedAngle().getDegrees());
		assertEquals(p.angle().signedPrinciple().getDegrees(),-30d,1e-13);
		assertEquals(p.negate().angle().signedPrinciple().getDegrees(),30d,1e-13);

	}

	@Test
	public void testSin() {
		
		Angle t=new Angle(Angle.EMPTY); 
		CodedPhase w = new CodedPhase(CodedPhase.EMPTY);
		for (int i = -360; i < 361; i++) {
			t.setPiRadians(i / 360);
			w.set(t.codedPhase());
			if (StrictMath.abs(t.getPiRadians()) < 1) {
				assertEquals(StrictMath.sin(t.getRadians()), w.sin(), 0);
			} else {
				assertEquals(StrictMath.sin(t.getRadians()), w.sin(), 1e-15);
				//System.out.println(t.getPiRadians()+" PiRad  " + t.getDegrees()+" deg  "+w.sin()+"  "+StrictMath.sin(t.getRadians())+"  "+StrictMath.sin(StrictMath.PI)+"  "+StrictMath.scalb(2d,-53));
				assertEquals(w.sin(), 0d, 0);
				assertTrue(StrictMath.abs(StrictMath.sin(t.getRadians())) < StrictMath.scalb(2d,-53));
			}
		}
		
		Angle a = Angle.inDegrees(0);
		CodedPhase p = new CodedPhase(a.codedPhase());
		
		//System.out.println(p.tan());
		assertEquals(p.sin(),0,1e-15);
		
		a.setDegrees(15d);
		p = a.codedPhase();
		//System.out.println(" Sin of 15 degrees: "+p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(15)),1e-15);
		

		a.setDegrees(30d);
		p = a.codedPhase();
		//System.out.println(p.signedAngle().getDegrees());
		//System.out.println(p.sin());
		assertEquals(p.sin(),.5d,1e-15);
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(30)),1e-15);
		
		a.setDegrees(90d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),1d,1e-15);
		
		a.setDegrees(180d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),0d,1e-15);
			
		a.setDegrees(-30d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),-.5d,1e-15);
		
		a.setDegrees(-90d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),-1d,1e-15);
		
		a.setDegrees(-180d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),0d,1e-15);
		
		a.setDegrees(150d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),.5d,1e-15);
		
		a.setDegrees(210d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),-.5d,1e-15);
		
		a.setDegrees(0.1d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(.1)),1e-15);

		a.setDegrees(0.0d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(0)),1e-15);
		
		a.setDegrees(-0.1d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(-.1)),1e-15);

		a.setDegrees(90.0001d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(90.0001)),1e-15);

		a.setDegrees(90.0d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(90)),1e-15);
	
		a.setDegrees(89.9999d);
		p = a.codedPhase();
		//System.out.println(p.sin());
		assertEquals(p.sin(),StrictMath.sin(StrictMath.toRadians(89.9999d)),1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
	}

	@Test
	public void testSubtract() {
		Angle a = Angle.inDegrees(0);
		Angle b = Angle.inDegrees(-30);
		CodedPhase p1 =  a.codedPhase();
		CodedPhase p2 = b.codedPhase();
		
        //System.out.println(p1.signedAngle().getDegrees()-0d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),0d,1e-13);
        
		p1.subtract(p2).angle().signedPrinciple();		
        //System.out.println(p1.signedAngle().getDegrees()-30d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),30d,1e-13);
        
		p1.subtract(p2).angle().signedPrinciple();		
        //System.out.println(p1.signedAngle().getDegrees()-60d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),60d,1e-13);
       
		p1.subtract(p2).angle().signedPrinciple();		
        //System.out.println(p1.signedAngle().getDegrees()-90d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),90d,1e-13);
      
		p1.subtract(p2).angle().signedPrinciple();		
        //System.out.println(p1.signedAngle().getDegrees()-120d);	
		assertEquals(p1.angle().signedPrinciple().getDegrees(),120d,1e-13);

        
        p1.subtract(p2).angle().signedPrinciple();        
        //System.out.println(p1.signedAngle().getDegrees()-150d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),150d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();	
        //System.out.println(p1.signedAngle().getDegrees() - 180d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),180d,1e-13);

		//System.out.println(p1.angle().getDegrees());
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+150d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-150d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+120d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-120d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+90d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-90d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+60d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-60d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+30d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),-30d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()+0d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),0d,1e-13);

        
		p1.subtract(p2).angle().signedPrinciple();
        //System.out.println(p1.signedAngle().getDegrees()-30d);
		assertEquals(p1.angle().signedPrinciple().getDegrees(),30d,1e-13);

		double n=StrictMath.PI*2d;
		a = Angle.inRadians(n);
		p1 =  a.codedPhase();
		b = Angle.inRadians(StrictMath.PI*(30d/45d));
		p2 =  b.codedPhase();
		for (int i = 0; i < 12; i++) {
			n -= b.getRadians();
			p1.subtract(p2);
			a = Angle.inRadians(n);
		    //System.out.println("Principle Angle: " + a.getRadians());
		    assertEquals(p1.angle().signedPrinciple().getRadians(), a.signedPrinciple().getRadians() ,1e-13);
		}
		
		//test add straight with clock arithmetic...
		b = Angle.inPiRadians(1d);
		Angle c = new Angle(b);
		p2 = b.codedPhase();
		CodedPhase p3 = new CodedPhase(p2);
		for (int i = -12; i < 12; i++) {
			a = Angle.inPiRadians(i/12d);
			p1 = a.codedPhase();
				c = new Angle(a).add(b);
				p3 = new CodedPhase(p1).subtractStraight();
				assertEquals(c.codedPhase().angle().signedPrinciple().getPiRadians(), p3.angle().signedPrinciple().getPiRadians() ,1e-14);
		}

		
		
		
		
		p1 = Angle.inDegrees(359.9).codedPhase();
		p2 = Angle.inDegrees(0.1).codedPhase();
		//System.out.println("Principle Angle Diff: " + (new Principle(p1).subtract(p2).abs()).signedAngle().getDegrees());
		//System.out.println("Principle Angle Diff: " + (new Principle(p2).subtract(p1).abs()).signedAngle().getDegrees());
	    assertEquals(-0.2d, new CodedPhase(p1).subtract(p2).angle().getDegrees() ,1e-13);
		p1 = Angle.inDegrees(179.85).codedPhase();
		p2 = Angle.inDegrees(180.15).codedPhase();
		//System.out.println("Principle Angle Diff: " + (new Principle(p1).subtract(p2).abs()).signedAngle().getDegrees());
		//System.out.println("Principle Angle Diff: " + (new Principle(p2).subtract(p1).abs()).signedAngle().getDegrees());
		assertEquals(-.3d, new CodedPhase(p1).subtract(p2).angle().getDegrees() ,1e-13);
		p1 = Angle.inDegrees(-0.1).codedPhase();
		p2 = Angle.inDegrees(0.1).codedPhase();
		//System.out.println("Principle Angle Diff: " + (new Principle(p1).subtract(p2).abs()	t = AsignedAngle().getDegrees());
		//System.out.println("Principle Angle Diff: " + (new Principle(p2).subtract(p1).abs()).signedAngle().getDegrees());
	    assertEquals(-0.2d, new CodedPhase(p1).subtract(p2).angle().getDegrees() ,1e-15);
		
	}

	@Test
	public void testTan() {
		
		Angle t=new Angle(Angle.EMPTY); 
		CodedPhase w = new CodedPhase(CodedPhase.EMPTY);
		for (int i = -360; i < 360; i++) {
			t.setPiRadians(i/360d); 
			w.set(t.codedPhase());
			if (w.isOrthogonal()) {
				//System.out.println("angle is: "+t.toDegreesString(16));
				//System.out.println("   StrictMath tan is: "+StrictMath.tan(t.getRadians()));
				//System.out.println("   coded tan is     : "+w.tan());
                assertTrue(StrictMath.abs(w.tan())>1e20);
				//System.out.println(" strict math tan(90) is      : "+StrictMath.tan(StrictMath.PI/2));
				//System.out.println(" strict math tan(270) is     : "+StrictMath.tan(3*StrictMath.PI/2));
				if (w.isPositive()) {
					assertTrue((w.tan() == Double.POSITIVE_INFINITY));
				} else {
					assertTrue((w.tan() == Double.NEGATIVE_INFINITY));
				}
			} else if(w.isStraight()){
				//System.out.println("angle is: "+t.toDegreesString(16));
				//System.out.println("   StrictMath tan is: "+StrictMath.tan(t.getRadians()));
				//System.out.println("   coded tan is     : "+w.tan());
				//System.out.println(" strict math tan(180) is      : "+StrictMath.tan(StrictMath.PI));
	            assertEquals(w.tan(),0d,0d);
			} else {
				if (StrictMath.abs(t.getDegrees()) < 90) {
					if (StrictMath.abs(t.getDegrees()) < .5) { // make larger limits...
						//System.out.println("angle is: "+t.toDegreesString(16));
						//System.out.println("   StrictMath tan is: " + StrictMath.tan(t.getRadians()));
						//System.out.println("   coded tan is     : " + w.tan());
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 0);
					} else if (StrictMath.abs(t.getDegrees()) <= 1.5) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-17);
					} else if (StrictMath.abs(t.getDegrees()) <= 29) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-16); //16
					} else if (StrictMath.abs(t.getDegrees()) <= 45) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-15);
					} else if (StrictMath.abs(t.getDegrees()) <= 86) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-14); //14
					} else if (StrictMath.abs(t.getDegrees()) <= 88.9) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-13);
					} else {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-12);
					}
					assertEquals(t.getPiRadians(), w.angle().getPiRadians(), 1e-16);
					
				} else { // make smaller limits...
					
					if (StrictMath.abs(t.getDegrees()) >= 177.1) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 0); //0
					} else if (StrictMath.abs(t.getDegrees()) >= 175.9) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-17); //17
					} else if (StrictMath.abs(t.getDegrees()) > 152.5) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-16);
					} else if (StrictMath.abs(t.getDegrees()) >= 108.1) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-15);
					} else if (StrictMath.abs(t.getDegrees()) >= 93.1) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-14); //14
					} else if (StrictMath.abs(t.getDegrees()) >= 91.6) {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-13);
					} else {
						assertEquals(StrictMath.tan(t.getRadians()), w.tan(), 1e-12);
					}
					assertEquals(t.getPiRadians(), w.angle().getPiRadians(), 1e-15);
				}
			}
		}

		Angle a = Angle.inDegrees(89.9);
		CodedPhase p = new CodedPhase(a.codedPhase());
		//System.out.println("tan 89.9 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(89.9)));
		assertEquals(StrictMath.tan(a.getRadians()), p.tan(), 1e-10); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 0); //0
		
		a = Angle.inDegrees(89.99);
		p = new CodedPhase(a.codedPhase());
		//System.out.println("tan 89.99 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(89.99)));
		assertEquals(StrictMath.tan(a.getRadians()), p.tan(), 1e-7); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 1e-16); //0

		a = Angle.inDegrees(89.999);
		p = new CodedPhase(a.codedPhase());
		//System.out.println("tan 89.999 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(89.999)));
		assertEquals(StrictMath.tan(a.getRadians()), p.tan(), 1e-6); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 0); //0

		a = Angle.inDegrees(89.9999);
		p = new CodedPhase(a.codedPhase());
		//System.out.println("tan 89.9999 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(89.9999)));
		assertEquals(StrictMath.tan(a.getRadians()), p.tan(), 1e-3); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 0); //0

		a = Angle.inDegrees(89.99999);
		p = new CodedPhase(a.codedPhase());
		//System.out.println("tan 89.99999 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(89.99999)));
		assertEquals(StrictMath.tan(a.getRadians()), p.tan(), 1e-3); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 1e-16); //0

		a = Angle.inDegrees(89.999999);
		p = new CodedPhase(a.codedPhase());
		//System.out.println("tan 89.999999 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(89.999999)));
		//System.out.println("89.999999 degrees == "+p.angle().getDegrees()+"   "+a.getDegrees());
		assertEquals(StrictMath.tan(a.getRadians()), p.tan(), 1e0); //0
		assertEquals(a.getDegrees(), p.angle().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 0); //0

		a = Angle.inDegrees(90.0);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 0); //0

		a = Angle.inDegrees(89.99999999999);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 0); //0
		//System.out.println("89.99999999999 degrees == "+p.angle().getDegrees()+"   "+a.getDegrees());
		
		//limit to exact degree measure+...
		a = Angle.inDegrees(89.999999999999);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().getDegrees(), 1e-11); //0
		assertEquals((a.getPiRadians()), p.angle().getPiRadians(), 1e-16); //0

		a = Angle.inDegrees(359);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
		//System.out.println("tan 359 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(359)));
		//System.out.println("sin 359 degrees == "+p.sin()+"   "+StrictMath.sin(StrictMath.toRadians(359)));
		//System.out.println("cos 359 degrees == "+p.cos()+"   "+StrictMath.cos(StrictMath.toRadians(359)));
		a = Angle.inDegrees(45);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
		//System.out.println("tan 45 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(45)));
		//System.out.println("sin 45 degrees == "+p.sin()+"   "+StrictMath.sin(StrictMath.toRadians(45)));
		//System.out.println("cos 45 degrees == "+p.cos()+"   "+StrictMath.cos(StrictMath.toRadians(45)));
		a = Angle.inDegrees(1);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
		//System.out.println("tan 1 degrees == "+p.tan()+"   "+StrictMath.tan(StrictMath.toRadians(1)));
		//System.out.println("sin 1 degrees == "+p.sin()+"   "+StrictMath.sin(StrictMath.toRadians(1)));
		//System.out.println("cos 1 degrees == "+p.cos()+"   "+StrictMath.cos(StrictMath.toRadians(1)));

		a = Angle.inDegrees(0);
		p = new CodedPhase(a.codedPhase());
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
		//System.out.println(p.tan());
		assertEquals(p.tan(),0D,1e-15);
		
		a.setDegrees(45d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		assertEquals(p.tan(),1,1e-15);
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(90d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		//System.out.println("ans: " + (p.tan()) +" ans+eps: " + ((p.tan())+ 0.4999999999999999d));
		assertEquals(1/p.tan(),0d,1e-15);
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(180d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		assertEquals(p.tan(),0d,1e-15);
		assertEquals(a.getDegrees(), p.angle().unsignedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().unsignedPrinciple().getPiRadians(), 0); //0
			
		a.setDegrees(-45d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		assertEquals(p.tan(),-1,1e-15);
		assertEquals(a.getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(-90d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		//System.out.println("ans: " + (p.tan()) +" ans+eps: " + ((p.tan())+ 0.4999999999999999d));
		assertEquals(1/p.tan(),0d,1e-15);
		assertEquals(a.getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(-180d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		assertEquals(p.tan(),0d,1e-15);
		assertEquals(a.getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(270d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		//System.out.println("ans: " + (p.tan()) +" ans+eps: " + ((p.tan())+ 0.4999999999999999d));
		assertEquals(1/p.tan(),0d,1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(-270d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		//System.out.println("ans: " + (p.tan()) +" ans+eps: " + ((p.tan())+ 0.4999999999999999d));
		assertEquals(1/p.tan(),0d,1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(-179.99d);
		p = a.codedPhase();
		//System.out.println("tan(-179.99d) = "+ p.tan());
		//System.out.println("tan(-179.99d) = "+ StrictMath.tan(StrictMath.toRadians(-179.99d)));
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(-179.99d)),1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 1e-13); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 1e-15); //0
		
		a.setDegrees(-180.0d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(-180.0d)),1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(179.99d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(179.99d)),1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 1e-13); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 1e-15); //0
		
	
		a.setDegrees(89.99d);
		p = a.codedPhase();
		//System.out.println("my tan(89.99d) = "+ p.tan());
		//System.out.println("tan(89.99d) = "+ StrictMath.tan(StrictMath.toRadians(89.99d)));
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(89.99d)),1e-7);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 1e-13); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 1e-16); //0

		a.setDegrees(90.0d);
		p = a.codedPhase();
		//System.out.println(p.tan());
		//System.out.println("tan(90.0d) = "+ p.tan());
		////System.out.println("tan(90.0d) = "+ StrictMath.tan(StrictMath.toRadians(90.0d)));
		//System.out.println("1/tan(90.0d) = "+ 1/StrictMath.tan(StrictMath.toRadians(90.0d)));
		assertEquals(1/p.tan(),1/StrictMath.tan(StrictMath.toRadians(90.0d)),1e-15);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		
		a.setDegrees(90.01d);
		p = a.codedPhase();
		//System.out.println("tan(90.01d) = "+ p.tan()); //mine more accurate!!
		//System.out.println("tan(90.01d) = "+ StrictMath.tan(StrictMath.toRadians(90.01d)));
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(90.01d)),1e-8);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		
		a.setDegrees(-90.01d);
		p = a.codedPhase();
		//System.out.println("tan(-90.01d) = "+ p.tan());
		//System.out.println("tan(-90.01d) = "+ StrictMath.tan(StrictMath.toRadians(90.0d)));
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(-90.01d)),1e-8);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0

		a.setDegrees(45.01d);
		p = a.codedPhase();
		//System.out.println("tan(45.01d) = "+ p.tan());
		//System.out.println("tan(45.01d) = "+ StrictMath.tan(StrictMath.toRadians(90.0d)));
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(45.01d)),1e-14);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 0); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 0); //0
		
		a.setDegrees(44.99d);
		p = a.codedPhase();
		//System.out.println(p.tan()+" is pos??");
		assertEquals(p.tan(),StrictMath.tan(StrictMath.toRadians(44.99d)),1e-14);
		assertEquals(a.signedPrinciple().getDegrees(), p.angle().signedPrinciple().getDegrees(), 1e-14); //0
		assertEquals((a.signedPrinciple().getPiRadians()), p.angle().signedPrinciple().getPiRadians(), 1e-16); //0

		
	}
	@Test
	public void testTanHalf() {
		
		Angle a = Angle.inDegrees(90);
		CodedPhase p = new CodedPhase(a.codedPhase());
		//System.out.println(p.tanHalf());
		assertEquals(p.tanHalf(),1.0D,1e-15);
		
		a.setDegrees(180.0D);
		p = a.codedPhase();
		//System.out.println(p.tanHalf());
		assertEquals(1d/p.tanHalf(),0,1e-15);
		
		a.setDegrees(-90.0D);
		p = a.codedPhase();
		//System.out.println(p.tanHalf());
		assertEquals(p.tanHalf(),-1.0d,1e-15);
		
		a.setDegrees(-180.0D);
		p = a.codedPhase();
		//System.out.println(p.tanHalf());
		assertEquals(1d/p.tanHalf(),0,1e-15);
		
		a.setDegrees(0d);
		p = a.codedPhase();
		//System.out.println(p.tanHalf());
		assertEquals(p.tanHalf(),0,1e-15);
		
	}
	
	@Test
	public void testWhatMissed() {
		
		
		//cotHalf()
		Angle a = Angle.inDegrees(30);
		Angle b = Angle.inDegrees(60);		
		CodedPhase c = a.codedPhase();
		CodedPhase d = a.codedPhase();
		CodedPhase e = b.codedPhase();
		assertEquals(1/StrictMath.tan(a.getRadians()),e.cotHalf(),1e-16);
		assertEquals(d.tan(),e.cot(),1e-15);
		assertEquals(e.tan(),d.cot(),1e-15);

		//hashcode, equals
		int hashC = c.hashCode();
		int hashD = d.hashCode();
		assertEquals(hashC,hashC);
		assertEquals(hashC,hashD);	
		assertTrue(c.equals(d));
		assertTrue(c.equals(c));
		assertTrue(c.equals(d));
		assertFalse(c.equals(a));
		d.set(b.codedPhase());
		assertFalse(c.equals(d));
		a.setPiRadians(Double.NaN);
		b.setPiRadians(Double.NaN);
		c.set(b.codedPhase());
		d.set(b.codedPhase());
		c.equals(CodedPhase.EMPTY);
		assertTrue(c.equals(d));
		d=null;
		assertFalse(c.equals(d));

		//for later test of signs +/- zero, infinity
		Double zm = -0d; 
		Double zp = 0d; 
		
		//tan, cot, cothalf
		c.set(CodedPhase.ZERO);
		assertTrue(c.tan()==0d);
		assertTrue(c.cot()==Double.POSITIVE_INFINITY);
		assertTrue(c.cotHalf()==Double.POSITIVE_INFINITY);
		
		d = new CodedPhase(CodedPhase.NEGATIVE_ZERO);
		assertTrue(d.cotHalf()==Double.NEGATIVE_INFINITY);
		
		
		c.set(CodedPhase.NEGATIVE_RIGHT);
		assertTrue(c.isOrthogonal());
    	//System.out.println("orthogonal check c:"+c.tanHalf());
		assertTrue(c.tanHalf()==-1d);
    	//System.out.println("orthogonal check c:"+c.tan());
		assertTrue( ((c.tan()) == Double.NEGATIVE_INFINITY) );
		assertTrue(zm.equals(c.cot()));
		assertTrue(c.cotHalf()==-1d);
		
		d.set(CodedPhase.RIGHT);		
		d = new CodedPhase(CodedPhase.RIGHT);
		assertTrue(d.isOrthogonal());
		d.set(d.negate());		
		assertTrue(d.isOrthogonal());
		d = CodedPhase.encodes(1d);		
		assertTrue(d.isOrthogonal());
		assertTrue(d.tan()==Double.POSITIVE_INFINITY);
		assertTrue(zp.equals(d.cot()));
		assertTrue(d.tanHalf()==1d);
		assertTrue(d.cotHalf()==1d);

		c.set(CodedPhase.NEGATIVE_STRAIGHT);
		assertTrue(c.isStraight());
		assertTrue(c.tanHalf()==Double.NEGATIVE_INFINITY);
		//System.out.println("zm:"+ zm);
		//System.out.println("c:"+ c.angle().getDegrees());
		//System.out.println("c.tanHalf():"+ c.tanHalf());
		//System.out.println("c.cotHalf():"+ c.cotHalf());
		//System.out.println("c.cot():"+ c.cot());
		//System.out.println("c.tan():"+ c.tan());
		assertTrue(zm.equals(c.tan()));
		assertTrue(zm.equals(c.cotHalf()));
		

		d = CodedPhase.encodes(CodedPhase.CODE_STRAIGHT);		
		//System.out.println(d.angle().getPiRadians());
		//System.out.println("d.cotHalf():"+ d.cotHalf());
		//System.out.println("d.cotHalf():"+ d.cotHalf());
		//System.out.println("d.tanHalf():"+ d.tanHalf());
		//System.out.println("d.cot():"+ d.cot());
		//System.out.println("d.tan():"+ d.tan());
		assertTrue(zp.equals(d.cotHalf()));
		assertTrue(d.cot()==Double.POSITIVE_INFINITY);
		assertTrue(d.tanHalf()==Double.POSITIVE_INFINITY);
		assertTrue(zp.equals(d.tan()));
		assertTrue(zp.equals(d.cotHalf()));

         		
		e = new CodedPhase(CodedPhase.RIGHT);
    	//System.out.println("orthogonal check e:"+e.tanHalf());
		assertTrue(e.isOrthogonal());
		e = new CodedPhase(CodedPhase.NEGATIVE_RIGHT);
    	//System.out.println("orthogonal check e:"+e.tanHalf());
		assertTrue(e.isOrthogonal());
		
		//encodes
    	e = CodedPhase.encodes(1.0d);
    	//System.out.println("orthogonal check e:"+e.tanHalf());
		assertTrue(e.isOrthogonal());

		e = Angle.inDegrees(0).codedPhase();
		assertEquals(1/StrictMath.tan(0),e.cotHalf(),1e-16);
		assertEquals(StrictMath.tan(0),e.tanHalf(),1e-16);
		e = Angle.inDegrees(-0).codedPhase();
		assertEquals(1/StrictMath.tan(-0),e.cotHalf(),1e-16);
		assertEquals(StrictMath.tan(-0),e.tanHalf(),1e-16);
		e = Angle.inDegrees(-90).codedPhase();
		assertEquals(1/StrictMath.tan(Angle.inDegrees(-45).getRadians()),e.cotHalf(),1e-15);
		assertEquals(StrictMath.tan(Angle.inDegrees(-45).getRadians()),e.tanHalf(),1e-15);
		
		e = Angle.inPiRadians(-1).codedPhase();
		CodedPhase f = Angle.inPiRadians(-1/2d).codedPhase();
        assertEquals(f.tan(),e.tanHalf(),1e-17);
		
		assertEquals(1/StrictMath.tan(Angle.inDegrees(-90).getRadians()),e.cotHalf(),1e-16);
    	
		//System.out.println("tan -90 degrees:"+ StrictMath.tan(Angle.inDegrees(-90).getRadians()));
    	
       	//System.out.println("tan -90 degrees:"+StrictMath.tan(Angle.inPiRadians(-1/2.d).getRadians()));

      	//System.out.println("-90 degrees diff:"+ ((180d/StrictMath.PI)*StrictMath.atan2(-1,0) + 90.0) );
      	//System.out.println("-half PI diff:"+ (StrictMath.atan2(-1,0) + (StrictMath.PI/2) ));
      	
      	//System.out.println("tan -90 degrees:"+ StrictMath.tan(StrictMath.atan2(-1,0)));
    	//System.out.println("tan -90 degrees:"+ StrictMath.tan(StrictMath.atan2(Double.NEGATIVE_INFINITY,0)));
       	
    	//System.out.println("tan -90 degrees:"+ CodedPhase.NEGATIVE_RIGHT.tan());       	
        //System.out.println("tan LEFT_ORTHOGONAL:"+e.tanHalf());

        assertEquals(CodedPhase.NEGATIVE_RIGHT.tan(),e.tanHalf(),1e-16);
        
		assertEquals(Double.NEGATIVE_INFINITY,e.tanHalf(),1e-16);
		assertEquals(Double.NEGATIVE_INFINITY,e.trueRound().tanHalf(),1e-16);
		
		e = Angle.inDegrees(180).codedPhase();
		assertEquals(1/StrictMath.tan(Angle.inDegrees(90).getRadians()),e.cotHalf(),1e-16);
        //System.out.println("tan of RIGHT_ORTHOGONAL:"+CodedPhase.RIGHT.tan());
        //System.out.println("tanHalf RIGHT_STRAIGHT:"+e.tanHalf());
        CodedPhase cp = new CodedPhase(CodedPhase.STRAIGHT);
        
        System.out.println("tan RIGHT:"+CodedPhase.RIGHT.tan());
		assertTrue(Double.isInfinite(CodedPhase.RIGHT.tan()));
		e = Angle.inDegrees(90).codedPhase();
		assertEquals(1/StrictMath.tan(Angle.inDegrees(45).getRadians()),e.cotHalf(),1e-15);
		assertEquals(StrictMath.tan(Angle.inDegrees(45).getRadians()),e.tanHalf(),1e-15);
		e = Angle.inDegrees(270).codedPhase();
		assertEquals(1/StrictMath.tan(Angle.inDegrees(-45).getRadians()),e.cotHalf(),1e-15);
		assertEquals(StrictMath.tan(Angle.inDegrees(-45).getRadians()),e.tanHalf(),1e-15);

		
		CodedPhase.main(null);
		

	}
	

}
