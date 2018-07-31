package MotionProfiling.MP_Classes;

import java.text.DecimalFormat;

public class MP_Straight extends MP_DrivePath{
	
	private double distance_, V0_, Vend_, exelAcc_, stopAcc_, Vmax_;
	
	public MP_Straight(double distance, double Vmax, double V0, double Vend, double exelAcc,
			double stopAcc) {
		Vmax = Math.abs(Vmax);
		V0 = Math.abs(V0);
		Vend = Math.abs(Vend);
		
		if (distance < 0){
			Vmax *= -1;
			Vend *= -1;
			V0 *= -1;
		}
		
		left_ = new MP_Path(distance, Vmax, V0, Vend, exelAcc, stopAcc);
		right_ = left_;
		
		V0_ = V0;
		Vend_ = Vend;
		exelAcc_ = exelAcc;
		stopAcc_ = stopAcc;
		distance_ = distance;
		Vmax_ = Vmax;
		
		/*System.out.println("Straight : " + distance + "  " + Vmax
				+ "  " + V0 + "  " + Vend + "  " + acc);
		*/
	}
	
	public double getDistance(){
		return distance_;
	}

	public double getTotalAngleDegrees() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getVend() {
		// TODO Auto-generated method stub
		return Vend_;
	}

	@Override
	public double getVmax() {
		// TODO Auto-generated method stub
		return Vmax_;
	}

	@Override
	public double getV0() {
		// TODO Auto-generated method stub
		return V0_;
	}

	@Override
	public double getExelerationAcc() {
		// TODO Auto-generated method stub
		return exelAcc_;
	}
	
	@Override
	public double getStopingAcc() {
		// TODO Auto-generated method stub
		return stopAcc_;
	}
	
	@Override
	public String toString() {
		DecimalFormat f = new DecimalFormat("##.##");
		String s = "Staight: " + f.format(distance_)
		 		+ ", " + f.format(V0_)+ ", " + f.format(Vmax_) + ", " + f.format(Vend_) 
				+ ", " + f.format(exelAcc_) + ", " + f.format(stopAcc_);
		return s;
	}

	@Override
	public double getY(double time) {
		// TODO Auto-generated method stub
		return left_.getCurrentState(time).pos;
	}

	@Override
	public double getX(double time) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getAngleRadians(double time) {
		// TODO Auto-generated method stub
		return 0;
	}
	
	@Override
	public double getAngleSimulator(double time) {
		// TODO Auto-generated method stub
		return 0;
	}
}
