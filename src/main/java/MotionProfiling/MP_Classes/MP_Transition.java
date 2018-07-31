package MotionProfiling.MP_Classes;

import MotionProfiling.MP_Constants;

public abstract class MP_Transition extends MP_DrivePath{

	protected double VR_, VS_, exelAcc_, stopAcc_, totalTime_, radius_; 
	protected boolean Vsign_;
		
	public double getX(double time){
		return NumericIntegral.numericIntegral(
				new NumericIntegral.Function() {
					double Rsign = radius_ > 0? 1 : -1;

					public double function(double t) {
						return Math.abs(getV(t)) * Math.sin(Math.abs(getAngleRadians(t)) * Rsign);
					}
				},
				0, time,
				0.00001);
	}
	
	public double getTotalX() {
		return getX(getTotalTime());
	}

	public double getY(double time){
		return NumericIntegral.numericIntegral(
				new NumericIntegral.Function() {
					
					public double function(double t) {
						return getV(t) * Math.cos(Math.abs(getAngleRadians(t)));
					}
				},
				0, time,
				0.00001);
	}
	
	public double getTotalY() {
		return getY(getTotalTime());
	}
	
	public double getTotalAngleDegrees() {
		return Math.toDegrees(getAngleRadians(totalTime_));
	}

	public double getTotalAngleDegreesWithSignIndicatingLeftOrRight() {
		double rSign = radius_ > 0? 1 : -1;
		return Math.abs(Math.toDegrees(getAngleRadians(totalTime_))) * rSign;
	}
	
	@Override
	public double getAngleRadians(double time){
		double Vsign = Vsign_? 1 : -1;		
		return Vsign * Math.abs(left_.getCurrentState(time).pos - right_.getCurrentState(time).pos)
				/ MP_Constants.ROBOT_WIDTH;
		
		
		//double dv = Math.abs(V_ - left_.getV0());
		//return Vsign * (((2 * dv * time) - Math.abs(acc_) * time * time) / MP_Constants.ROBOT_WIDTH);
		//return Vsign * (Math.abs(acc_) * time * time) / MP_Constants.ROBOT_WIDTH;
	}
	
	@Override
	public double getAngleSimulator(double time) {
		double angleSign = Vsign_ ^ radius_ > 0? -1 : 1;
		return angleSign * Math.abs(getAngleRadians(time));
	}

	@Override
	public double getVmax() {
		// TODO Auto-generated method stub
		return Math.abs(VS_) > Math.abs(VR_)? VS_ : VR_;
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
	
	public abstract double getV(double time);
}
