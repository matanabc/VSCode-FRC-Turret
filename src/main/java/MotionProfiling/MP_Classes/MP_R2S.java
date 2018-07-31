package MotionProfiling.MP_Classes;

import java.text.DecimalFormat;

public class MP_R2S extends MP_Transition{
	
	public MP_R2S(double radius, double V, double acc) {
		set(radius, V, V, acc, acc);
	}
	
	/*
	 * For a right turn put + in radius. For a left turn put - in radius.
	 */
	public MP_R2S(double radius, double VR, double VS, double exelAcc, double stopAcc) {
		set(radius, VR, VS, exelAcc, stopAcc);
	}

	private void set (double radius, double VR, double VS, double exelAcc, double stopAcc) {
		double leftRatio, rightRatio;
		
		Vsign_ = VS > 0;
		exelAcc_ = radius > 0? Math.abs(exelAcc) : -Math.abs(exelAcc);
		stopAcc_ = radius > 0? Math.abs(stopAcc) : -Math.abs(stopAcc);
		VR_ = VR;
		VS_ = VS;
		radius_ = radius;
		
		if (radius > 0){
			// right turn
			leftRatio = MP_Radius.bigRatio(radius); 
			rightRatio = MP_Radius.smallRatio(radius);
		}else {
			// left turn
			radius = Math.abs(radius);
			leftRatio = MP_Radius.smallRatio(radius);
			rightRatio = MP_Radius.bigRatio(radius);
		}
				
		
		left_ = new MP_Path(VR * leftRatio, VS, exelAcc, stopAcc);
		right_ = new MP_Path(VR * rightRatio, VS, exelAcc, stopAcc);
		
		if (left_.getTotalTime() > right_.getTotalTime()) {
			right_ = new MP_Path(VR * rightRatio, VS, (VS - VR * rightRatio) / left_.getTotalTime());
		} else {
			left_ = new MP_Path(VR * leftRatio, VS, (VS - VR * leftRatio) / right_.getTotalTime());
		}
	
		totalTime_ = left_.getTotalTime();
	}
	
	@Override
	public String toString() {
		DecimalFormat f = new DecimalFormat("##.##");
		String s = "R2S     : " + f.format(radius_) + ", " + f.format(VR_) + ", " + f.format(VS_) + ", "
				+ f.format(exelAcc_) + ", " + f.format(stopAcc_);
		return s;
	}
	
	@Override
	public double getV(double time) {
		// TODO Auto-generated method stub
		return VR_ + (VS_ - VR_) * (time / totalTime_);
	}
		
	@Override
	public double getV0() {
		// TODO Auto-generated method stub
		return VR_;
	}
	
	@Override
	public double getVend() {
		// TODO Auto-generated method stub
		return VS_;
	}
}
