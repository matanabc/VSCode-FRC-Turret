package MotionProfiling.MP_Classes;

import java.text.DecimalFormat;

public class MP_S2R extends MP_Transition{

	
	/*
	 * For a right turn put + in radius. For a left turn put - in radius.
	 */
	public MP_S2R(double radius, double V, double acc) {
		set(radius, V, V, acc, acc);
	}
	
	public MP_S2R(double radius, double VS, double VR, double exelAcc, double stopAcc) {
		set(radius, VS, VR, exelAcc, stopAcc);
	}
	
	private void set (double radius, double VS, double VR, double exelAcc, double stopAcc) {
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
		
		
		left_ = new MP_Path(VS, VR * leftRatio, exelAcc, stopAcc);
		right_ = new MP_Path(VS, VR * rightRatio, exelAcc, stopAcc);
		
		if (left_.getTotalTime() > right_.getTotalTime()) {
			if (right_.getTotalTime() == 0) {
				right_ = new MP_Path(VS * left_.getTotalTime(), VS, VS, VS, 1, 1);
			} else {
				right_ = new MP_Path(VS, VR * rightRatio, (VR * rightRatio - VS) / left_.getTotalTime());
			}
		} else if (left_.getTotalTime() < right_.getTotalTime()){
			if (left_.getTotalTime() == 0) {
				left_ = new MP_Path(VS * right_.getTotalTime(), VS, VS, VS, 1, 1);
			} else {
				left_ = new MP_Path(VS, VR * leftRatio, (VR * leftRatio - VS) / right_.getTotalTime());
			}
		}
		
		totalTime_ = left_.getTotalTime();
	}
	
	@Override
	public String toString() {
		DecimalFormat f = new DecimalFormat("##.##");
		String s = "S2R     : " + f.format(radius_) + ", " + f.format(VS_) + ", " + f.format(VR_) + ", "
				+ f.format(exelAcc_) + ", " + f.format(stopAcc_);
		return s;
	}

	@Override
	public double getV(double time) {
		// TODO Auto-generated method stub
		return VS_ + (VS_ - VR_) * (time / totalTime_);
	}


	@Override
	public double getVend() {
		// TODO Auto-generated method stub
		return VR_;
	}


	@Override
	public double getV0() {
		// TODO Auto-generated method stub
		return VS_;
	}
	
}
