package MotionProfiling.PID_Classes;

public class PID_Gains {
	public double kp;
	public double ki;
	public double kd;
	
	public PID_Gains(){
		kp = 0;
		ki = 0;
		kd = 0;
	}
	
	public PID_Gains(double kp_, double ki_, double kd_){
		kp = kp_;
		ki = ki_;
		kd = kd_;
	}
	
}
