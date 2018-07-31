package MotionProfiling.PID_Classes;

public class Setpoint {
	
	public double pos;
	public double vel;
	public double acc;
	
	public Setpoint(double pos_, double vel_, double acc_){

			pos = pos_;
			vel = vel_;
			acc = acc_;
				
	}

	public Setpoint() {
		
		pos = 0;
		vel = 0;
		acc = 0;
	}
	
}
