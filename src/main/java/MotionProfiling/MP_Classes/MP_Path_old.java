package MotionProfiling.MP_Classes;

import MotionProfiling.PID_Classes.Setpoint;

public class MP_Path_old {
	public static final double MAX_SPEED = 2;
	public static final double MAX_ACCELERATION = 1;
	
	private double distance_, Vmax_, V0_, Vend_;
	private double speedingTime_, slowingTime_, cruisingTime_;
	private double speedingDistance_, cruisingDistance_, slowingDistance_;
	private double startAcc_, endAcc_;
	
	private boolean isLegal_ = true;
	
	public MP_Path_old(double distance){
		if (distance > 0){
			calculateTimes(distance, MAX_SPEED, 0, 0, MAX_ACCELERATION);
		}else {
			calculateTimes(distance, -MAX_SPEED, 0, 0, -MAX_ACCELERATION);
		}
	}
	
	public MP_Path_old(double V0, double Vend, double acc){
		startAcc_ = V0 < Vend? Math.abs(acc) : -Math.abs(acc);
		endAcc_ = 0; // Doesn't matter
		
		double t = Math.abs((Vend - V0) / acc);
		speedingTime_ = t;
		cruisingTime_ = 0;
		slowingTime_ = 0;
	
		distance_ = V0 * t + 0.5 * startAcc_ * t * t;
		speedingDistance_ = distance_;
		cruisingDistance_ = 0;
		slowingDistance_ = 0;
	
		V0_ = V0;
		Vend_ = Vend;
		Vmax_ = 0; // Doesn't matter
	}
	
	public MP_Path_old(double distance, double Vmax, double V0, double Vend, double acc) {
		calculateTimes(distance, Vmax, V0, Vend, acc);
	}
	
	private void calculateTimeFliped(double distance, double Vmax, double V0, double Vend,  double acc){
		calculateTimes(-distance, -Vmax, -V0, -Vend, acc);
		speedingDistance_ *= -1;
		cruisingDistance_ *= -1;
		slowingDistance_ *= -1;
		distance_ *= -1;
		V0_ *= -1;
		Vmax_ *= -1;
		Vend_ *= -1;
		startAcc_ *= -1;
		endAcc_ *= -1;
	}
	
	private void calculateTimes(double distance, double Vmax, double V0, double Vend,
			double exelAcc){
		double stopAcc = exelAcc;
		exelAcc = Math.abs(exelAcc);
		stopAcc = Math.abs(stopAcc);
		
		if (distance < 0.0 || (distance == 0.0 && V0 < 0.0)){
			calculateTimeFliped(distance, Vmax, V0, Vend, exelAcc);
			return;
		}
		
		startAcc_ = Vmax > V0? exelAcc : -exelAcc;
		endAcc_ = Vmax > Vend? -stopAcc : stopAcc;
		
		double speedingFromV02VendAcc = Vend > V0? exelAcc : -stopAcc;
		double speedingFromV02VendTime = (Vend - V0) / speedingFromV02VendAcc;
		double speedingFromV02VendDistance = V0 * speedingFromV02VendTime
					+ 0.5 * speedingFromV02VendAcc * speedingFromV02VendTime * speedingFromV02VendTime;
		
		
		if ((speedingFromV02VendDistance - distance) > 0.00001){
			isLegal_ = false;
			return;
		}
		
		double speeding2VmaxDistance = (Vmax * Vmax - V0 * V0) / (2 * startAcc_);
		double slowing2VendDistance = (Vmax * Vmax - Vend * Vend) / (2 * -endAcc_);
		double cruisingDistance = distance - (speeding2VmaxDistance + slowing2VendDistance);
		
		/*
		 * this condition is if just speeding to maximum velocity will take the
		 * mechanism too far you need to slow down before reaching maximum velocity
		 * 
		 * the condition is calculated by this table :
		 * 
		 *  _________________________________________________________________________
		 * |              | distance > dis2VmaxAndBack | distance <= dis2VmaxAndBack |
		 * |______________|____________________________|_____________________________|
		 * |distance > 0  |             ok             |     need to recalculate     |
		 * |______________|____________________________|_____________________________|
		 * |distance <= 0 |     need to recalculate    |              ok             |
		 * |______________|____________________________|_____________________________|
		*/
		
		if (distance > 0 ^ distance > (speeding2VmaxDistance + slowing2VendDistance)){
			double vmSquard = 	(2 * distance * startAcc_ * stopAcc 
								+ V0 * V0 * stopAcc 
								+ Vend * Vend * startAcc_)
					
								/ (startAcc_ + stopAcc);
			//Vmax = Math.sqrt((2 * distance * exelAcc + V0 * V0 + Vend * Vend) / 2);
			Vmax = Math.sqrt(vmSquard);
			cruisingDistance = 0;
		}
		
		speedingTime_ = Math.abs(Vmax - V0) / exelAcc;
		slowingTime_ = Math.abs(Vmax - Vend) / stopAcc;
		cruisingTime_ = cruisingDistance / Vmax;

		speedingDistance_ = (Vmax * Vmax - V0 * V0) / (2 * startAcc_);
		cruisingDistance_ = cruisingDistance;
		slowingDistance_ = (Vmax * Vmax - Vend * Vend) / (2 * -endAcc_);
		
		Vmax_ = Vmax;
		V0_ = V0;
		Vend_ = Vend;
		distance_ = distance;
		
		
		/*System.out.println("------------------------------");
		System.out.println("Times :");
		System.out.println("Speeding : " + speedingTime_);
		System.out.println("Cruising : " + cruisingTime_);
		System.out.println("Slowing  : " + slowingTime_);
		System.out.println("Total time : " + getTotalTime());
		
		System.out.println("Distances : ");
		System.out.println("Speeding : " + speedingDistance_);
		System.out.println("Cruising : " + cruisingDistance_);
		System.out.println("Slowing  : " + slowingDistance_);
		System.out.println("------------------------------");
		*/
	}
	
	public Setpoint getCurrentState(double currTime){
		Setpoint setpoint = new Setpoint();
		
		if (currTime < speedingTime_) {
			setpoint.vel = V0_ + startAcc_ * currTime;
			setpoint.pos = V0_ * currTime + 0.5 * startAcc_ * currTime * currTime;
			setpoint.acc = startAcc_;
			
			return setpoint;
		}
		
		currTime -= speedingTime_;
		
		if (currTime < cruisingTime_){
			setpoint.pos = speedingDistance_ + currTime * Vmax_;
			setpoint.vel = Vmax_;
			setpoint.acc = 0;
			
			return setpoint;
		}
		
		currTime -= cruisingTime_;
		
		if (currTime < slowingTime_){
			setpoint.pos = speedingDistance_ + cruisingDistance_ +
					Vmax_ * currTime + 0.5 * endAcc_ * currTime * currTime;
			setpoint.vel = Vmax_ + endAcc_ * currTime;
			setpoint.acc = endAcc_;
			
			return setpoint;
		}
		
		setpoint.pos = distance_;
		setpoint.vel = Vend_;
		setpoint.acc = 0;
		
		return setpoint;
	}
	
	public double getTotalTime(){
		return speedingTime_ + cruisingTime_ + slowingTime_;
	}
	
	public void scale(double scaleRate){
		distance_ *= scaleRate;
		Vmax_ *= scaleRate;
		startAcc_ *= scaleRate;
		endAcc_ *= scaleRate;
		V0_ *= scaleRate;
		Vend_ *= scaleRate;
		
		speedingDistance_ *= scaleRate;
		cruisingDistance_ *= scaleRate;
		slowingDistance_ *= scaleRate;
	}
	
	public boolean isLegal(){
		return isLegal_;
	}
	
	public double getV0(){
		return V0_;
	}
}
