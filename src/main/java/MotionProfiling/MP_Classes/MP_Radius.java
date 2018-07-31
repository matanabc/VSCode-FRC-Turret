package MotionProfiling.MP_Classes;

import java.text.DecimalFormat;

import MotionProfiling.MP_Constants;

public class MP_Radius extends MP_DrivePath{
	
	protected double angle_, radius_, Vend_, V0_, exelAcc_, stopAcc_, Vmax_;
	protected double angleSimulator_, radiusSimulator_;
	
	public MP_Radius(double radius, double angle, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc) {
		
		angle_ = angle;
		radius_ = radius;
		Vmax_ = Vmax;
		V0_ = V0;
		Vend_ = Vend;
		exelAcc_ = exelAcc;
		stopAcc_ = stopAcc;

		
		set(radius, angle, Vmax, V0, Vend, exelAcc, stopAcc);
	}
	
	/*
	 * Parameters :
	 * 			radius - 	The radius of the turn. For a left turn put -.
	 * 
	 * 			angle - 	The angle (part of a circle) the robot will do in the move.
	 * 						For moving backwords put -.
	 * 
	 * 			Vmax - 		The maximum speed in m/sec 
	 * 						of the MIDDlE of the robot will do (left and right avrage).
	 * 
	 * 			V0 -		The starting speed in m/sec 
	 * 						of the MIDDLE of the robot (left and right avrage).
	 * 
	 * 			acc - 		The maximum acceleration in m/sec^2 in absulut value.
	 */
	protected void set(double radius, double angle, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc){
		angle = Math.toRadians(angle);
		
		Vmax = Math.abs(Vmax);
		V0 = Math.abs(V0);
		Vend = Math.abs(Vend);
		
		if (angle < 0){
			Vmax *= -1;
			Vend *= -1;
			V0 *= -1;
		}
		
		Vmax_ = Vmax;
		V0_ = V0;
		Vend_ = Vend;
		
		double leftRatio, rightRatio;
		double distance = Math.abs(radius) * angle;
		
		if (radius == 0){
			distance = Math.abs(angle) * (MP_Constants.ROBOT_WIDTH / 2);
			
			Vmax = Math.abs(Vmax);
			V0 = Math.abs(V0);
			Vend = Math.abs(Vend);
			
			if (angle > 0){
				leftRatio = 1;
				rightRatio = -1;
			} else {
				leftRatio = -1;
				rightRatio = 1;
			}
			
		} else if (radius > 0) {
			leftRatio = bigRatio(radius);
			rightRatio = smallRatio(radius);
		}else {
			leftRatio = smallRatio(radius); 
			rightRatio = bigRatio(radius);
		}
		
		left_ = new MP_Path(distance, Vmax , V0, Vend, exelAcc, stopAcc);
		right_ = new MP_Path(distance, Vmax, V0, Vend, exelAcc, stopAcc);
		
		
		left_.scale(leftRatio);
		right_.scale(rightRatio);
		
		angleSimulator_ = radius_ > 0 ^ angle_ > 0? -Math.abs(angle_) : Math.abs(angle_);
		radiusSimulator_ = angle_ > 0? Math.abs(radius_) : -Math.abs(radius_);
	}
	
	public double getTotalX() {
		return radius_ * Math.cos(angle_);
	}

	public double getTotalY() {
		return radius_ * Math.sin(angle_);
	}

	public double getTotalAngleDegrees() {
		return angle_;
	}

	@Override
	public double getVend() {
		// TODO Auto-generated method stub
		return Vend_;
	}
	
	public double getRadius(){
		return radius_;
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
		String s = "Radius: " + f.format(radius_) + ", " + f.format(angle_) + ", " 
				+ f.format(V0_) + ", "+ f.format(Vmax_) + ", " + f.format(Vend_) + ", " 
				+ f.format(exelAcc_) + ", " + f.format(stopAcc_);
		return s;
	}
	
	public static double getLeftSpeedForRadius(double radius, double speed) {
		if (radius == 0 ){
			return speed;
		}
		
		speed = getMaxSpeedForRadius(radius) * speed;
		return radius > 0? bigRatio(radius) * speed : smallRatio(radius) * speed;
	}
	
	public static double getRightSpeedForRadius(double radius, double speed) {
		return getLeftSpeedForRadius(-radius, speed);
	}
	
	public static double getMaxSpeedForRadius(double radius){
		if (radius == 0 ){
			return MP_Constants.MP_DEFAULT_VMAX;
		}
		
		return MP_Constants.MP_DEFAULT_VMAX / (1 + MP_Constants.ROBOT_WIDTH / (2 * Math.abs(radius)));
	}
	
	public static double getBigAccForRadiusAvrageAcc(double radius, double acc){
		return acc * 1 + MP_Constants.ROBOT_WIDTH / (2 * radius);
	}
	
	public static double getMaxVel(double radius){
		if (radius == 0 ){
			return MP_Constants.MP_DEFAULT_VMAX;
		}
		
		return MP_Constants.MP_DEFAULT_VMAX / bigRatio(radius);
	}
	
	public static double getMaxAcc(double acc, double radius){
		if (radius == 0 ){
			return acc;
		}
		
		return acc / bigRatio(radius);
	}
	
	public static double bigRatio(double radius){
		return 1 + MP_Constants.ROBOT_WIDTH / (2 * Math.abs(radius));
	}
	
	public static double smallRatio(double radius){
		return 1 - MP_Constants.ROBOT_WIDTH / (2 * Math.abs(radius));
	}

	@Override
	public double getY(double time) {
		if (radius_ == 0) {
			return 0;
		}
		 
		return radiusSimulator_ * Math.sin(Math.abs(getAngleRadians(time)));
	}

	@Override
	public double getX(double time) {
		if (radius_ == 0) {
			return 0;
		}
		
		return (radius_ / Math.abs(radius_)) * Math.abs(radiusSimulator_ - (radiusSimulator_ * Math.cos(getAngleRadians(time))));
	}

	@Override
	public double getAngleRadians(double time) {
		return Math.toRadians(angle_) * 
				(left_.getCurrentState(time).pos / left_.getCurrentState(getTotalTime()).pos);
	}
	
	@Override
	public double getAngleSimulator(double time) {
		double angleSign = angle_ > 0 ^ radius_ > 0? -1 : 1;
		
		if (radius_ == 0) {
			angleSign = angle_ > 0? 1 : -1;
		}
		
		
		return angleSign * Math.abs(getAngleRadians(time));
	}
}
