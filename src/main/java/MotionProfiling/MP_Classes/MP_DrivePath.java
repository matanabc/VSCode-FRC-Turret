package MotionProfiling.MP_Classes;

public abstract class MP_DrivePath {
	
	protected MP_Path left_, right_;
	
	public MP_Path getLeftPath(){
		return left_;
	}
	public MP_Path getRightPath(){
		return right_;
	}
	
	// public abstract double getTotalX();
	// public abstract double getTotalY();
	public abstract double getTotalAngleDegrees();
	public abstract double getVend();
	public abstract double getVmax();
	public abstract double getV0();
	public abstract double getExelerationAcc();
	public abstract double getStopingAcc();
	
	
	public abstract double getY(double time);
	public abstract double getX(double time);
	public abstract double getAngleRadians(double time);
	public abstract double getAngleSimulator(double time);
	
	public double getAngleDegrees(double time) {
		return Math.toDegrees(getAngleRadians(time));
	}
	
	public double getTotalTime(){
		return left_.getTotalTime();
	}
	
	public boolean isLegal(){
		return left_.isLegal() && right_.isLegal();
	}
	
	public abstract String toString();
}
