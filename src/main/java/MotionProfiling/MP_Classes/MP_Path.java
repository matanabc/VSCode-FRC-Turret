package MotionProfiling.MP_Classes;
import java.util.ArrayList;

import MotionProfiling.PID_Classes.Setpoint;

public class MP_Path {
	public static final double MAX_SPEED = 2;
	public static final double MAX_ACCELERATION = 1;
	
	private double distance_, V0_;
	private double totalTime_;
	
	private boolean isLegal_ = true;
	
	private ArrayList<Segment> segments_ = new ArrayList<Segment>();
	
	public class Segment {
		private double distance, V0, Ve, totalTime, acc;
		
		public Segment(double V0, double Ve, double acc) {
			this.V0 = V0;
			this.Ve = Ve;
			this.acc = V0 < Ve? acc : -acc;
			
			this.totalTime = Math.abs((Ve - V0) / acc);
			distance = V0 * totalTime + 0.5 * this.acc * totalTime * totalTime;
		}
		
		public Segment(double V, double distance) {
			this.distance = distance;
			V0 = V;
			Ve = V;
			acc = 0;
			totalTime = distance / V;
		}
		
		public void scale(double rate) {
			distance *= rate;
			V0 *= rate;
			Ve *= rate;
			acc *= rate;
		}
		
		public Setpoint getCurrState(double time) {
			return new Setpoint(V0 * time + 0.5 * acc * time * time, V0 + acc * time, acc);
		}
		
		public double getTotalTime() {
			return totalTime;
		}
		
		public double getTotalDistance() {
			return distance;
		}
		
		public double getVe() {
			return Ve;
		}
		
		public double getAcc() {
			return acc;
		}
	}
	
	public MP_Path(double distance){
		if (distance > 0){
			calculateTimes(distance, MAX_SPEED, 0, 0, MAX_ACCELERATION, MAX_ACCELERATION);
		}else {
			calculateTimes(distance, -MAX_SPEED, 0, 0, -MAX_ACCELERATION, MAX_ACCELERATION);
		}
	}
	
	public MP_Path(double V0, double Vend, double acc){
		acc = Math.abs(acc);
		
		Segment temp = new Segment(V0, Vend, acc);
		
		segments_.add(temp);
		
		V0_ = V0;
		totalTime_ = temp.getTotalTime();
	}
	
	public MP_Path(double V0, double Vend, double exelAcc, double stopAcc){
		exelAcc = Math.abs(exelAcc);
		stopAcc = Math.abs(stopAcc);
		
		if (V0 > 0 ^ Vend > 0) {
			/*segments_.add(new Segment(V0, 0, stopAcc));
			segments_.add(new Segment(0, Vend, exelAcc));*/
			
			segments_.add(new Segment(V0, Vend, exelAcc));
		} else {
			if (Math.abs(V0) < Math.abs(Vend)) {
				segments_.add(new Segment(V0, Vend, exelAcc));
			} else {
				segments_.add(new Segment(V0, Vend, stopAcc));
			}
		}
				
		V0_ = V0;
		calcTotalTime();
	}
	
	public MP_Path(double distance, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc) {
		calculateTimes(distance, Vmax, V0, Vend, exelAcc, stopAcc);
	}
	
	private void calculateTimeFliped(double distance, double Vmax, double V0, double Vend,  
			double exelAcc, double stopAcc){
		calculateTimes(-distance, -Vmax, -V0, -Vend, exelAcc, stopAcc);
		
		if (!isLegal()) {
			return;
		}
		
		scale(-1);
		V0_ = segments_.get(0).V0;
	}
	
	private void calculateTimes(double distance, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc){
		exelAcc = Math.abs(exelAcc);
		stopAcc = Math.abs(stopAcc);
		
		V0_ = V0;
		
		if (distance < 0){
			calculateTimeFliped(distance, Vmax, V0, Vend, exelAcc, stopAcc);
			return;
		}
		
		if (Vmax <= 0) {
			isLegal_ = false;
			return;
		}
		
		if (Vend < 0) {
			isLegal_ = false;
			return;
		}
		
		if (Vmax < Vend) {
			isLegal_ = false;
			return;
		}
		
		if (V0 < 0) {
			Segment temp = new Segment(V0, 0, stopAcc);
			segments_.add(temp);
			distance -= temp.getTotalDistance();
			V0 = 0;
		}
		

		if (new Segment(V0, Vend, V0 < Vend? exelAcc : stopAcc).getTotalDistance() > distance) {
			System.out.println("Can't get from V0 to Vend");
			System.out.println("V0 " + V0 + "   Ve " + Vend + "  acc " + (V0 < Vend? exelAcc : stopAcc));
			isLegal_ = false;
			return;
		}
		
		Segment speeding2Vmax = new Segment(V0, Vmax, exelAcc);
		Segment slowing2Vend = new Segment(Vmax, Vend, stopAcc);
		
		double speedingAndSlowingDistance = 
				speeding2Vmax.getTotalDistance() + slowing2Vend.getTotalDistance();
		if (speedingAndSlowingDistance > distance) {
			double newVax = Math.sqrt((2 * distance * exelAcc * stopAcc 
					+ V0 * V0 * stopAcc 
					+ Vend * Vend * exelAcc)
		
					/ (exelAcc + stopAcc));
			
			if (newVax < Vend) {
				isLegal_ = false;
				return;
			}
			
			segments_.add(new Segment(V0, newVax, exelAcc));
			segments_.add(new Segment(newVax, Vend, stopAcc));
		} else {
			segments_.add(speeding2Vmax);
			segments_.add(new Segment(Vmax, distance - speedingAndSlowingDistance));
			segments_.add(slowing2Vend);
		}
		
		calcTotalTime();
	}
	
	private void calcTotalTime() {
		totalTime_ = 0;
		for (int i = 0; i < segments_.size(); i ++) {
			totalTime_ += segments_.get(i).getTotalTime();
		}
	}
	
	public Setpoint getCurrentState(double currTime){
		Setpoint setpoint = new Setpoint();
		Setpoint setpointTemp;
		Segment segmentTemp = new Segment(1, 1);
		
		for (int i = 0; i < segments_.size(); i ++) {
			segmentTemp = segments_.get(i);
			if (currTime < segmentTemp.getTotalTime()) {
				setpointTemp = segmentTemp.getCurrState(currTime);
				
				setpoint.pos += setpointTemp.pos;
				setpoint.vel = setpointTemp.vel;
				setpoint.acc = segmentTemp.acc;
				
				return setpoint;
			}
			
			setpoint.pos += segmentTemp.getTotalDistance();
			currTime -= segmentTemp.getTotalTime();
		}
		
		setpoint.vel = segmentTemp.getVe();
		setpoint.acc = segmentTemp.getAcc();
		
		return setpoint;
	}
	
	public double getTotalTime(){
		return totalTime_;
	}
	
	public void scale(double scaleRate){
		for (int i = 0; i < segments_.size(); i ++) {
			segments_.get(i).scale(scaleRate);
		}
	}
	
	public boolean isLegal(){
		return isLegal_;
	}
	
	public double getV0(){
		return V0_;
	}
}
