package MotionProfiling.PID_Classes;

public class PID_Variables {
	public double error;
	public double lastError;
	public double errorSum;
	public double pos;
	public Setpoint setpoint;
	public double output;
	public double maxOutput;
	public double maxErr;
	
	public void resetVars(){
		error = 0;
		lastError = 0;
		errorSum = 0;
	}
}
