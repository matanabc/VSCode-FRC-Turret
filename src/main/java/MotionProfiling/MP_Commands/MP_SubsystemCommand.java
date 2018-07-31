package MotionProfiling.MP_Commands;

import MotionProfiling.MP_Subsystem;
import MotionProfiling.MP_Classes.MPGains;
import MotionProfiling.MP_Classes.MP_Path;
import MotionProfiling.PID_Classes.Setpoint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class MP_SubsystemCommand extends Command{

	private MP_Subsystem system_;

	private MP_Path motion_;
	private double 	startingTime_, time_;
	private MPGains gains_;
	private double errorSum_, lastError_;
	private double position_, Vmax_, Vend_, acc_;
	//private double maxSpeed_;
	private double startingPos_;
	
	private boolean end = false;

	public MP_SubsystemCommand(MP_Subsystem system, double position, double Vmax,
			double Vend, double acc, MPGains gains) {

		setVariables(system, position, Vmax, Vend, acc, gains);
	}

	public MP_SubsystemCommand(MP_Subsystem system, double position, MPGains gains){
		setVariables(system, position, system.getDefultVmax(), 0, system.getDefultAcc(), gains);
	}

	public MP_SubsystemCommand(MP_Subsystem system, double position){

		setVariables(system, position, system.getDefultVmax(), 0, system.getDefultAcc(),
				system.getDefultGains());
	}

	private void setVariables(MP_Subsystem system, double position, double Vmax,
			double Vend, double acc, MPGains gains){
		requires(system);

		position_ = position;
		Vmax_ = Math.abs(Vmax);
		Vend_ = Math.abs(Vend);
		acc_ = acc;
		system_ = system;
		gains_ = gains;
		//maxSpeed_ = maxSpeed;
	}

	@Override
	protected void initialize() {
		end = false;
		
		Setpoint currState = system_.getCurrState();

		startingPos_ = system_.getPosition();
		double distance = position_ - startingPos_;

		if (distance < 0){
			motion_ = new MP_Path(distance, -Vmax_, currState.vel, -Vend_, acc_, acc_);
		} else{
			motion_ = new MP_Path(distance, Vmax_, currState.vel, Vend_, acc_, acc_);
		}
		
		if (distance == 0){
			end = true; 
		}

		startingTime_ = Timer.getFPGATimestamp();
		errorSum_ = 0;
		lastError_ = 0;
		time_ = 0;
	}

	@Override
	protected void execute() {
		time_ = Timer.getFPGATimestamp() - startingTime_;

		Setpoint setpoint = motion_.getCurrentState(time_);

		double error = (startingPos_ + setpoint.pos) - system_.getPosition();

		double output = gains_.kv * setpoint.vel + gains_.ka * setpoint.acc +
				error * gains_.kp + errorSum_ * gains_.ki + (error - lastError_) * gains_.kd;

		errorSum_ += error;

		if (error > 0 ^ lastError_ > 0){
			errorSum_ = 0;
		}

		lastError_ = error;

		/*
		if(output > maxSpeed_ || output < -maxSpeed_){

			output = maxSpeed_ * (output / Math.abs(output));
		}*/
		system_.setOutput(output);

		/*SmartDashboard.putNumber(system_.toString() + " output MP:", output);
		SmartDashboard.putNumber(system_.toString() + " error MP:", error);
		SmartDashboard.putNumber(system_.toString() + " setpoint.pos + startingPos", (startingPos_ + setpoint.pos));
		SmartDashboard.putNumber(system_.toString() + " setpoint.pos", setpoint.pos);*/

	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return time_ >= motion_.getTotalTime() || end;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		time_ = Timer.getFPGATimestamp() - startingTime_;
		//system_.setCurrState(motion_.getCurrentState(time_));


		/*
		 * if(startingPos_ < motion_.getCurrentState(time_).pos){
			system_.setStartPos(motion_.getCurrentState(time_).pos + startingPos_);
		}else{
			if(motion_.getCurrentState(time_).pos + startingPos_ > 180){
				system_.setStartPos(startingPos_ - motion_.getCurrentState(time_).pos);
			}else{
				system_.setStartPos(startingPos_ + motion_.getCurrentState(time_).pos);
			}
		}	*/
		
		/*
		if(startingPos_ > motion_.getCurrentState(time_).pos){
			if(motion_.getCurrentState(time_).pos + startingPos_ > 180){
				system_.setStartPos(startingPos_ - motion_.getCurrentState(time_).pos);
			}else{
				system_.setStartPos(startingPos_ + motion_.getCurrentState(time_).pos);
			}
		}else{
			system_.setStartPos(motion_.getCurrentState(time_).pos + startingPos_);
		}*/
		
		

		Setpoint tmp = motion_.getCurrentState(time_);
		tmp.pos = position_;
		system_.setCurrState(tmp);

		//SmartDashboard.putNumber("motion curr state", motion_.getCurrentState(time_).pos);

		system_.setOutput(0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}

}
