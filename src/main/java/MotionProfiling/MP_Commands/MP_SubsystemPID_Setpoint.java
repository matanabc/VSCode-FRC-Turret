package MotionProfiling.MP_Commands;

import MotionProfiling.MP_Subsystem;
import MotionProfiling.PID_Classes.PID_Gains;
import MotionProfiling.PID_Classes.PID_Variables;
import MotionProfiling.PID_Classes.Setpoint;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MP_SubsystemPID_Setpoint extends Command{

	private MP_Subsystem system_;
	private PID_Gains gains_;
	private PID_Variables pidV_;
	
	private double setpoint_;
	

	public MP_SubsystemPID_Setpoint(MP_Subsystem system, double setpoint, PID_Gains pidGains, double maxOutput, double maxErr) {
		requires(system);

		pidV_ = new PID_Variables();
		gains_ = pidGains;
		system_ = system;
		setpoint_ = setpoint;
		pidV_.maxOutput = maxOutput;
		pidV_.maxErr = maxErr;
	}
	
	public MP_SubsystemPID_Setpoint(MP_Subsystem system, double setpoint) {
		requires(system);

		pidV_ = new PID_Variables();
		system_ = system;
		gains_ = system_.getDefultPID_Gains();
		setpoint_ = setpoint;
		pidV_.maxOutput = system_.getDefultMaxOutput();
		pidV_.maxErr = system.getDefultMaxErr();
	}	

	@Override
	protected void initialize() {
		pidV_.resetVars();
		
	}

	@Override
	protected void execute() {
		
		pidV_.pos = system_ != null ? system_.getPosition() : 0; 
		pidV_.error = setpoint_ - pidV_.pos;	
		pidV_.output =	pidV_.error * gains_.kp + pidV_.errorSum * gains_.ki + (pidV_.error - pidV_.lastError) * gains_.kd;	

		SmartDashboard.putNumber(system_.toString() + " pid setpoint : " , setpoint_);
		SmartDashboard.putNumber(system_.toString() + " pid pos : ", pidV_.pos);

		pidV_.errorSum += pidV_.error;

		if (pidV_.error > 0 ^ pidV_.lastError > 0){
			pidV_.errorSum = 0;
		}

		pidV_.lastError = pidV_.error;

		if (Math.abs(pidV_.output) > pidV_.maxOutput){
			pidV_.output = pidV_.maxOutput * (pidV_.output / Math.abs(pidV_.output));

		}

		system_.setOutput(pidV_.output);

		SmartDashboard.putNumber(system_.toString() + " pid Error: = ", pidV_.error);
		SmartDashboard.putNumber(system_.toString() + " pid Output: = ", pidV_.output);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub}
		return Math.abs(pidV_.error) <= pidV_.maxErr;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		system_.setCurrState(new Setpoint(setpoint_, 0, 0));
		system_.setOutput(0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}
}
