package MotionProfiling.MP_Commands;

import MotionProfiling.MP_Subsystem;
import MotionProfiling.PID_Classes.PID_Gains;
import MotionProfiling.PID_Classes.PID_Variables;
import edu.wpi.first.wpilibj.command.Command;

public class MP_SubsystemPID extends Command{

	private MP_Subsystem system_;
	private PID_Gains gains_;
	private PID_Variables pidV_;

	private double setpoint_;

	public MP_SubsystemPID(MP_Subsystem system, PID_Gains pidGains, double maxOutput) {
		requires(system);
		
		
		pidV_ = new PID_Variables();
		gains_ = pidGains;
		system_ = system;
		setpoint_ = system.getCurrState().pos;
		pidV_.maxOutput = maxOutput;		
	}	
	
	public MP_SubsystemPID(MP_Subsystem system) {
		requires(system);

		pidV_ = new PID_Variables();
		system_ = system;
		gains_ = system_.getDefultPID_Gains();
		setpoint_ = system.getCurrState().pos;
		pidV_.maxOutput = system_.getDefultMaxOutput();
	}	

	@Override
	protected void initialize() {
		pidV_.resetVars();

		setpoint_ = system_.getCurrState().pos;	
	}

	@Override
	protected void execute() {
		pidV_.pos = system_ != null ? system_.getPosition() : 0; 
		pidV_.error = setpoint_ - pidV_.pos;	
		pidV_.output =	pidV_.error * gains_.kp + pidV_.errorSum * gains_.ki + (pidV_.error - pidV_.lastError) * gains_.kd;	

		/*SmartDashboard.putNumber(system_.toString() + " pid setpoint : " , setpoint_);
		SmartDashboard.putNumber(system_.toString() + " pid pos : ", pidV_.pos);*/

		pidV_.errorSum += pidV_.error;

		if (pidV_.error > 0 ^ pidV_.lastError > 0){
			pidV_.errorSum = 0;
		}

		pidV_.lastError = pidV_.error;

		if (Math.abs(pidV_.output) > pidV_.maxOutput){
			pidV_.output = pidV_.maxOutput * (pidV_.output / Math.abs(pidV_.output));
		}
		
		system_.setOutput(pidV_.output);

		/*SmartDashboard.putNumber(system_.toString() + " pid Error: = ", pidV_.error);
		SmartDashboard.putNumber(system_.toString() + " pid Output: = ", pidV_.output);*/
		
		//System.out.println("----------!!!!!!!!!!!!!!!!!------------");
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		//time_ = Timer.getFPGATimestamp() - startingTime_;
		system_.setOutput(0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}
}
