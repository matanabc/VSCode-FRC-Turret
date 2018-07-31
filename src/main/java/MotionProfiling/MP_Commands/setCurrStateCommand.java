package MotionProfiling.MP_Commands;

import MotionProfiling.MP_Subsystem;
import MotionProfiling.PID_Classes.Setpoint;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class setCurrStateCommand extends Command {

	private MP_Subsystem system_;
	private Setpoint currState_;
    public setCurrStateCommand(MP_Subsystem system, double pos) {
        system_ = system;
        currState_ = new Setpoint(pos,0,0);
        
        requires(system_);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	system_.setCurrState(currState_);
    	//System.out.println("!!!!!!!!!!!!!!!----------------!!!!!!!!!!!!!!!!!!!!!");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
