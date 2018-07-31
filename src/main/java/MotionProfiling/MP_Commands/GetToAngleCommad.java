package MotionProfiling.MP_Commands;

import MotionProfiling.MP_Subsystem;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetToAngleCommad extends Command {

	private MP_Subsystem system_;
	private double angle_;
	private boolean direction_;
	
    public GetToAngleCommad(MP_Subsystem system, double angle, boolean bigFromAngle_Or_SmallFromAngle) {
        // Use requires() here to declare subsystem dependencies
        //requires(system);
    	
    	system_ = system;
    	angle_ = angle;
    	direction_= bigFromAngle_Or_SmallFromAngle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(direction_){
    		return system_.getPosition() > angle_;
    	}
    	else{
    		return system_.getPosition() < angle_;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
