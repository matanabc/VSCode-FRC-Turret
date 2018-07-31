package MotionProfiling.MP_Commands;

import MotionProfiling.MP_Classes.MPGains;
import MotionProfiling.MP_Classes.MP_DrivePath;
import MotionProfiling.MP_Classes.MP_Path;
import MotionProfiling.MP_Controllers.MPDoubleSidePos;
import MotionProfiling.MP_Controllers.MPDriveController;
import MotionProfiling.PID_Classes.PID_Variables;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;

public class MP_DrivePathFollower extends Command {

	protected MP_Path leftPath_, rightPath_;
	private MP_DrivePath path_;
	private MPGains gains_ = new MPGains();

	private double 	startingTime_, time_;
	private PID_Variables leftPID_ = new PID_Variables();
	private PID_Variables rightPID_ = new PID_Variables();

	private MPDriveController driveController_;

	public MP_DrivePathFollower(MPDriveController driveController, MP_DrivePath path, MPGains gains) {
		// TODO Auto-generated constructor stub
		requires(Robot.driveSystem);

		path_ = path;

		leftPath_ = path.getLeftPath();
		rightPath_ = path.getRightPath();

		gains_ = gains;
		driveController_ = driveController;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		leftPID_.resetVars();
		rightPID_.resetVars();

		driveController_.reset();
		startingTime_ = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute()
	{
		time_ = Timer.getFPGATimestamp() - startingTime_;

		MPDoubleSidePos currPos = driveController_.getPosition();

		leftPID_.setpoint = leftPath_.getCurrentState(time_);
		rightPID_.setpoint = rightPath_.getCurrentState(time_);
		//System.out.println("vel Left ="+leftPID_.setpoint.vel);
		//System.out.println("vel Right ="+rightPID_.setpoint.vel);
		/*SmartDashboard.putNumber("leftPID_.setpoint", leftPID_.setpoint.pos);
		SmartDashboard.putNumber("rightPID_.setpoint", rightPID_.setpoint.pos);*/

		leftPID_.error = leftPID_.setpoint.pos - currPos.left;
		rightPID_.error = rightPID_.setpoint.pos - currPos.right;
		SmartDashboard.putNumber("errorL", leftPID_.error);
		SmartDashboard.putNumber("errorR", rightPID_.error);
		
		//System.out.println("right error1 = " + rightPID_.error + " leftPID_.setpoint.pos = " + leftPID_.setpoint.pos + " currPos.left" + currPos.left);
		//System.out.println("left error1 = " + leftPID_.error);
		/*SmartDashboard.putNumber("currL", currPos.left);
		SmartDashboard.putNumber("currR", currPos.right);

		SmartDashboard.putNumber("leftVel", leftPID_.setpoint.vel);
		SmartDashboard.putNumber("rightVel", rightPID_.setpoint.vel);

		SmartDashboard.putNumber("errorL", leftPID_.error);
		SmartDashboard.putNumber("errorR", rightPID_.error);

		SmartDashboard.putNumber("kp * error (right)", gains_.kp * rightPID_.error);
		SmartDashboard.putNumber("kp * error (left)", gains_.kp * leftPID_.error);
		System.out.println("----------------leftPID_.error = " + leftPID_.error);*/

		double leftOutput = gains_.kv * leftPID_.setpoint.vel + gains_.ka * leftPID_.setpoint.acc
				+ (gains_.kp * leftPID_.error) + (gains_.ki * leftPID_.errorSum) +
				(gains_.kd * (leftPID_.error - leftPID_.lastError));
		
		/*System.out.println("gains_.kp = " + gains_.kp);
		System.out.println("leftPID_.error = " + leftPID_.error);
		System.out.println("gains_.ki = " + gains_.ki);
		System.out.println("leftPID_.errorSum = " + leftPID_.errorSum);
		System.out.println("gains_.kd = " + gains_.kd);
		System.out.println("leftPID_.error - leftPID_.lastErr = " + (leftPID_.error - leftPID_.lastError));
		System.out.println("leftOutput = " + leftOutput);*/

		double rightOutput = gains_.kv * rightPID_.setpoint.vel + gains_.ka * rightPID_.setpoint.acc
				+ gains_.kp * rightPID_.error + gains_.ki * rightPID_.errorSum +
				gains_.kd * (rightPID_.error - rightPID_.lastError);
		//System.out.println("right error2 = " + rightPID_.error);
		leftPID_.errorSum += leftPID_.error;
		rightPID_.errorSum += rightPID_.error;

		if (leftPID_.error > 0 ^ leftPID_.lastError > 0){
			leftPID_.errorSum = 0;
		}

		if (rightPID_.error > 0 ^ rightPID_.lastError > 0){
			rightPID_.errorSum = 0;
		}

		leftPID_.lastError = leftPID_.error;
		rightPID_.lastError = rightPID_.error;

		//if(Robot.oi.)
		driveController_.setOutput(leftOutput, rightOutput);//leftOutput, rightOutput


		/*SmartDashboard.putNumber("leftOutput", leftOutput);
		SmartDashboard.putNumber("rightOutput", rightOutput);*/

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return time_ >= rightPath_.getTotalTime();
	}

	// Called once after isFinished returns true
	protected void end() {
		if(path_.getVend() == 0){
			driveController_.setOutput(0, 0);
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	public MP_DrivePath getPath(){
		return path_;
	}
}
