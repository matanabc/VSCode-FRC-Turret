package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class DriveSpeed extends Command {

	private double speedLeft, speedRight;

	public DriveSpeed(double speedLeft, double speedRight) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveSystem);
		this.speedLeft = speedLeft;
		this.speedRight = speedRight;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveSystem.tank(speedRight, speedLeft);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveSystem.tank(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
