 package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class DriveWithJoysticks extends Command{

	private double speed_, turn_;
	//private double lastSpeed_ = 0;
	
	public DriveWithJoysticks() {
		// TODO Auto-generated constructor stub
		requires(Robot.driveSystem);
	}
	
	
	protected void initialize() {
		// TODO Auto-generated method stub
		//lastSpeed_ = 0;
	}

	protected void execute() {
		speed_ = Robot.oi.AdelStick.getRawAxis(1);
		turn_ = Robot.oi.AdelStick.getRawAxis(3) - Robot.oi.AdelStick.getRawAxis(2);//4	-
	
		Robot.driveSystem.arcade(-speed_, turn_);
	}

	
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.driveSystem.tank(0,0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}

}
