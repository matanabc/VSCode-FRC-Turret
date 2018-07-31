package robot.commands.drive;

import java.util.LinkedList;

import LogFile.CommandWrite;
import robot.Robot;

public class DriveSpeedCommandWrite extends CommandWrite{

	private double speedLeft, speedRight;

	public DriveSpeedCommandWrite(double speedLeft, double speedRight) {
		super("Drive turn low speed button",  Robot.logFile);

		requires(Robot.driveSystem);
		this.speedLeft = speedLeft;
		this.speedRight = speedRight;
	}

	@Override
	protected void initializeWrite() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void executeWrite() {
		// TODO Auto-generated method stub
		Robot.driveSystem.tank(speedRight, speedLeft);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void endWrite() {
		// TODO Auto-generated method stub

		Robot.driveSystem.tank(0, 0);
	}

}
