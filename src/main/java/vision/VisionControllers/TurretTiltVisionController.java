package vision.VisionControllers;

import MotionProfiling.PID_Classes.PID_Gains;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Robot;
import vision.VisionClass.VisionControllerInterface;

public class TurretTiltVisionController implements VisionControllerInterface{

	@Override
	public double getSource() {
		// Need to return Source for PID
		return 0;
	}

	@Override
	public Subsystem getSubsystem() {
		// Need to return the subsystem who will do the tilt
		return Robot.turretTiltSystem;
	}

	@Override
	public void setOutput(double output) {
		// Need to put the output value for motors
		Robot.turretTiltSystem.setpointPosition(castYpixel(output, 0));
	}

	@Override
	public PID_Gains getGains() {
		// Need to return PID Gains for tilt system
		return null;
	}

	@Override
	public double getMaxOutput() {
		// Need to return pan max output
		return 0;
	}

	@Override
	public double getMaxerror() {
		// Need to return tilt max error
		return 0;
	}

	@Override
	public double castYpixel(double yPixel, double sourcePosition) {
		// TODO Auto-generated method stub
		return yPixel;
	}
}
