package vision.VisionControllers;

import MotionProfiling.PID_Classes.PID_Gains;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.RobotMap;
import vision.VisionClass.VisionControllerInterface;

public class TurretPanVisionController implements VisionControllerInterface{

	@Override
	public double getSource() {
		// Need to return Source for PID
		return Robot.turretPanSystem.getSideEncoderPosition() + 10;
	}

	@Override
	public Subsystem getSubsystem() {
		// Need to return the subsystem who will do the tilt
		return Robot.turretPanSystem;
	}

	@Override
	public void setOutput(double output) {
		// Need to put the output value for motors
		//Robot.turretPanSystem.setSideMotorSetPoint(output);
		SmartDashboard.putNumber("SetOutpot!!!!", output);

	}

	@Override
	public PID_Gains getGains() {
		// Need to return PID Gains for tilt system
		return null;
	}

	@Override
	public double getMaxOutput() {
		// Need to return pan max output
		return RobotMap.PAN_MAX_OUTPUT;
	}

	@Override
	public double getMaxerror() {
		// Need to return tilt max error
		return RobotMap.PAN_MAX_ERROR;
	}

	@Override
	public double castYpixel(double yPixel, double sourcePosition) {
		// TODO Auto-generated method stub
		return 0;
	}
}
