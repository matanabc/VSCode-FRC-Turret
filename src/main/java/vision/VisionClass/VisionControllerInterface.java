package vision.VisionClass;

import MotionProfiling.PID_Classes.PID_Gains;
import edu.wpi.first.wpilibj.command.Subsystem;

public interface VisionControllerInterface{
	
	/**
	 * @return 
	 * need to return the subsystem you want to use in your controller
	 */
	public Subsystem getSubsystem();
	
	/**
	 * @return 
	 * need to return the PID_Gains of your system  if you use PWM control, if not return null 
	 */
	public PID_Gains getGains();
	
	/**
	 * @return 
	 * need to return value of sensor to use in the PWM controller and for history, if you don't want to do both return 0
	 */
	public double getSource();

	/**
	 * @param 
	 * output	-		this will be the output or position for what you want it to control like motors or servo
	 */
	public void setOutput(double output);
	
	/**
	 * @return 
	 * need to return max output if you use PWM control, if not return 0  
	 */
	public double getMaxOutput();
	
	/**
	 * @return 
	 * need to return max error if you use PWM control, it will stop the PID if he between those value, if not return 0
	 */
	public double getMaxerror();
	
	/**
	 * @param 
	 * yPixel	-		this will be the height of the target in the frame, be aware that it will be in pixels
	 * 					 and not in meters from ground 
	 * @param 
	 * sourcePosition	-		this will be the position of your source when the frame was take
	 * 
	 * @return 
	 * need to return the convert of those parameters to RPM, Distance or Angle usually it will be by function.
	 * if you don't want to use it you can return 0 or yPixel at lest 
	 */
	public double castYpixel(double yPixel, double sourcePosition);
}
