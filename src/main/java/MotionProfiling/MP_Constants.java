package MotionProfiling;

public class MP_Constants {
	
	/**
	 * Drive System Constants
	 */
	
	public static final double ROBOT_WIDTH = 0.7;
	public static final double ENCODER_DISTANCE_PER_PULSE = 0.0019540791402052 / 5; 

	// not the real maximum speed. It is the maximum speed you want the robot to drive.
	public static final double MP_DEFAULT_VMAX = 3.5;
	
	public static final double MAX_EXELRATION_ACC = 2;
	public static final double MAX_STOPPING_ACC = 4;

	
	public static final double KP = 12;//20
	public static final double KI = 0;
	public static final double KD = 0;
	public static final double MAX_SPEED = 4.12;
	public static final double KV = 1 / MAX_SPEED;
	public static final double KA = 0.15;//0.08
	
	public static final double NAVX_KP = 20;//12;//8
	public static final double NAVX_KI = 0;
	public static final double NAVX_KD = 0;
	
}
