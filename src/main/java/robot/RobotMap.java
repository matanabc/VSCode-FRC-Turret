/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	//Joystick
	public static final int ADEL_JOYSTICK_CHANEL = 0;

	//can
	public static final int MASTER_SHOOTER_MOTOR_CAN = 0;
	public static final int SLAVE_SHOOTER_MOTOR_CAN = 1;
	public static final int SIDE_MOTOR_CAN = 2;

	//pwm
	public static final int DRIVE_RIGHT_PWM = 0;
	public static final int DRIVE_LEFT_PWM = 1;
	public static final int TILT_SERVO_PWM = 2;

	//drive
	public static final double DRIVE_SLOW = 0.5;

	//Shooter
	public static final double KP_SIDE = 2;
	public static final double KI_SIDE = 0;
	public static final double KD_SIDE = 60;
	public static final int KINTEGRAL_I_ZONE_SIDE = (int) (1023/KP_SIDE);

	public static final double KP_SPEEN_SPEED = 0.2;
	public static final double PULLS_PER_TIC = 4125;
	public static final double ENCODER_POSITION_ANGLE = 0.03773584905660377358490566037736;
	
	public static final int SERVO_MIN_ANGLE = 0;
	public static final int SERVO_MAX_ANGLE = 50;
	
	public static final double MAX_RPM_ERROR = 200;

	//vision
	public static final int MAX_DECODER_HOSTORY_SIZE = 1000;
	public static final double CAMERA_ANGLE = 52.3;
	public static final double CAMERA_WIDTH = 160;
	public static final String NT_VALUE_NAME = "TargetInfo";
	public static final double OFFSEAT_CAMERA_FROM_CENTER = 0;
	public static final double PAN_MAX_OUTPUT = 0.4;
	public static final double PAN_MAX_ERROR = 1;
	public static final double TILT_MAX_OUTPUT = 0.3;
	public static final double TILT_MAX_ERROR = 1;
	
	//write command
	public static final String FILE_PLACE = "/home/admin/LogFile.txt";
}
