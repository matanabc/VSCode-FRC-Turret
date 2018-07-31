package robot.subsystems.Turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.RobotMap;

/**
 *
 */
public class TurretTiltSystem extends Subsystem {
	
	private Servo tiltServo = new Servo(RobotMap.TILT_SERVO_PWM);
	
	public TurretTiltSystem(){
		setpointPosition(RobotMap.SERVO_MIN_ANGLE);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setpointPosition(double position) {
    	if(position > RobotMap.SERVO_MAX_ANGLE){
    		tiltServo.setAngle(RobotMap.SERVO_MAX_ANGLE);
    	}else if(position < RobotMap.SERVO_MIN_ANGLE){
    		tiltServo.setAngle(RobotMap.SERVO_MIN_ANGLE);
    	}else{
    		tiltServo.setAngle(position);
    	}
    }
    
    public double getServoPosition() {
    	return tiltServo.getAngle();
    }
}

