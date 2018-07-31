package robot.subsystems.Turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import robot.RobotMap;

/**
 *
 */
public class TurretPanSystem extends Subsystem {

	private WPI_TalonSRX sideMoter_ = new WPI_TalonSRX(RobotMap.SIDE_MOTOR_CAN);
	
	public TurretPanSystem() {
		resetTalon();
		setSideToStay();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void resetTalon(){

		sideMoter_.set(com.ctre.phoenix.motorcontrol.ControlMode.Position,0);
		sideMoter_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
		sideMoter_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
		
    	sideMoter_.config_kF(0, 0, 0);
    	sideMoter_.config_kP(0, 0, 0);
    	sideMoter_.config_kI(0, 0, 0);
    	sideMoter_.config_kD(0, 0, 0);
    	
    	sideMoter_.config_kP(0, RobotMap.KP_SIDE, 0);
    	sideMoter_.config_kD(0, RobotMap.KD_SIDE, 0);
    	sideMoter_.config_IntegralZone(0, RobotMap.KINTEGRAL_I_ZONE_SIDE, 0);

	}
	
	public void setSideToStay(){
		sideMoter_.set(ControlMode.Position, (getSideEncoderPosition()/RobotMap.ENCODER_POSITION_ANGLE));
	}
	
	public void setSideMotorSetPoint(double position){
		//sideMotor_.set(speed);
		//sideEncoder_.set(0.2);
		sideMoter_.set(ControlMode.Position, (position/RobotMap.ENCODER_POSITION_ANGLE));
	}

	public double getSideEncoderPosition(){
		return sideMoter_.getSelectedSensorPosition(0) * RobotMap.ENCODER_POSITION_ANGLE;
	}
}

