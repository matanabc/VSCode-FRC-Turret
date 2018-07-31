/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import robot.commands.TurretCommands.PanTiltAndShootCommand;
import robot.commands.TurretCommands.ResetTalonsEncodersCommand;
import robot.commands.drive.DriveSpeedCommandWrite;
import vision.VisionCommands.VisionCommand;
import vision.VisionControllers.PanVisionController;
import vision.VisionControllers.TurretPanVisionController;
import vision.VisionControllers.TurretShooterVisionController;
import vision.VisionControllers.TurretTiltVisionController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public Joystick AdelStick = new Joystick(RobotMap.ADEL_JOYSTICK_CHANEL);

	public Button[] AdelBtns = new Button[12];
	
	public PanVisionController VC = new PanVisionController();
	
	public TurretPanVisionController TPVC = new TurretPanVisionController();
	public TurretTiltVisionController TTVC = new TurretTiltVisionController();
	public TurretShooterVisionController TSVC = new TurretShooterVisionController();

	public OI(){
		for (int i = 0; i < 12; i ++){
			AdelBtns[i] = new JoystickButton(AdelStick, i + 1);
		}
	}

	public void loadOIs(){		

		//Adel turn slow speed 
		AdelBtns[5].whileHeld(new DriveSpeedCommandWrite(-RobotMap.DRIVE_SLOW, RobotMap.DRIVE_SLOW));//Turn right
		AdelBtns[4].whileHeld(new DriveSpeedCommandWrite(RobotMap.DRIVE_SLOW, -RobotMap.DRIVE_SLOW));//Turn left
		
		AdelBtns[0].whileHeld(new PanTiltAndShootCommand(Robot.VM, TPVC, TTVC, TSVC));
		
		AdelBtns[1].whileHeld(new ResetTalonsEncodersCommand());

		AdelBtns[2].whileHeld(new VisionCommand(Robot.VM, VC, true, true));

	}
}
