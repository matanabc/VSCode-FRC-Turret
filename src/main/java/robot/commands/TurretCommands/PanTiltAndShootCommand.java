package robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import vision.VisionClass.VisionControllerInterface;
import vision.VisionClass.VisionMaster;
import vision.VisionCommands.VisionCommand;

/**
 *
 */
public class PanTiltAndShootCommand extends CommandGroup {

    public PanTiltAndShootCommand(VisionMaster VM, VisionControllerInterface VCPan, VisionControllerInterface VCTilt,
    								VisionControllerInterface VCShoot) {
       
    	addParallel(new VisionCommand(VM, VCPan, true, false));
        addParallel(new VisionCommand(VM, VCTilt, false, false));
        addSequential(new VisionCommand(VM, VCShoot, false, false));
    }
}
