package vision.VisionClass;

import robot.Robot;

public class VisionAddHistoryTrhad  implements Runnable{

	@Override
	public void run() {
		while(true){
			//Robot.vision.addEncoderPositionToHistory();
			if(Robot.VM != null){
				Robot.VM.addPanAndTiltPositionToHistory();
			}else{
				System.out.println("Vision master is null!!!");
			}
		}
	}

}
