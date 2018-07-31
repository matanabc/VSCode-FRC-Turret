package MotionProfiling.MP_Commands;

import java.util.ArrayList;

import MotionProfiling.MP_Constants;
import MotionProfiling.MP_Classes.MPGains;
import MotionProfiling.MP_Classes.MP_DrivePath;
import MotionProfiling.MP_Classes.MP_R2S;
import MotionProfiling.MP_Classes.MP_Radius;
import MotionProfiling.MP_Classes.MP_S2R;
import MotionProfiling.MP_Classes.MP_Straight;
import MotionProfiling.MP_Classes.MP_Transition;
import MotionProfiling.MP_Controllers.MPDriveController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class MP_AutoGenerator extends CommandGroup{
	
	private class Node{
		public Command command;
		public char type;
		public double timeout;
		
		public Node(Command com, char type) {
			this.type = type;
			this.command = com;
			this.timeout = 0;
		}
		
		public Node(Command com, char type, double timeout) {
			this.type = type;
			this.command = com;
			this.timeout = timeout;
		}
	}
	
	MPDriveController dc_;
	MPGains gains_;
	private ArrayList<Node> list_ = new ArrayList<Node>();
	
	
	public void setDefaultDriveControllerAndGains(MPDriveController dc, MPGains gains){
		dc_ = dc;
		gains_ = gains;
	}

	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * V0  		-			V0 of the move.
	 * 						except of :
	 * 							if Vend of last move is 0 	: 	V0 will be 0 anyway. 
	 * 							if last move is straight	: 	V0 will be last move Vend.
	 * 							if minus 				 	: 	V0 will be last move Vend.
	 * @param
	 * Vmax 	- 			maximum speed.
	 * @param
	 * Vend 	- 			endling speed.
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addStraight(double distance, double V0, double Vmax, double Vend, double exelAcc, double stopAcc,
			MPDriveController dc, MPGains gains)
			throws Exception{
		int lastDrivePathIndex = getLastDriveCommand();
		MP_DrivePath preMove;
		MP_Straight newMove;
		
		Vmax 	*= MP_Constants.MAX_SPEED;
		Vend 	*= MP_Constants.MAX_SPEED;
		V0 		*= MP_Constants.MAX_SPEED;
		
		exelAcc *= MP_Constants.MAX_EXELRATION_ACC;
		stopAcc *= MP_Constants.MAX_STOPPING_ACC;
		
		if (lastDrivePathIndex == -1){
			newMove = new MP_Straight(distance, Vmax, 0, Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "your move is not legal";
				
				throw new Exception(errMes);
			}
			
			retzef(new MP_DrivePathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		preMove = ((MP_DrivePathFollower) list_.get(lastDrivePathIndex).command).getPath();
				
		if (preMove.getVend() == 0){
			newMove = new MP_Straight(distance, Vmax, 0, Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "your move is not legal";
				
				throw new Exception(errMes);
			}
			
			retzef(new MP_DrivePathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		if (preMove instanceof MP_Straight){
			newMove = new MP_Straight(
					distance, Vmax, preMove.getVend(), Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "Considuring the previus Vend your move is not legal."
						+ "Try to decrease the deltaV between the two moves.";
				
				throw new Exception(errMes);
			}
			
			retzef(new MP_DrivePathFollower(
					dc, 
					newMove, 
					gains));
			
		} else if (preMove instanceof MP_Radius){
			if (V0 < 0) {
				/*
				 * Because this is straight after radius if we will put the same relation
				 * between the maximum speeds the speed will increase during the transition
				 * and because exelAcc is less then stopAcc the transition will take more time.
				 * That is why here we put the exact same speed. 
				 */
				V0 = preMove.getVend();
			}
			
			MP_R2S trans = new MP_R2S(((MP_Radius) preMove).getRadius(),
					preMove.getVend(), V0,
					exelAcc, stopAcc);
			
			double transAngle = trans.getTotalAngleDegrees();
			double preMoveAngle = ((MP_Radius) preMove).getTotalAngleDegrees();
			
			if (Math.abs(transAngle) > Math.abs(preMoveAngle)){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The transition angle is more then the radius before\n"
						+ "Please try one of those options : \n"
						+ "\t- increase radius\n"
						+ "\t- decrease radius Vend\n";
				
				throw new Exception(errMes);
			}
			
			MP_Radius newRadius = new MP_Radius(
					((MP_Radius) preMove).getRadius(),
					preMoveAngle - transAngle,
					preMove.getVmax(),
					preMove.getV0(),
					preMove.getVend(),
					preMove.getExelerationAcc(),
					preMove.getStopingAcc());
			
			if (!newRadius.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The previus Radius modification is not legal"
						+ "Please try one of those options : \n"
						+ "\t- increase radius\n"
						+ "\t- decrease radius Vend\n";
				
				throw new Exception(errMes);
			}
			
			// preperring the new move
			newMove = new MP_Straight(
					distance, Vmax, V0, Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + "\n"
						+ "The new move is not legal";
				
				throw new Exception(errMes);
			}
			
			// changing the radius before to match the transition
			list_.set(lastDrivePathIndex
					, new Node(new MP_DrivePathFollower(dc, newRadius, gains), 's'));
			
			//adding the transition afters
			retzef(new MP_DrivePathFollower(dc, trans, gains));
			
			//adding the new Move
			retzef(new MP_DrivePathFollower(dc, newMove, gains));
		}
	}
	
	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * V0  		-			V0 of the move.
	 * 						except of :
	 * 							if Vend of last move is 0 	: 	V0 will be 0 anyway. 
	 * 							if last move is straight	: 	V0 will be last move Vend.
	 * 							if minus 				 	: 	V0 will be last move Vend.
	 * @param
	 * Vmax 	- 			maximum speed.
	 * @param
	 * Vend 	- 			endling speed.
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 */
	public void addStraight(double distance, double V0, double Vmax, double Vend, double exelAcc, double stopAcc)
			throws Exception{
		addStraight(distance, V0, Vmax, Vend, exelAcc, stopAcc, dc_, gains_);
	}
	
	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * Vend 	- 			endling speed.
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addStraight(double distance, double Vend, MPDriveController dc,
			MPGains gains) throws Exception{
		addStraight(distance, -1, 1, Vend, 1, 1, dc, gains); // -1 in V0 means last Vend
	}
	
	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * Vend 	- 			endling speed.
	 */
	public void addStraight(double distance, double Vend) throws Exception{
		addStraight(distance, Vend, dc_, gains_);
	}
	
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * V0		-		V0 of the move
	 * 					except for :
	 * 						if V0 of last move is 0		:	V0 will be 0.
	 * 						if minus 					: 	V0 will be last move Vend.	
	 * @param
	 * Vmax 	- 		Vmax of the MIDDLE of the robot!
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addRadius(double radius, double angle, double V0, double Vmax, double Vend,
			double exelAcc, double stopAcc, MPDriveController dc, MPGains gains)
					throws Exception{
		int lastDrivePathIndex = getLastDriveCommand();
		MP_DrivePath preMove;
		MP_DrivePath newMove;
		
		Vmax 	*= MP_Radius.getMaxSpeedForRadius(radius);
		Vend	*= MP_Radius.getMaxSpeedForRadius(radius);
		V0		*= MP_Radius.getMaxSpeedForRadius(radius);
		
		double radiusExelAcc = exelAcc * MP_Radius.getMaxAcc(MP_Constants.MAX_EXELRATION_ACC, radius);
		double radiusStopAcc = stopAcc * MP_Radius.getMaxAcc(MP_Constants.MAX_STOPPING_ACC, radius);
		
		
		if (lastDrivePathIndex == -1){
			newMove = new MP_Radius(radius, angle, Vmax, 0, Vend, radiusExelAcc, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The move you tryed to do is not legal";
				throw new Exception(errMes);
			}
			
			retzef(new MP_DrivePathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		preMove = ((MP_DrivePathFollower) list_.get(lastDrivePathIndex).command).getPath();
				
		if (preMove.getVend() == 0){
			newMove = new MP_Radius(radius, angle, Vmax, 0, Vend, radiusExelAcc, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The move you tryed to do is not legal";
				throw new Exception(errMes);
			}
			
			retzef(new MP_DrivePathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		if (preMove instanceof MP_Straight){
			if (V0 < 0) {
				// the Vend related to the maximum speed (from 0 to 1)
				V0 = preMove.getVend() / MP_Constants.MAX_SPEED;
				
				// the related speed for the radius
				V0 *= MP_Radius.getMaxSpeedForRadius(radius);
			}
			
			MP_S2R trans = new MP_S2R(	radius,
										preMove.getVend(),
										V0,
										exelAcc * MP_Constants.MAX_EXELRATION_ACC,
										stopAcc * MP_Constants.MAX_STOPPING_ACC);

			double transAngle = trans.getTotalAngleDegrees();

			if (Math.abs(angle) < Math.abs(transAngle)){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The transition angle is more then the radius angle\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (radius Vend)\n"
						+ "\t- increase radius\n";
				
				throw new Exception(errMes);
			}
			
			newMove = new MP_Radius(
					radius
					, angle - transAngle
					, Vmax
					, V0
					, Vend
					, radiusExelAcc
					, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The new radius (considuring the transmition angle) is not legal\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (radius Vend)\n"
						+ "\t- increase radius\n";
				
				throw new Exception(errMes);
			}
			
			retzef(new MP_DrivePathFollower(dc, trans, gains));
			retzef(new MP_DrivePathFollower(dc, newMove, gains));
			
		} else if (preMove instanceof MP_Radius){
			if (!(radius > 0 ^ ((MP_Radius) preMove).getRadius() > 0)){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "You cant combine two radiuses in the same direction";
				
				throw new Exception(errMes);	
			}
			
			if (V0 < 0) {
				V0 = preMove.getVend();
			}
			
			MP_R2S first = new MP_R2S(	((MP_Radius) preMove).getRadius(),
										preMove.getVend(),
										(preMove.getVend() + V0) / 2,
										exelAcc * MP_Constants.MAX_EXELRATION_ACC,
										stopAcc * MP_Constants.MAX_STOPPING_ACC);
			
			MP_S2R second = new MP_S2R(	radius, 
										(preMove.getVend() + V0) / 2, 
										V0,
										exelAcc * MP_Constants.MAX_EXELRATION_ACC,
										stopAcc * MP_Constants.MAX_STOPPING_ACC);
			
			double firstAngle = first.getTotalAngleDegrees();
			double secondAngle = second.getTotalAngleDegrees();
			double lastMoveAngle = Math.abs(preMove.getTotalAngleDegrees());

			System.out.println("1");
			
			
			if (lastMoveAngle < firstAngle){
				if (angle < firstAngle + secondAngle){
					String errMes = "Cant make this move : addRadius(" + radius
							+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
							+ "the angle of the previus move is less then the transmition angle.\n"
							+ "Please try one of those options : \n"
							+ "\t- decrease transition speed (radius Vend)\n"
							+ "\t- increase radius\n";
					
					throw new Exception(errMes);
				}
				
			} else if (angle < secondAngle){
				if (lastMoveAngle < firstAngle + secondAngle){
					String errMes = "Cant make this move : addRadius(" + radius
							+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
							+ "the angle of the mew move is less then the transmition angle.\n"
							+ "Please try one of those options : \n"
							+ "\t- decrease transition speed (radius Vend)\n"
							+ "\t- increase radius\n";
					
					throw new Exception(errMes);
				}
			}
			MP_Radius updatePreMove = new MP_Radius(((MP_Radius) preMove).getRadius()
					, preMove.getTotalAngleDegrees() - firstAngle
					, preMove.getVmax()
					, preMove.getV0()
					, preMove.getVend()
					, preMove.getExelerationAcc()
					, preMove.getStopingAcc());
			
			if (!updatePreMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The modification of the previus radius (considuring the transmition angle) is not legal\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (radius Vend)\n"
						+ "\t- increase radius\n";
				
				throw new Exception(errMes);
			}
			
			newMove = new MP_Radius(radius
					, angle - secondAngle
					, Vmax
					, V0
					, Vend
					, radiusExelAcc
					, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The new radius (considuring the transmition angle) is not legal\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (previus radius Vend)\n"
						+ "\t- increase radius (new radius)\n";
				
				throw new Exception(errMes);
			}
			
			// changing the radius before to match the transition
			list_.set(lastDrivePathIndex
					, new Node(new MP_DrivePathFollower(dc, updatePreMove, gains), 's')); // need to be fixed
			
			// adding the transmitions
			retzef(new MP_DrivePathFollower(dc, first, gains));
			retzef(new MP_DrivePathFollower(dc, second, gains));
			
			//adding the new move
			retzef(new MP_DrivePathFollower(dc, newMove, gains));
		}
	}
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * V0		-		V0 of the move
	 * 					except for :
	 * 						if V0 of last move is 0		:	V0 will be 0.
	 * 						if minus 					: 	V0 will be last move Vend.	
	 * @param
	 * Vmax 	- 		Vmax of the MIDDLE of the robot!
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 */
	public void addRadius(double radius, double angle, double V0, double Vmax, double Vend,
			double exelAcc, double stopAcc) throws Exception{
		addRadius(radius, angle, V0, Vmax, Vend, exelAcc, stopAcc, dc_, gains_);
	}
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addRadius(double radius, double angle, double Vend,
			MPDriveController dc, MPGains gains) throws Exception{
		
		addRadius(radius, angle, -1, 1, Vend, 1, 1, dc, gains); // -1 in V0 means last move Vend
	}
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 */
	public void addRadius(double radius, double angle, double Vend) throws Exception{
		addRadius(radius, angle, Vend, dc_, gains_);
	}
	
	/**
	 * returns the index of the last drive command.	
	 * @return
	 * the index of the last drive command.
	 * if there is no such command, returns (-1).
	 */
	private int getLastDriveCommand(){
		for (int i = list_.size() - 1; i >= 0; i --){
			if (list_.get(i).command instanceof MP_DrivePathFollower){
				return i;
			}
		}
		return -1;
	}
	
	public void retzef(Command command, double timeout) throws Exception{
		int lastMoveIndex = getLastDriveCommand();
		if (!(command instanceof MP_DrivePathFollower) && lastMoveIndex >= 0){
			MP_DrivePathFollower temp= (MP_DrivePathFollower)(list_.get(lastMoveIndex).command);
			if (temp.getPath().getVend() != 0){
				String errMes = "Cant add sequentioal command if robot is not in a stop\n";
				throw new Exception(errMes);
			}
		}
		
		list_.add(new Node(command, 's', timeout));
	}
	
	public void retzef(Command command) throws Exception{
		retzef(command, 0);
	}
	
	public void makbil(Command command, double timeout){
		list_.add(new Node(command, 'p', timeout));
	}
	
	public void makbil(Command command){
		makbil(command, 0);
	}
	
	public void makbil2Previus(Command command, double timeout) throws Exception{
		int i;
		for (i = list_.size() - 1; i >= 0; i--){
			if (list_.get(i).type == 's'){
				break;
				// i is now the index of the last sequential command
			}
		}
		
		if (i < 0){
			String errMes = "Cant add perller to previus command becouse there is no such command\n";
			throw new Exception(errMes);
		}
		
		// adding the perlar command to be before the last sequential command
		list_.add(i, new Node(command, 'p', timeout));
	}
	
	public void makbil2Previus(Command command) throws Exception{
		makbil2Previus(command, 0);
	}
	
	/**
	 *  generates auto by addning all the commands in seq and parl.
	 */
	public void generateAuto(){
		for (int i = 0; i < list_.size(); i ++){
			Node temp = list_.get(i);
			if (temp.type == 's'){
				if (temp.timeout != 0){
					super.addSequential(temp.command, temp.timeout);
				} else {
					super.addSequential(temp.command);
				}
			} else if (temp.type == 'p'){
				if (temp.timeout != 0){
					super.addParallel(temp.command, temp.timeout);
				} else {
					super.addParallel(temp.command);
				}
			}else {
				System.out.println("Error : type is not s/p");
			}
		}
	}
	
	/**
	 *  call the toString method for each driving command
	 */
	public void printMoves(){
		System.out.println("Print Moves");
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_DrivePathFollower){
				System.out.println(((MP_DrivePathFollower)temp).getPath().toString());
			}
		}
	}
	
	/**
	 * needs to print the moves in simulator format
	 */
	public void printMovesSimulator(){
		double totalTime = 0;
		//System.out.println("Moves for Simulator : ");
		System.out.println("1.06");
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_DrivePathFollower){
				totalTime += ((MP_DrivePathFollower)temp).getPath().getTotalTime();
				MP_DrivePath temp2 = ((MP_DrivePathFollower) temp).getPath();
				if (temp2 instanceof MP_Transition){
					System.out.printf("t,%.20f,%.20f,%.20f", ((MP_Transition)temp2).getTotalX()
							,((MP_Transition)temp2).getTotalY()
							,((MP_Transition)temp2).getTotalAngleDegreesWithSignIndicatingLeftOrRight());
					System.out.println();
				} else if (temp2 instanceof MP_Radius){
					System.out.printf("r,%.20f,%.20f",((MP_Radius)temp2).getRadius(),((MP_Radius)temp2).getTotalAngleDegrees());
					System.out.println();
				} else if (temp2 instanceof MP_Straight){
					System.out.printf("s,%.20f", ((MP_Straight)temp2).getDistance());
					System.out.println();
				} 
			}
		}
		System.out.println(totalTime);
	}
	
	/**
	 *  prints and returns the last drive command total time
	 * @return
	 * last move total time
	 */
	public double lastMoveTotalTime(){
		System.out.println("Last Move Total Time :");
		int lastMoveIndex = getLastDriveCommand();
		if (lastMoveIndex == -1){
			System.out.println("there is no last move!");
			return 0;
		}
		
		double totalTime = ((MP_DrivePathFollower)list_.get(lastMoveIndex).command)
				.getPath().getTotalTime();
		System.out.println(totalTime);
		return totalTime;
	}

	/**
	 * returns a list of the drive paths in this auto
	 * @return
	 * list of the drive paths in this auto
	 */
	public ArrayList<MP_DrivePath> getPathArray(){
		ArrayList<MP_DrivePath> ret = new ArrayList<MP_DrivePath>();
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_DrivePathFollower){
				ret.add(((MP_DrivePathFollower) temp).getPath());
			}
		}
		return ret;
	}
	
}
