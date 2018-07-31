package LogFile;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Calendar;
import java.util.LinkedList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.RobotMap;

public class WriteToFile implements Runnable{

	protected FileWriter fw;
	protected BufferedWriter bw;

	protected LinkedList<String> writeToFile;
	
	protected String timeStatus;
	protected long statusStart;

	public WriteToFile(LinkedList<String> writeToFile) {
		
		timeStatus = "Disabled";
		statusStart = Calendar.getInstance().getTimeInMillis();

		this.writeToFile = writeToFile;

		startToWrite();
	}

	@Override
	public void run() {

		while(bw != null) {
			if(!writeToFile.isEmpty()) {
				if(writeToFile.getFirst().equals("clean")) {
					cleanFile();
				}
				writeToFile(writeToFile.removeFirst());
			}
			
			resetTime();
		}
	}

	protected void writeToFile(String write) {
		try {
			bw.write(write + getTime());
			bw.newLine();
			bw.flush();
		}catch (Exception e) {
			e.printStackTrace();
		}
	}

	protected void startToWrite() {
		try {
			
			cleanFile();
			
			fw = new FileWriter(RobotMap.FILE_PLACE, true);
			bw = new BufferedWriter(fw);

			writeToFile("Robot is on");

			new Thread(this).start();
			
			SmartDashboard.putString("Log File Masage: ", "Starting to write");

		}catch (Exception e) {
			e.printStackTrace();

			SmartDashboard.putString("Log File Masage: ", "Error: " + e.getMessage());
		}
	}

	protected void cleanFile() {
		try {
			fw = new FileWriter(RobotMap.FILE_PLACE);
			bw = new BufferedWriter(fw);

			writeToFile("Robot is on");

			fw = new FileWriter(RobotMap.FILE_PLACE, true);
			bw = new BufferedWriter(fw);
		}catch (Exception e) {
			e.printStackTrace();
		}
	}

	protected void stopToWrite() {
		try {
			bw.close();
			fw.close();
		}catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	protected String getTime(){
		return " " + Long.toString((Calendar.getInstance().getTimeInMillis() - statusStart) / 1000) + " seconds from start " + timeStatus;
	}
	
	protected void resetTime(){
		if (DriverStation.getInstance().isAutonomous() && !timeStatus.equals("Auto")) {
			timeStatus = "Auto";
			statusStart = Calendar.getInstance().getTimeInMillis();
		}else if (DriverStation.getInstance().isOperatorControl() && !timeStatus.equals("tele")) {
			timeStatus = "tele";
			statusStart = Calendar.getInstance().getTimeInMillis();
		}else if (DriverStation.getInstance().isDisabled() && !timeStatus.equals("Disabled")) {
			timeStatus = "Disabled";
			statusStart = Calendar.getInstance().getTimeInMillis();
		}
	}
}
