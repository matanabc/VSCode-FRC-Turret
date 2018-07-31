package vision.VisionClass;

import java.util.Calendar;
import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMaster{

	private int maxDecoderHestorySize;
	private double pixelToAngle;
	private double senterInPixel;
	private String NTValueName;
	private double lastTime;
	protected VisionAddHistoryTrhad addHistoryTrhad;
	private VisionData data;
	private double offSet;
	private VisionControllerInterface VCTilt;
	private VisionControllerInterface VCPan;
	private boolean singleTbothF;
	private  boolean panTtiltF;

	//private String chose;

	//Tree of history position from encoder
	private TreeMap<Long,Double> panHistory;
	private TreeMap<Long,Double> tiltHistory;


	/**
	 * @param 
	 * VC	-		this need to be vision controller you use for pan or tilt.
	 * 
	 * @param 
	 * maxDecoderHestorySize	-		this need to be max of history.
	 * 
	 * @param
	 * cameraAngle	-		this need to be the camera angle you use for vision. 
	 *
	 * @param 
	 * cameraWidth	-		this need to be the camera Width of pixel you use for vision.
	 * 
	 * @param 
	 * NTValueName	-		this need to be the name of the NetworkTable of the data from the vision.
	 * 
	 * @param
	 * offSet	-		this need to be the offset of the camera for the center. 
	 * 
	 * @param 
	 * panTtiltF	-		this need to true or false, true if you use vision controller for pan false if you use vision controller for tilt.
	 */
	public VisionMaster(VisionControllerInterface VC, int maxDecoderHestorySize, double cameraAngle, 
			double cameraWidth, String NTValueName, double offSet, boolean panTtiltF) {

		this.maxDecoderHestorySize= maxDecoderHestorySize;
		this.pixelToAngle = cameraAngle / cameraWidth; 
		this.senterInPixel = cameraWidth / 2;
		this.NTValueName = NTValueName;
		this.offSet = offSet;

		if(panTtiltF){
			this.VCPan = VC;
		}else{
			this.VCTilt = VC;
		}

		singleTbothF = true;
		this.panTtiltF = panTtiltF;

		panHistory = new TreeMap<Long,Double>();

		addHistoryTrhad = new VisionAddHistoryTrhad();
		startAddToHistoryTrhad();

		data = new VisionData(0, 0);
	}


	/**
	 * @param 
	 * VCPan	-		this need to be vision controller you use for pan.
	 * 
	 * @param 
	 * VCTilt	-		this need to be vision controller you use for tilt.
	 * 
	 * @param 
	 * maxDecoderHestorySize	-		this need to be max of history.
	 * 
	 * @param
	 * cameraAngle	-		this need to be the camera angle you use for vision. 
	 *
	 * @param 
	 * cameraWidth	-		this need to be the camera Width of pixel you use for vision.
	 * 
	 * @param 
	 * NTValueName	-		this need to be the name of the NetworkTable of the data from the vision.
	 * 
	 * @param
	 * offSet	-		this need to be the offset of the camera for the center. 
	 */
	public VisionMaster(VisionControllerInterface VCPan, VisionControllerInterface VCTilt, int maxDecoderHestorySize,
			double cameraAngle, double cameraWidth, String NTValueName, double offSeat) {

		this.maxDecoderHestorySize= maxDecoderHestorySize;
		this.pixelToAngle = cameraAngle / cameraWidth; 
		this.senterInPixel = cameraWidth / 2;
		this.NTValueName = NTValueName;
		this.offSet = offSeat;
		this.VCPan = VCPan;
		this.VCTilt = VCTilt;

		singleTbothF = false;

		panHistory = new TreeMap<Long,Double>();
		tiltHistory = new TreeMap<Long,Double>();

		addHistoryTrhad = new VisionAddHistoryTrhad();
		startAddToHistoryTrhad();

		data = new VisionData(0, 0);
	}

	//Don't change if you don't need to!!!
	public void addPanAndTiltPositionToHistory() {

		if(singleTbothF){

			if(panTtiltF){
				//Add position and time to history
				panHistory.put(Calendar.getInstance().getTimeInMillis(), VCPan.getSource());

				// remove the lowest key so he don't grow forever...
				if(panHistory.size() > this.maxDecoderHestorySize) {
					panHistory.remove(panHistory.firstKey());	
				}

			}else{
				//Add position and time to history
				tiltHistory.put(Calendar.getInstance().getTimeInMillis(), VCTilt.getSource());

				// remove the lowest key so he don't grow forever...
				if(tiltHistory.size() > this.maxDecoderHestorySize) {
					tiltHistory.remove(tiltHistory.firstKey());	
				}
			}

		}else{

			//Add position and time to history
			panHistory.put(Calendar.getInstance().getTimeInMillis(), VCPan.getSource());
			tiltHistory.put(Calendar.getInstance().getTimeInMillis(), VCTilt.getSource());

			// remove the lowest key so he don't grow forever...
			if(panHistory.size() > this.maxDecoderHestorySize) {
				panHistory.remove(panHistory.firstKey());	
			}

			// remove the lowest key so he don't grow forever...
			if(tiltHistory.size() > this.maxDecoderHestorySize) {
				tiltHistory.remove(tiltHistory.firstKey());	
			}
		}
	}

	//Don't change if you don't need to!!!
	protected double getPanPositionFromTime(Long time) {
		//check if is bigger then 0
		if(panHistory.size()==0) return 0;

		//Find the key value who closer to the time from camera  	
		Entry<Long, Double> panTime = panHistory.floorEntry(time);

		//If the key equals to null get the first key
		if(panTime == null) {
			panTime = panHistory.firstEntry();
		}

		//Return encoder position form the time
		return panTime.getValue();
	}

	protected double getTiltPositionFromTime(Long time) {
		//check if is bigger then 0
		if(tiltHistory.size()==0) return 0;

		//Find the key value who closer to the time from camera  	
		Entry<Long, Double> tiltTime = tiltHistory.floorEntry(time);

		//If the key equals to null get the first key
		if(tiltTime == null) {
			tiltTime = tiltHistory.firstEntry();
		}

		//Return encoder position form the time
		return tiltTime.getValue();
	}

	//Don't change if you don't need to!!!

	/**
	 * @return
	 * it will return you object of the data from the vision, 
	 * getTargetAngle() will give you the angle of the target, getTargetHeight() will give you the height of the target
	 */
	public VisionData getTargetPosition() {

		try{
			String [] TargetInfo = SmartDashboard.getString(this.NTValueName, "0;").split(";");

			/*
			 *	TargetInfo[0] - get the x pixel from the frame
			 *	TargetInfo[1] - get the x pixel from the frame
			 *	TargetInfo[2] - get the time when the frame was taking
			 */

			if(TargetInfo.length == 3){//if the data is came;
				if(Long.parseLong(TargetInfo[2]) > lastTime){//if the frame take after the last time

					lastTime = Double.parseDouble(TargetInfo[2]);//change the last time 

					setVisionData(TargetInfo);
				}
			}
			
			SmartDashboard.putString("Data vision error", "there was not error whet the data vision");
			
			return data;
			
		}catch (Exception e) {
			e.printStackTrace();
			
			SmartDashboard.putString("Data vision error", "there was a error whet the data vision, " + e.getMessage());
			
			return data;
		}
	}

	//Don't change if you don't need to!!!
	protected void startAddToHistoryTrhad(){
		new Thread(addHistoryTrhad).start();
	}	

	protected void setVisionData(String [] TargetInfo){

		if(singleTbothF){
			if(panTtiltF){
				this.data.setTargetAngle(

						(getPanPositionFromTime(Long.parseLong(TargetInfo[2])) //position when the frame was take

								- 	//need to check it it cold be - or +

								((Double.parseDouble(TargetInfo[0]) - this.senterInPixel) * this.pixelToAngle)) //angle error when the frame was take

						+

						offSet); //offSeat from target if camera is not in the center

				this.data.setTargetHeight(Double.parseDouble(TargetInfo[1])); //target Height in pixel

			}else{
				this.data.setTargetAngle(Double.parseDouble(TargetInfo[0])); //offSeat from target if camera is not in the center

				this.data.setTargetHeight(VCTilt.castYpixel(Double.parseDouble(TargetInfo[1]),
						getTiltPositionFromTime(Long.parseLong(TargetInfo[2]))));

			}

		} else {

			this.data.setTargetAngle(

					(getPanPositionFromTime(Long.parseLong(TargetInfo[2])) //position when the frame was take

							- 	//need to check it it cold be - or +

							((Double.parseDouble(TargetInfo[0]) - this.senterInPixel) * this.pixelToAngle)) //angle error when the frame was take

					+

					offSet); //offSeat from target if camera is not in the center

			this.data.setTargetHeight(VCTilt.castYpixel(Double.parseDouble(TargetInfo[1]),
					getTiltPositionFromTime(Long.parseLong(TargetInfo[2]))));

		}
	}
}
