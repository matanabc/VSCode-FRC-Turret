package vision.VisionClass;

public class VisionData {
	
	private double angleToTarget;
	private double pixelHeightToTarget;
	
	public VisionData(double angleToTarget, double pixelHeightToTarget) {
		this.angleToTarget = angleToTarget;
		this.pixelHeightToTarget = pixelHeightToTarget;
	}
	
	public double getTargetAngle() {
		return this.angleToTarget;
	}
	
	public double getTargetHeight() {
		return this.pixelHeightToTarget;
	}
	
	public void setTargetAngle(double angleToTarget){
		this.angleToTarget = angleToTarget;
	}
	
	public void setTargetHeight(double pixelHeightToTarget){
		this.pixelHeightToTarget = pixelHeightToTarget;
	}
}
