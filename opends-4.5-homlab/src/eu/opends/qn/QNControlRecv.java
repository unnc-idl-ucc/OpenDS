package eu.opends.qn;

public class QNControlRecv {
	private float QNClock;
	private float Accelerator;
	private float Brake;
	private float Steering;
	private boolean report;
	
	public boolean isReport() {
		return report;
	}
	public void setReport(boolean report) {
		this.report = report;
	}
	public float getQNClock() {
		return QNClock;
	}
	public void setQNClock(float qNClock) {
		QNClock = qNClock;
	}
	public float getAccelerator() {
		return Accelerator;
	}
	public void setAccelerator(float accelerator) {
		Accelerator = accelerator;
	}
	public float getBrake() {
		return Brake;
	}
	public void setBrake(float brake) {
		Brake = brake;
	}
	public float getSteering() {
		return Steering;
	}
	public void setSteering(float steering) {
		Steering = steering;
	}
	@Override
	public String toString() {
		return "QNControlRecv [QNClock=" + QNClock + ", Accelerator=" + Accelerator + ", Brake=" + Brake + ", Steering="
				+ Steering + ", report=" + report + "]";
	}
	
}
