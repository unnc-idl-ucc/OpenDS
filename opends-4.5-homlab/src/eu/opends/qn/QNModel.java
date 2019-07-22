package eu.opends.qn;

import java.util.HashMap;

import eu.opends.customization.CriticalElement;
public class QNModel {
	
	private float OpenDSClock;
	// variables related to near point/ far point
	private float nearPointAngle;
	private float farPointAngle;
	private float farPointDistance;
	private float speed;
	// variables related to critical elements
	private HashMap<String, CriticalElement> criticalElements;

	public QNModel(float openDSClock, float nearPointAngle, float farPointAngle, float farPointDistance, float speed,
			HashMap<String, CriticalElement> criticalElements) {
		OpenDSClock = openDSClock;
		this.nearPointAngle = nearPointAngle;
		this.farPointAngle = farPointAngle;
		this.farPointDistance = farPointDistance;
		this.speed = speed;
		this.criticalElements = criticalElements;
	}
	
	public HashMap<String, CriticalElement> getCriticalElements() {
		return criticalElements;
	}
	public void setCriticalElements(HashMap<String, CriticalElement> criticalElements) {
		this.criticalElements = criticalElements;
	}
	public float getOpenDSClock() {
		return OpenDSClock;
	}
	public void setOpenDSClock(float openDSClock) {
		OpenDSClock = openDSClock;
	}
	public float getSpeed() {
		return speed;
	}
	public void setSpeed(float speed) {
		this.speed = speed;
	}
	public float getNearPointAngle() {
		return nearPointAngle;
	}
	public void setNearPointAngle(float nearPointAngle) {
		this.nearPointAngle = nearPointAngle;
	}
	public float getFarPointAngle() {
		return farPointAngle;
	}
	public void setFarPointAngle(float farPointAngle) {
		this.farPointAngle = farPointAngle;
	}
	public float getFarPointDistance() {
		return farPointDistance;
	}
	public void setFarPointDistance(float farPointDistance) {
		this.farPointDistance = farPointDistance;
	}


	@Override
	public String toString() {
		return "QNModel [OpenDSClock=" + OpenDSClock + ", nearPointAngle=" + nearPointAngle + ", farPointAngle="
				+ farPointAngle + ", farPointDistance=" + farPointDistance + ", speed=" + speed + ", criticalElements="
				+ criticalElements + "]";
	}

	public String toSendToQN() {
		return "QNModel [OpenDSClock=" + OpenDSClock + "; nearPointAngle=" + nearPointAngle + "; farPointAngle="
				+ farPointAngle + "; farPointDistance=" + farPointDistance + "; speed=" + speed + "; criticalElements="
				+ criticalElements + "]";
	}
}
