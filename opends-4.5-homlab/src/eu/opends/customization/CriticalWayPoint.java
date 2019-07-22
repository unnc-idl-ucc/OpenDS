/*
 * Created by Yelly
 * on 27th July, 2017
 * This class defines structure of CriticalWayPoints.
 */

package eu.opends.customization;

import com.jme3.math.Vector3f;

public class CriticalWayPoint {
	private Vector3f pos; // global translation of the waypoint (in meters)
	private float width; // width of the road at this point (in meters)
	private float dts; //distance to start (in meters)
	
	public CriticalWayPoint(Vector3f pos, float width, float dts) {
		this.pos = pos;
		this.width = width;
		this.dts = dts;
	}
	
	public CriticalWayPoint() {
		this.pos = new Vector3f(0f, 0f, 0f);
		this.width = 0;
		this.dts = 0;
	}
	
	public Vector3f getPos() {
		return pos;
	}
	public void setPos(Vector3f pos) {
		this.pos = pos;
	}
	public float getWidth() {
		return width;
	}
	public void setWidth(float width) {
		this.width = width;
	}
	public float getDts() {
		return dts;
	}
	public void setDts(float dts) {
		this.dts = dts;
	}
	@Override
	public String toString() {
		return "CriticalWayPoint [pos=" + pos + ", width=" + width + ", dts=" + dts + "]";
	}
	
	
}
