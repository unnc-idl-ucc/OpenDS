/*
*  This file is part of OpenDS (Open Source Driving Simulator).
*  Copyright (C) 2016 Rafael Math
*
*  OpenDS is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  OpenDS is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
*/

package eu.opends.car;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.jme3.collision.CollisionResults;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.Geometry;

import eu.opends.analyzer.DrivingTaskLogger;
import eu.opends.basics.MapObject;
import eu.opends.basics.SimulationBasics;
import eu.opends.camera.CameraFactory;
import eu.opends.car.LightTexturesContainer.TurnSignalState;
import eu.opends.customization.CriticalElement;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.scenario.ScenarioLoader;
import eu.opends.drivingTask.scenario.ScenarioLoader.CarProperty;
import eu.opends.drivingTask.settings.SettingsLoader;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.environment.Crosswind;
import eu.opends.environment.TrafficLightCenter;
import eu.opends.main.SimulationDefaults;
import eu.opends.main.Simulator;
import eu.opends.qn.QNCenter;
import eu.opends.qn.QNControlRecv;
import eu.opends.qn.QNLogger;
import eu.opends.qn.QNModel;
import eu.opends.simphynity.SimphynityController;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.Util;
import eu.opends.traffic.FollowBox;
import eu.opends.traffic.FollowBoxSettings;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficCar;
import eu.opends.traffic.TrafficObject;
import eu.opends.traffic.Waypoint;
import eu.opends.trafficObjectLocator.TrafficObjectLocator;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;

/**
 * Driving Car
 * 
 * @author Rafael Math
 */
public class SteeringCar extends Car implements TrafficObject
{
	// minimum steering percentage to be reached for switching off the turn signal automatically
	// when moving steering wheel back towards neutral position
	private float turnSignalThreshold = 0.25f;
	
    private TrafficObjectLocator trafficObjectLocator;
    private boolean handBrakeApplied = false;
    
    // Simphynity Motion Seat
    private SimphynityController simphynityController;
    
    // adaptive cruise control
	private boolean isAdaptiveCruiseControl = false;
	private float minLateralSafetyDistance;
	private float minForwardSafetyDistance;
	private float emergencyBrakeDistance;
	private boolean suppressDeactivationByBrake = false;
	
	// crosswind (will influence steering angle)
	private Crosswind crosswind = new Crosswind("left", 0, 0);
	
	private FollowBox followBox;
	
	private boolean isAutoPilot;

    // added by Yelly
    /*private DistanceToStartUtil dts;
	public DistanceToStartUtil getDts()
	{
		return dts;
	}*/
	
	// added by Yelly£¬for debug
	//private int dtscnt = 1;
	
	// added by Yelly, for near point far point visualization
	private boolean visualizeNFPoints = false;
	private Spatial visualizedNearPoint = null;
	private Spatial visualizedFarPoint = null;
	private int counter = 0;
	private Vector3f farPointPos;
	private Vector3f nearPointPos;
	
	private final String visualizedNearPointModelPath = "Models/Yelly/sphere-red/ball.scene";
	private final String visualizedFarPointModelPath = "Models/Yelly/sphere-blue/ball.scene";
	private final float visualizedNearPointModelAngles[] = new Quaternion().fromAngles(
			270 * FastMath.DEG_TO_RAD,
			-90 * FastMath.DEG_TO_RAD, 
			0 * FastMath.DEG_TO_RAD).toAngles(null);
	private final float visualizedFarPointModelAngles[] = new Quaternion().fromAngles(
			270 * FastMath.DEG_TO_RAD,
			-90 * FastMath.DEG_TO_RAD, 
			0 * FastMath.DEG_TO_RAD).toAngles(null);
	private final Vector3f visualizedFarPointModelScale = new Vector3f(1f,1f,1f);
	private final Vector3f visualizedNearPointModelScale = new Vector3f(1f,1f,1f);
	
	// added by Yelly
	private boolean useQN = false;   
	private QNCenter qn;
	private long initialClockStamp; // in millis
	private boolean initStat = true; // true if the model runs in initial state (just accelerate without taking commands from QN-ACTR)
	private static QNLogger qnLogger;
	public static QNLogger getQnLogger()
	{
		return qnLogger;
	}
	
	// rotate steering wheel
	Vector3f rot_point = new Vector3f(2.0f, 2.0f, 2.0f);
    float angleST = 0;
    Quaternion initialPositionSteering = new Quaternion();
    Quaternion rotationSteering = new Quaternion();
	
    
    
    //for connection to QN driving model
    private float initalAccDuration =  3.0f; // in second
    
	// added by Yelly
    // the list of critical elements (possibly) visible to the driver
    //suppose each element has its unique name (id)
    HashMap<String, CriticalElement> criticalElements = new HashMap<String, CriticalElement>();
    HashMap<String, Vector3f> visible_ce = new HashMap<String, Vector3f>(); // critical elements that the model has seen so far
    
    //float visible_angle = 160.0f; // in degree
    								// visible_angle is set according to frustum angle
    float visible_dist = 200.0f; // in meter
    								// visible_dist is different from frustum far dist
    								// because I suppose driver could not see as far as ...
    float visible_angle;
    boolean visible_angle_initialized = false;
    
	public SteeringCar(Simulator sim) 
	{		
		this.sim = sim;
		
		
		DrivingTask drivingTask = SimulationBasics.getDrivingTask();
		ScenarioLoader scenarioLoader = drivingTask.getScenarioLoader();
		
		initialPosition = scenarioLoader.getStartLocation();
		if(initialPosition == null)
			initialPosition = SimulationDefaults.initialCarPosition;
		
		this.initialRotation = scenarioLoader.getStartRotation();
		if(this.initialRotation == null)
			this.initialRotation = SimulationDefaults.initialCarRotation;
			
		// add start position as reset position
		Simulator.getResetPositionList().add(new ResetPosition(initialPosition,initialRotation));
		
		mass = scenarioLoader.getChassisMass();
		
		minSpeed = scenarioLoader.getCarProperty(CarProperty.engine_minSpeed, SimulationDefaults.engine_minSpeed);
		maxSpeed = scenarioLoader.getCarProperty(CarProperty.engine_maxSpeed, SimulationDefaults.engine_maxSpeed);
			
		decelerationBrake = scenarioLoader.getCarProperty(CarProperty.brake_decelerationBrake, 
				SimulationDefaults.brake_decelerationBrake);
		maxBrakeForce = 0.004375f * decelerationBrake * mass;
		
		decelerationFreeWheel = scenarioLoader.getCarProperty(CarProperty.brake_decelerationFreeWheel, 
				SimulationDefaults.brake_decelerationFreeWheel);
		maxFreeWheelBrakeForce = 0.004375f * decelerationFreeWheel * mass;
		
		engineOn = scenarioLoader.getCarProperty(CarProperty.engine_engineOn, SimulationDefaults.engine_engineOn);
		if(!engineOn)
			showEngineStatusMessage(engineOn);
		
		Float lightIntensityObj = scenarioLoader.getCarProperty(CarProperty.light_intensity, SimulationDefaults.light_intensity);
		if(lightIntensityObj != null)
			lightIntensity = lightIntensityObj;
		
		transmission = new Transmission(this);
		powerTrain = new PowerTrain(this);
		
		modelPath = scenarioLoader.getModelPath();
		
		init();

        // allows to place objects at current position
        trafficObjectLocator = new TrafficObjectLocator(sim, this);
        
        // load settings of adaptive cruise control
        isAdaptiveCruiseControl = scenarioLoader.getCarProperty(CarProperty.cruiseControl_acc, SimulationDefaults.cruiseControl_acc);
    	minLateralSafetyDistance = scenarioLoader.getCarProperty(CarProperty.cruiseControl_safetyDistance_lateral, SimulationDefaults.cruiseControl_safetyDistance_lateral);
    	minForwardSafetyDistance = scenarioLoader.getCarProperty(CarProperty.cruiseControl_safetyDistance_forward, SimulationDefaults.cruiseControl_safetyDistance_forward);
    	emergencyBrakeDistance = scenarioLoader.getCarProperty(CarProperty.cruiseControl_emergencyBrakeDistance, SimulationDefaults.cruiseControl_emergencyBrakeDistance);
    	suppressDeactivationByBrake = scenarioLoader.getCarProperty(CarProperty.cruiseControl_suppressDeactivationByBrake, SimulationDefaults.cruiseControl_suppressDeactivationByBrake);
    	
    	// if initialSpeed > 0 --> cruise control will be on at startup
    	targetSpeedCruiseControl = scenarioLoader.getCarProperty(CarProperty.cruiseControl_initialSpeed, SimulationDefaults.cruiseControl_initialSpeed);
		isCruiseControl = (targetSpeedCruiseControl > 0);
    	
		SettingsLoader settingsLoader = SimulationBasics.getSettingsLoader();
        if(settingsLoader.getSetting(Setting.Simphynity_enableConnection, SimulationDefaults.Simphynity_enableConnection))
		{
        	String ip = settingsLoader.getSetting(Setting.Simphynity_ip, SimulationDefaults.Simphynity_ip);
			if(ip == null || ip.isEmpty())
				ip = "127.0.0.1";
			int port = settingsLoader.getSetting(Setting.Simphynity_port, SimulationDefaults.Simphynity_port);
			
	    	simphynityController = new SimphynityController(sim, this, ip, port);
		}
        
        // AutoPilot **************************************************************	
        FollowBoxSettings followBoxSettings = scenarioLoader.getAutoPilotFollowBoxSettings();
        isAutoPilot = scenarioLoader.isAutoPilot();
        if(followBoxSettings != null) followBox = new FollowBox(sim, this, followBoxSettings, isAutoPilot);
        // AutoPilot **************************************************************	
        
        // added by Yelly:
        //this.visible_angle = this.sim.getNumberOfScreens() * this.sim.getCameraFactory().getAngleBetweenAdjacentCameras();
        
        // at initialization, set near point and far point both to near point place
		if(this.sim.getCriticalPoints().size()>0) {
			updateLookingDir();
			updatePrevPastIndex();
			updateDistanceToStart();
			updateLateralPos();
			this.farPointPos = getFarPointPos();
			this.nearPointPos = getNearPointPos();
	        
	        // visualizing near point and far point
			if(this.visualizeNFPoints) {
		        if(this.visualizedFarPoint != null) sim.getSceneNode().detachChild(this.visualizedFarPoint);
		        this.visualizedFarPoint = sim.getAssetManager().loadModel(this.visualizedFarPointModelPath);
				sim.getSceneNode().attachChild(this.visualizedFarPoint);
				this.visualizedFarPoint.setName("FarPoint"+counter);
				this.visualizedFarPoint.setLocalTranslation(farPointPos);
				this.visualizedFarPoint.setLocalRotation(new Quaternion().fromAngles(this.visualizedFarPointModelAngles));
				this.visualizedFarPoint.setLocalScale(this.visualizedFarPointModelScale);
				
		        if(this.visualizedNearPoint != null) sim.getSceneNode().detachChild(this.visualizedNearPoint);
		        this.visualizedNearPoint = sim.getAssetManager().loadModel(this.visualizedNearPointModelPath);
				sim.getSceneNode().attachChild(this.visualizedNearPoint);
				this.visualizedNearPoint.setName("NearPoint"+counter);
				this.visualizedNearPoint.setLocalTranslation(nearPointPos);
				this.visualizedNearPoint.setLocalRotation(new Quaternion().fromAngles(this.visualizedNearPointModelAngles));
				this.visualizedNearPoint.setLocalScale(this.visualizedNearPointModelScale);
			}
			
			counter++;
		}

		
		// initialize critical element list
		// by marking every objects-car or sign-invisible at first

		ArrayList<TrafficObject> trafficObjectList = PhysicalTraffic.getTrafficObjectList();
		int ind = 0;
		for(TrafficObject vehicle : trafficObjectList){
			if(vehicle instanceof TrafficCar) {
				if(vehicle.getName().startsWith("vehicle"))	{
					// suppose all vehicles' names are in the format: "vehicle_color_..."
					String[] name_parts = vehicle.getName().split("_");
					if(name_parts.length < 2) {
						System.err.println("error vehicle name format");
						continue;
					}
					this.criticalElements.put(vehicle.getName(), new CriticalElement(vehicle.getName(), "vehicle", name_parts[1], false, false, false, false, false, false, ind));
				}
				ind++;
			}
		}

		List<MapObject> modelList = Simulator.getDrivingTask().getSceneLoader().getMapObjects();
		ind = 0;
		for(MapObject model : modelList){
			if(model.getName().startsWith("sign"))	{
				// suppose all signs' names are in the format: "sign_content_..."
				String[] name_parts = model.getName().split("_");
				if(name_parts.length < 2) {
					System.err.println("error signs name format");
					continue;
				}
				this.criticalElements.put(model.getName(), new CriticalElement(model.getName(), "sign", name_parts[1], false, false, false, false, false, false, ind));
			}
			ind++;
		}
		
		if(useQN) {
			this.qn = new QNCenter();
			SteeringCar.qnLogger = new QNLogger(Simulator.getOutputFolder());
		}
		else {
			this.qn = null;
		}
		
		this.initialClockStamp = System.currentTimeMillis();
	}


	/*public float getVisible_angle() {
		return visible_angle;
	}


	public void setVisible_angle(float visible_angle) {
		this.visible_angle = visible_angle;
	}*/


	public float getVisible_dist() {
		return visible_dist;
	}


	public void setVisible_dist(float visible_dist) {
		this.visible_dist = visible_dist;
	}


	public TrafficObjectLocator getObjectLocator()
	{
		return trafficObjectLocator;
	}
	
	
	public boolean isHandBrakeApplied()
	{
		return handBrakeApplied;
	}
	
	
	public void applyHandBrake(boolean applied)
	{
		handBrakeApplied = applied;
	}

	
	// start applying crosswind and return to 0 (computed in update loop)
	public void setupCrosswind(String direction, float force, int duration)
	{
		crosswind = new Crosswind(direction, force, duration);
	}
	
	
	Vector3f lastVelocity = new Vector3f(0,0,0);
	long m_nLastChangeTime = 0;
	
	public void setAutoPilot(Boolean isAutoPilot)
	{
		if(this.isAutoPilot == isAutoPilot)
			return;
		
		this.isAutoPilot = isAutoPilot;
		if(!isAutoPilot)
		{
			steer(0);
			brakePedalIntensity = 0;
			acceleratorPedalIntensity = 0;
			PanelCenter.getMessageBox().addMessage("Auto Pilot off", 3);
			Simulator.getDrivingTaskLogger().reportText("Auto Pilot off", new Date());
		}
		else
		{
			PanelCenter.getMessageBox().addMessage("Auto Pilot on", 3);
			Simulator.getDrivingTaskLogger().reportText("Auto Pilot on", new Date());
		}
	}
	
	public boolean isAutoPilot()
	{
		return isAutoPilot;
	}
	
	/**
	 * this method comes from CANClient
	 * Compares the current steering angle (in the simulator) with the given 
	 * steering angle (of the real car). The bigger the difference, the faster
	 * the steering angle of the simulator will be changed to the wanted value 
	 */
	private void updateSteeringAngle(float steeringAngle) 
	{
		final float maxSteeringAngle = 180; // this is a parameter subject to be changed, 
											// I took this value (180) from CAN setting of BMW
		try {
			
			// get target steering angle from real car
			// maximum angle will be matched to -1 or 1, respectively
			float targetAngle = -Math.max(Math.min(steeringAngle/maxSteeringAngle,1),-1);
			
			// print target (real car) steering angle
			//System.out.println("target: " + targetAngle);
			
			// if target angle is close to straight ahead, steer straight ahead
			if((targetAngle >= -0.001f) && (targetAngle <= 0.001f))	
				targetAngle = 0;
			
			steer(targetAngle);
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
	
	
	/**
	 * inside-mirror view, by default 50 degree
	 * @param element_pos 
	 * @return if visible in the back center mirror, return true; else return false
	 */
	private boolean elementVisibleInsideMirror(Vector3f element_pos) {
		final Camera back_cam = CameraFactory.getBackViewPort().getCamera();
		Vector3f back_cam_facing = back_cam.getDirection(); // this direction has been normalized
		Vector3f element_vec = element_pos.subtract(back_cam.getLocation());
		
		// within frustumFar and frustumNear range
		float projectCentral = back_cam_facing.dot(element_vec); // back_cam_facing is normalized
		//System.out.println("back_cam_facing: " + back_cam_facing + ", element_vec:"+ element_vec +", projectCentral: " + projectCentral + ", back_cam.getFrustumNear(): " + back_cam.getFrustumNear());
		// not use frustum_far because I assume driver could not see as far as mirror can...
		if(projectCentral > this.visible_dist || projectCentral < back_cam.getFrustumNear()) return false;
		
		// within left and right range
		Vector3f element_vec_projY_back = new Vector3f(element_vec.x, back_cam_facing.y, element_vec.z);
		final float back_cam_halfLevelAngle = FastMath.atan2(back_cam.getFrustumRight(), back_cam.getFrustumNear());
		//System.out.println("back_cam_halfLevelAngle*2=" + back_cam_halfLevelAngle * FastMath.RAD_TO_DEG * 2);
		if(FastMath.acos(back_cam_facing.dot(element_vec_projY_back.normalize())) > back_cam_halfLevelAngle) return false;
			
		// within top and bottom range
		if(FastMath.acos(element_vec.normalize().dot(element_vec_projY_back.normalize())) > FastMath.atan2(back_cam.getFrustumTop(), back_cam.getFrustumNear())) return false;
	
		//System.out.println("in back mirror");
		return true;
	}
	
	/**
	 * left-mirror view, by default 72 degree
	 * @param element_pos 
	 * @return
	 */
	private boolean elementVisibleLeftMirror(Vector3f element_pos) {
		final Camera leftBack_cam = CameraFactory.getLeftBackViewPort().getCamera();
		Vector3f leftBack_cam_facing = leftBack_cam.getDirection(); // this direction has been normalized
		Vector3f element_vec = element_pos.subtract(leftBack_cam.getLocation());
		
		// within frustumFar and frustumNear range
		float projectCentral = leftBack_cam_facing.dot(element_vec); // back_cam_facing is normalized
		// not use frustum_far because I assume driver could not see as far as mirror can...
		if(projectCentral > this.visible_dist || projectCentral < leftBack_cam.getFrustumNear()) return false;
		
		// within left and right range
		Vector3f element_vec_projY_leftBack = new Vector3f(element_vec.x, leftBack_cam_facing.y, element_vec.z);
		final float leftBack_cam_halfLevelAngle = FastMath.atan2(leftBack_cam.getFrustumRight(), leftBack_cam.getFrustumNear());
		//System.out.println("leftBack_cam_halfLevelAngle*2=" + leftBack_cam_halfLevelAngle * FastMath.RAD_TO_DEG * 2);
		if(FastMath.acos(leftBack_cam_facing.dot(element_vec_projY_leftBack.normalize())) > leftBack_cam_halfLevelAngle) return false;
			
		// within top and bottom range
		if(FastMath.acos(element_vec.normalize().dot(element_vec_projY_leftBack.normalize())) > FastMath.atan2(leftBack_cam.getFrustumTop(), leftBack_cam.getFrustumNear())) return false;
			
		//System.out.println("in left-back mirror");
		return true;
	}
	
	/**
	 * right-mirror view, by default 72 degree
	 * @param element_pos 
	 * @return
	 */
	private boolean elementVisibleRightMirror(Vector3f element_pos) {
		final Camera rightBack_cam = CameraFactory.getRightBackViewPort().getCamera();
		Vector3f rightBack_cam_facing = rightBack_cam.getDirection(); // this direction has been normalized
		Vector3f element_vec = element_pos.subtract(rightBack_cam.getLocation());
		
		// within frustumFar and frustumNear range
		float projectCentral = rightBack_cam_facing.dot(element_vec); // back_cam_facing is normalized
		// not use frustum_far because I assume driver could not see as far as mirror can...
		if(projectCentral > this.visible_dist || projectCentral < rightBack_cam.getFrustumNear()) return false;
		
		// within right and right range
		Vector3f element_vec_projY_rightBack = new Vector3f(element_vec.x, rightBack_cam_facing.y, element_vec.z);
		final float rightBack_cam_halfLevelAngle = FastMath.atan2(rightBack_cam.getFrustumRight(), rightBack_cam.getFrustumNear());
		//System.out.println("rightBack_cam_halfLevelAngle*2=" + rightBack_cam_halfLevelAngle * FastMath.RAD_TO_DEG * 2);
		if(FastMath.acos(rightBack_cam_facing.dot(element_vec_projY_rightBack.normalize())) > rightBack_cam_halfLevelAngle) return false;
			
		// within top and bottom range
		if(FastMath.acos(element_vec.normalize().dot(element_vec_projY_rightBack.normalize())) > FastMath.atan2(rightBack_cam.getFrustumTop(), rightBack_cam.getFrustumNear())) return false;
			
		//System.out.println("in right-back mirror");
		return true;
	}
	
	/**
	 * left blind zone view, seen when driver turns his head to left by 90 degrees
	 * 	field of view: visible_angle. By default, one screen, 40 degree
	 * @param element_pos 
	 * @return
	 */
	private boolean elementVisibleLeft(Vector3f element_pos) {
		// check if the object within visible angle
		float heading_in_radian = (float) Math.toRadians(this.getHeadingDegree());
		Vector3f headDir = leftPerpendicular(new Vector3f(FastMath.sin(heading_in_radian), 0, -FastMath.cos(heading_in_radian)));
		Vector3f curToElement = element_pos.subtract(this.getPosition()).normalize();

		float absAngleRadian = FastMath.acos(headDir.dot(curToElement));
		float absAngleDegree = (float) Math.toDegrees(absAngleRadian);
		
		if(absAngleDegree > visible_angle/2) return false;
		
		// check if the object within visible distance
		if(this.getPosition().distance(element_pos) > this.visible_dist) return false;

		//System.out.println("in left view");

		return true;
	}
	
	/**
	 * right blind zone view, seen when driver turns his head to right by 90 degrees
	 * 	field of view: visible_angle. By default, one screen, 40 degree
	 * @param element_pos 
	 * @return
	 */
	private boolean elementVisibleRight(Vector3f element_pos) {
		// check if the object within visible angle
		float heading_in_radian = (float) Math.toRadians(this.getHeadingDegree());
		Vector3f headDir = leftPerpendicular(new Vector3f(FastMath.sin(heading_in_radian), 0, -FastMath.cos(heading_in_radian))).mult(-1f);
		Vector3f curToElement = element_pos.subtract(this.getPosition()).normalize();

		float absAngleRadian = FastMath.acos(headDir.dot(curToElement));
		float absAngleDegree = (float) Math.toDegrees(absAngleRadian);
		
		if(absAngleDegree > visible_angle/2) return false;
		
		// check if the object within visible distance
		if(this.getPosition().distance(element_pos) > this.visible_dist) return false;

		//System.out.println("in left view");

		return true;
	}
		
	
	/**
	 * front view, 
	 * 	field of view: visible_angle. By default, one screen, 40 degree
	 * @param element_pos 
	 * @return if visible in the front view, return true; else return false
	 */
	private boolean elementVisibleFront(Vector3f element_pos) {
//		
//		System.out.println("visible_angle:" + visible_angle);
//		System.out.println("this.sim.getNumberOfScreens():" + this.sim.getNumberOfScreens());
//		System.out.println("this.sim.getCameraFactory().getAngleBetweenAdjacentCameras():" + this.sim.getCameraFactory().getAngleBetweenAdjacentCameras());
		
		// check if the object within visible angle
		float heading_in_radian = (float) Math.toRadians(this.getHeadingDegree());
		Vector3f headDir = new Vector3f(FastMath.sin(heading_in_radian), 0, -FastMath.cos(heading_in_radian));
		Vector3f curToElement = element_pos.subtract(this.getPosition()).normalize();
		//float absAngle =  headDir.angleBetween(curToNearPt);
		float absAngleRadian = FastMath.acos(headDir.dot(curToElement));
		float absAngleDegree = (float) Math.toDegrees(absAngleRadian);
		
		if(absAngleDegree > visible_angle/2) return false;
		
		// check if the object within visible distance
		if(this.getPosition().distance(element_pos) > this.visible_dist) return false;

		//System.out.println("in front view");

		return true;
	}
	
	private HashMap<String, CriticalElement> updateCriticalElementVisibility() {

		if(!visible_angle_initialized) {
			visible_angle = this.sim.getNumberOfScreens() * this.sim.getCameraFactory().getAngleBetweenAdjacentCameras();
			visible_angle_initialized = true;
		}
		
		HashMap<String, CriticalElement> changed_critical_element = new HashMap<String, CriticalElement>();
		
		Iterator<Entry<String, CriticalElement>> it = this.criticalElements.entrySet().iterator();
		int type; //0-sign, 1-vehicle
		while(it.hasNext()) {
			Map.Entry<String, CriticalElement> pair = (Map.Entry<String, CriticalElement>)it.next();
			
			Vector3f vehicle_pos;
			if(pair.getValue().getType().equals("vehicle"))	{
				TrafficObject vehicle = PhysicalTraffic.getTrafficObjectList().get(pair.getValue().getIndex());
				//System.out.println("vehicle: " + vehicle.getName() + ":::::");
				vehicle_pos = vehicle.getPosition();
				type = 1;
			}
			else {
				MapObject sign = Simulator.getDrivingTask().getSceneLoader().getMapObjects().get(pair.getValue().getIndex());
				vehicle_pos = sign.getLocation();
				type = 0;
			}
				
			boolean changed = false;
			boolean visiblility, visibility_all_view = false;
			//System.out.println("critical element: " + pair.getKey() +", at position: " + vehicle_pos);
			
			// front view
			visiblility = elementVisibleFront(vehicle_pos);
			if(visiblility) {
				if(this.useQN && this.initStat) changed = true;
				else if(!pair.getValue().isFront_visibility()) changed = true;
				visibility_all_view = true;
				//System.out.print("front-view visible, ");
			}
			else if(pair.getValue().isFront_visibility()) changed = true;
			pair.getValue().setFront_visibility(visiblility);
				
			// inside-mirror view
			if(type ==0) visiblility = false; // driver cannot see signs in mirrors because signs are one-sided
			else visiblility = elementVisibleInsideMirror(vehicle_pos);
			if(visiblility) {
				if(this.useQN && this.initStat) changed = true;
				else if(!pair.getValue().isInsideMirror_visibility()) changed = true;
				visibility_all_view = true;
				//System.out.print("inside-mirror-view visible, ");
			}
			else if(pair.getValue().isInsideMirror_visibility()) changed = true;
			pair.getValue().setInsideMirror_visibility(visiblility);
				
			// left-mirror view
			if(type ==0) visiblility = false; // driver cannot see signs in mirrors because signs are one-sided
			else visiblility = elementVisibleLeftMirror(vehicle_pos);
			if(visiblility) {
				if(this.useQN && this.initStat) changed = true;
				else if(!pair.getValue().isLeftMirror_visibility()) changed = true;
				visibility_all_view = true;
				//System.out.print("left-mirror-view visible, ");
			}
			else if(pair.getValue().isLeftMirror_visibility()) changed = true;
			pair.getValue().setLeftMirror_visibility(visiblility);
				
			// right-mirror view
			if(type ==0) visiblility = false; // driver cannot see signs in mirrors because signs are one-sided
			else visiblility = elementVisibleRightMirror(vehicle_pos);
			if(visiblility) {
				if(this.useQN && this.initStat) changed = true;
				else if(!pair.getValue().isRightMirror_visibility()) changed = true;
				visibility_all_view = true;
				//System.out.print("right-mirror-view visible, ");
			}
			else if(pair.getValue().isRightMirror_visibility()) changed = true;
			pair.getValue().setRightMirror_visibility(visiblility);
			
			// left view
			visiblility = elementVisibleRight(vehicle_pos);
			if(visiblility) {
				if(this.useQN && this.initStat) changed = true;
				else if(!pair.getValue().isRight_visibility()) changed = true;
				visibility_all_view = true;
				//System.out.print("left-view visible, ");
			}
			else if(pair.getValue().isRight_visibility()) changed = true;
			pair.getValue().setRight_visibility(visiblility);
		
			// right view
			visiblility = elementVisibleLeft(vehicle_pos);
			if(visiblility) {
				if(this.useQN && this.initStat) changed = true;
				else if(!pair.getValue().isLeft_visibility()) changed = true;
				//System.out.print("right-view visible, ");
			}
			else if(pair.getValue().isLeft_visibility()) changed = true;
			pair.getValue().setLeft_visibility(visiblility);
				
			if(changed) {
				changed_critical_element.put(pair.getKey(), pair.getValue());
			}
			
			if(visibility_all_view) {
				this.visible_ce.put(pair.getKey(), vehicle_pos); // even if the visibility doesn't change, the position might have changed
				//System.out.println("\nbeing put into visible_ce map: pair.getKey()=" + pair.getKey() + ", vehicle_pos=" + vehicle_pos);
			}
		}
		
		/*System.out.println("updated critical elements:");
		System.out.println(this.criticalElements);
		System.out.println("changed_critical_element to return:");
		System.out.println(changed_critical_element);*/
		
		return changed_critical_element;
	}
	
	// will be called, in every frame
	@Override
	public void update(float tpf, ArrayList<TrafficObject> vehicleList)	{
		
//		System.out.println(followBox.getTrafficObject().getName() );  
		
		//System.out.println("current pos: " + this.getPosition());
		// AutoPilot **************************************************************
		if(!sim.isPause() && isAutoPilot)
		{
			// update steering
			Vector3f wayPoint = followBox.getPosition();
			steerTowardsPosition(wayPoint);
			
			// update speed
			updateSpeed(tpf, vehicleList);
		}		
		
		// update movement of follow box according to vehicle's position
		if(followBox != null) {				// Cao, 2018-03-12, changed from: if(followBox != null) {
			
			Vector3f vehicleCenterPos = centerGeometry.getWorldTranslation();
			
			
//			System.out.println("self location: " + vehicleCenterPos.x + ", " + vehicleCenterPos.y + ", " + vehicleCenterPos.z);
			
//			System.out.println("followBox location before : " + followBox.getPosition().x + ", " + followBox.getPosition().y + ", " + followBox.getPosition().z);

			
			followBox.update(vehicleCenterPos);
			
//			System.out.println("followBox location after : " + followBox.getPosition().x + ", " + followBox.getPosition().y + ", " + followBox.getPosition().z);
//			System.out.println();
			
		}
		// AutoPilot **************************************************************	

		//for QN connection
		float currentTimeSecond = (System.currentTimeMillis() - this.initialClockStamp) / 1000;	
		
		if(!isAutoPilot){
			//for QN driving model
			//acceleratorPedalIntensity -1 for ahead, 1 for backward	
//			System.out.println("acceleratorPedalIntensity: " + acceleratorPedalIntensity);	
			if(useQN) {
				if(currentTimeSecond  <   initalAccDuration) {
					//acceleration
					acceleratorPedalIntensity = - 1f;   // give the car a little initial speed for far point calculation to work.
					//this.initStat = true;
				}
				else {
					QNControlRecv qncontrol = this.qn.receive();
					if(qncontrol.isReport()) {
						SteeringCar.qnLogger.reportText("\n\t" + qncontrol.getQNClock() + "\tposition:{" + this.getPosition().x+", " + this.getPosition().y+", " + this.getPosition().z+"}\tspeed(m/s): " + this.getCurrentSpeedMs());
						SteeringCar.qnLogger.reportText("visible critical elements: ");
						
						Iterator<Entry<String, Vector3f>> it = this.visible_ce.entrySet().iterator();
						while(it.hasNext()) {
							Map.Entry<String, Vector3f> pair = (Map.Entry<String, Vector3f>)it.next();
							SteeringCar.qnLogger.reportText(pair.getKey() + ": {" + pair.getValue().x+", " + pair.getValue().y+", " + pair.getValue().z+"}");
						}
						System.out.println("report triggered");
					}
					
					//acceleration	
					/*
					 * mind the intensity value for setAcceleratorIntensity:
					 * @param intensity
					 *            -1 for full ahead and 1 for full backwards
					 */
					this.acceleratorPedalIntensity = -qncontrol.getAccelerator();
					//System.out.println("in SteeringCar update, this.acceleratorPedalIntensity=" + this.acceleratorPedalIntensity);
					
					//brake
					this.brakePedalIntensity = qncontrol.getBrake();
					//System.out.println("in SteeringCar update, this.brakePedalIntensity =" +this.brakePedalIntensity);
					
					//steering
					// mind in the comments, I find for QN-ACTR, 1 for left, -1 for right,
					// while for OpenDS steer() method, 1 for right, -1 for left.
					// so multiple by -1???
					updateSteeringAngle(-qncontrol.getSteering());
					//steer(qncontrol.getSteering() * FastMath.DEG_TO_RAD);
					
					//System.out.println("in SteeringCar update, steering by " + (qncontrol.getSteering() * FastMath.DEG_TO_RAD) + "(" +  qncontrol.getSteering() +" in degree )");
					
					//send  TODO move to the end
					
//					this.qn.send(new QNModel(currentTimeSecond, this.getNearFarPointAngle(this.nearPointPos), this.getNearFarPointAngle(this.farPointPos), this.getPosition().distance(this.farPointPos), this.getCurrentSpeedMs(), this.getName(), this.getPosition()).toSendToQN());
					
					
				}
			}
			
			// accelerate
			float pAccel = 0;
			if(!engineOn)
			{
				// apply 0 acceleration when engine not running
				pAccel = powerTrain.getPAccel(tpf, 0) * 30f;
			}
			else if(isAutoAcceleration && (getCurrentSpeedKmh() < minSpeed))
			{
				// apply maximum acceleration (= -1 for forward) to maintain minimum speed
				pAccel = powerTrain.getPAccel(tpf, -1) * 30f;
			}
			else if(isCruiseControl && (getCurrentSpeedKmh() < targetSpeedCruiseControl))
			{
				// apply maximum acceleration (= -1 for forward) to maintain target speed
				pAccel = powerTrain.getPAccel(tpf, -1) * 30f;
				
				if(isAdaptiveCruiseControl)
				{
					// lower speed if leading car is getting to close
					pAccel = getAdaptivePAccel(pAccel);
				}
			}
			else
			{		
				// apply acceleration according to gas pedal state
				pAccel = powerTrain.getPAccel(tpf, acceleratorPedalIntensity) * 30f;			
			}		
			
			// acceleration
			transmission.performAcceleration(pAccel);
		
			// brake lights
			setBrakeLight(brakePedalIntensity > 0);
			
			if(handBrakeApplied)
			{
				// hand brake
				carControl.brake(maxBrakeForce);
				PanelCenter.setHandBrakeIndicator(true);
			}
			else
			{
				// brake	
				float appliedBrakeForce = brakePedalIntensity * maxBrakeForce;
				float currentFriction = powerTrain.getFrictionCoefficient() * maxFreeWheelBrakeForce;
				carControl.brake(appliedBrakeForce + currentFriction);
				PanelCenter.setHandBrakeIndicator(false);
			}
		}
		
		// lights
		leftHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
        leftHeadLight.setPosition(carModel.getLeftLightPosition());
        leftHeadLight.setDirection(carModel.getLeftLightDirection());
        
        rightHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
        rightHeadLight.setPosition(carModel.getRightLightPosition());
        rightHeadLight.setDirection(carModel.getRightLightDirection());
        
        // cruise control indicator
        if(isCruiseControl)
        	PanelCenter.setCruiseControlIndicator(targetSpeedCruiseControl);
        else
        	PanelCenter.unsetCruiseControlIndicator();
        
        trafficObjectLocator.update();
        
        // switch off turn signal after turn        
        if(hasFinishedTurn())
        {
        	lightTexturesContainer.setTurnSignal(TurnSignalState.OFF);
        }
        
        lightTexturesContainer.update();
        
		steeringInfluenceByCrosswind = crosswind.getCurrentSteeringInfluence();

        updateFrictionSlip();
        
        updateWheel();
        
        if(simphynityController != null)
        	simphynityController.update();
		    //simphynityController.updateNervtehInstructions();
        
        Boolean shadowModelActive = SimulationBasics.getSettingsLoader().getSetting(Setting.HighPolygon_carModel, SimulationDefaults.HighPolygon_carModel);
        
        try {
	        if (shadowModelActive){
		        Node steeringWheel = Util.findNode(carNode, "SteeringWheel");
		        
		        float currentPosition = getSteeringWheelStateNoNoise(sim.getCar().getSteeringWheelState());
		        //System.out.println("Position of steering wheel " + -currentPosition); //debug 
		
		        if (currentPosition == 0.0f){
		        	steeringWheel.setLocalRotation(initialPositionSteering);
		        }
		        else
		        {
		        	rotationSteering.fromAngleAxis(FastMath.DEG_TO_RAD*15, new Vector3f(0.0f, currentPosition, 0.0f ));
		        	steeringWheel.setLocalRotation(rotationSteering);
		        }
	        }
        } catch (Exception e){
        	e.printStackTrace();
        }

		
		// added by Yelly:
        // update dts
		if(this.sim.getCriticalPoints().size()>0) {
			//System.out.println("preIndex: " + this.prevPastIndex);
			updateLookingDir();
			updatePrevPastIndex();
			updateDistanceToStart();
			updateLateralPos();
			this.farPointPos = getFarPointPos();
			this.nearPointPos = getNearPointPos();
	        
	        // visualizing near point and far point
			if(this.visualizeNFPoints) {
		        if(this.visualizedFarPoint != null) sim.getSceneNode().detachChild(this.visualizedFarPoint);
		        this.visualizedFarPoint = sim.getAssetManager().loadModel(this.visualizedFarPointModelPath);
				sim.getSceneNode().attachChild(this.visualizedFarPoint);
				this.visualizedFarPoint.setName("FarPoint"+counter);
				this.visualizedFarPoint.setLocalTranslation(farPointPos);
				this.visualizedFarPoint.setLocalRotation(new Quaternion().fromAngles(this.visualizedFarPointModelAngles));
				this.visualizedFarPoint.setLocalScale(this.visualizedFarPointModelScale);
				
		        if(this.visualizedNearPoint != null) sim.getSceneNode().detachChild(this.visualizedNearPoint);
		        this.visualizedNearPoint = sim.getAssetManager().loadModel(this.visualizedNearPointModelPath);
				sim.getSceneNode().attachChild(this.visualizedNearPoint);
				this.visualizedNearPoint.setName("NearPoint"+counter);
				this.visualizedNearPoint.setLocalTranslation(nearPointPos);
				this.visualizedNearPoint.setLocalRotation(new Quaternion().fromAngles(this.visualizedNearPointModelAngles));
				this.visualizedNearPoint.setLocalScale(this.visualizedNearPointModelScale);
			}
			
			counter++;
		}

		//for QN connection
		if(useQN) {
			if(currentTimeSecond  <   initalAccDuration) {
				//nothing
			}
			else {
				// update critical element visibility
				HashMap<String, CriticalElement> changeCEList = updateCriticalElementVisibility();
				//System.out.println(this.criticalElements);
				//System.out.println(changeCEList);
				
				this.initStat = false; // notice that the state changes after method updateCriticalElementVisibility called, so For the first sending, all visible critical element at this point will be sent to QN-ACTR
				//if(changeCEList.size()!=0)	System.out.println("criticalElements=" + changeCEList);
				this.qn.send(new QNModel(currentTimeSecond, this.getNearFarPointAngle(this.nearPointPos), this.getNearFarPointAngle(this.farPointPos), this.getPosition().distance(this.farPointPos), this.getCurrentSpeedMs(),changeCEList).toSendToQN());
			}
		}
	}
	
	private float getSteeringWheelStateNoNoise(float currentValue){
		if  ( currentValue > -0.003f && currentValue < 0.003f ){	
			return 0.0f;
		}
		else {
			return currentValue;
		}
		
	}
	
	
	
    float leftWheelsPos = 2.2f;
    float backAxleHeight = -3.0f;
    float backAxlePos = 2.45f;
    long prevTime = 0;
    
    
	private void updateWheel() 
	{     
		long time = System.currentTimeMillis();
		if(time - prevTime > 1000)
		{/*
			Vector3f wheelDirection = new Vector3f(0, -1, 0);
			Vector3f wheelAxle = new Vector3f(-1, 0, 0);
			float wheelRadius = 0.5f;
			float suspensionLenght = 0.2f;
		
			carControl.removeWheel(3);
		
			backAxlePos += 0.05f;
		
			// add back left wheel
			Geometry geom_wheel_fl = Util.findGeom(carNode, "WheelBackLeft");
			geom_wheel_fl.setLocalScale(wheelRadius*2);
			geom_wheel_fl.center();
			BoundingBox box = (BoundingBox) geom_wheel_fl.getModelBound();
			carControl.addWheel(geom_wheel_fl.getParent(), 
        		box.getCenter().add(leftWheelsPos, backAxleHeight, backAxlePos),
                wheelDirection, wheelAxle, suspensionLenght, wheelRadius, true);

			System.out.println("backAxlePos: " + backAxlePos);
			
			prevTime = time;
			*/
		}
		//System.out.println("prevTime: " + prevTime + "  time: " + time);
	}


	private void updateFrictionSlip() 
	{
		/*
        // ice
        carControl.setRollInfluence(0, 0.5f); 
        carControl.setRollInfluence(1, 0.5f); 
        carControl.setRollInfluence(2, 0.5f); 
        carControl.setRollInfluence(3, 0.5f);
        
        carControl.setFrictionSlip(0, 1f); 
        carControl.setFrictionSlip(1, 1f); 
        carControl.setFrictionSlip(2, 1f); 
        carControl.setFrictionSlip(3, 1f); 
        */
	}


	private boolean hasStartedTurning = false;
	private boolean hasFinishedTurn() 
	{
		TurnSignalState turnSignalState = lightTexturesContainer.getTurnSignal();
		float steeringWheelState = getSteeringWheelState();
		
		if(turnSignalState == TurnSignalState.LEFT)
		{
			if(steeringWheelState > turnSignalThreshold)
				hasStartedTurning = true;
			else if(hasStartedTurning)
			{
				hasStartedTurning = false;
				return true;
			}
		}
		
		if(turnSignalState == TurnSignalState.RIGHT)
		{
			if(steeringWheelState < -turnSignalThreshold)
				hasStartedTurning = true;
			else if(hasStartedTurning)
			{
				hasStartedTurning = false;
				return true;
			}
		}
		
		return false;
	}


	// Adaptive Cruise Control ***************************************************	
	
	private float getAdaptivePAccel(float pAccel)
	{
		brakePedalIntensity = 0f;

		// check distance from traffic vehicles
		for(TrafficObject vehicle : PhysicalTraffic.getTrafficObjectList())
		{
			if(belowSafetyDistance(vehicle.getPosition()))
			{
				pAccel = 0;
			
				if(vehicle.getPosition().distance(getPosition()) < emergencyBrakeDistance)
					brakePedalIntensity = 1f;
			}
		}
		
		return pAccel;
	}

	
	private boolean belowSafetyDistance(Vector3f obstaclePos) 
	{	
		float distance = obstaclePos.distance(getPosition());
		
		// angle between driving direction of traffic car and direction towards obstacle
		// (consider 3D space, because obstacle could be located on a bridge above traffic car)
		Vector3f carFrontPos = frontGeometry.getWorldTranslation();
		Vector3f carCenterPos = centerGeometry.getWorldTranslation();
		float angle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, obstaclePos, false);
		
		float lateralDistance = distance * FastMath.sin(angle);
		float forwardDistance = distance * FastMath.cos(angle);
		
		if((lateralDistance < minLateralSafetyDistance) && (forwardDistance > 0) && 
				(forwardDistance < Math.max(0.5f * getCurrentSpeedKmh(), minForwardSafetyDistance)))
		{
			return true;
		}
		
		return false;
	}


	public void increaseCruiseControl(float diff) 
	{
		targetSpeedCruiseControl = Math.min(targetSpeedCruiseControl + diff, 260.0f);	
	}


	public void decreaseCruiseControl(float diff) 
	{
		targetSpeedCruiseControl = Math.max(targetSpeedCruiseControl - diff, 0.0f);
	}

	
	public void disableCruiseControlByBrake() 
	{
		if(!suppressDeactivationByBrake)
			setCruiseControl(false);
	}
	// Adaptive Cruise Control ***************************************************


	
	public float getDistanceToRoadSurface() 
	{
		// reset collision results list
		CollisionResults results = new CollisionResults();

		// aim a ray from the car's center downwards to the road surface
		Ray ray = new Ray(getPosition(), Vector3f.UNIT_Y.mult(-1));

		// collect intersections between ray and scene elements in results list.
		sim.getSceneNode().collideWith(ray, results);
		
		// return the result
		for (int i = 0; i < results.size(); i++) 
		{
			// for each hit, we know distance, contact point, name of geometry.
			float dist = results.getCollision(i).getDistance();
			Geometry geometry = results.getCollision(i).getGeometry();

			if(geometry.getName().contains("CityEngineTerrainMate"))
				return dist - 0.07f;
		}
		
		return -1;
	}		
	
 	// AutoPilot *****************************************************************
	
	
	// AutoPilot *****************************************************************
	
	private void updateSpeed(float tpf, ArrayList<TrafficObject> vehicleList) 
	{
		float targetSpeed = getTargetSpeed();
		
		//if(overwriteSpeed >= 0)
		//	targetSpeed = Math.min(targetSpeed, overwriteSpeed);
		
		// stop car in order to avoid collision with other traffic objects and driving car
		// also for red traffic lights
		if(obstaclesInTheWay(vehicleList))
			targetSpeed = 0;
		
		float currentSpeed = getCurrentSpeedKmh();
		
		//System.out.print(name + ": " + targetSpeed + " *** " + currentSpeed);
		
		
		// set pedal positions
		if(currentSpeed < targetSpeed)
		{
			// too slow --> accelerate
			setAcceleratorPedalIntensity(-1);
			setBrakePedalIntensity(0);
			//System.out.println("gas");
			//System.out.print(" *** gas");
		}
		else if(currentSpeed > targetSpeed+1)
		{
			// too fast --> brake
			
			// currentSpeed >= targetSpeed+3 --> brake intensity: 100%
			// currentSpeed == targetSpeed+2 --> brake intensity:  50%
			// currentSpeed <= targetSpeed+1 --> brake intensity:   0%
			float brakeIntensity = (currentSpeed - targetSpeed - 1)/2.0f;
			brakeIntensity = Math.max(Math.min(brakeIntensity, 1.0f), 0.0f);
			
			// formerly use
			//brakeIntensity = 1.0f;
			
			setBrakePedalIntensity(brakeIntensity);
			setAcceleratorPedalIntensity(0);
			
			//System.out.println("brake: " + brakeIntensity);
			//System.out.print(" *** brake");
		}
		else
		{
			// else release pedals
			setAcceleratorPedalIntensity(0);
			setBrakePedalIntensity(0);
			//System.out.print(" *** free");
		}
		
		
		
		// accelerate
		if(engineOn)
			//carControl.accelerate(acceleratorPedalIntensity * accelerationForce);
			transmission.performAcceleration(powerTrain.getPAccel(tpf, acceleratorPedalIntensity) * 30f);
		else
			//carControl.accelerate(0);
			transmission.performAcceleration(0);
		//System.out.print(" *** " + gasPedalPressIntensity * accelerationForce);
		
		// brake	
		float appliedBrakeForce = brakePedalIntensity * maxBrakeForce;
		float currentFriction = 0.2f * maxFreeWheelBrakeForce;
		carControl.brake(appliedBrakeForce + currentFriction);
		
		//System.out.print(" *** " + appliedBrakeForce + currentFriction);
		//System.out.println("");
	}


	public float getTargetSpeed() 
	{
		// maximum speed for current way point segment
		float regularSpeed = followBox.getSpeed();

		// reduced speed to reach next speed limit in time
		float reducedSpeed = followBox.getReducedSpeed();
		
		float targetSpeed = Math.max(Math.min(regularSpeed, reducedSpeed),0);
		
		// limit maximum speed to speed of steering car 
		//if(isSpeedLimitedToSteeringCar)
		//	targetSpeed = Math.min(sim.getCar().getCurrentSpeedKmh(), targetSpeed);
		
		return targetSpeed;
	}
	
	
	/**
	 * Returns the signum of the speed change between this and the previous way point: 
	 * 0 if speed has not changed (or no previous way point available), 
	 * 1 if speed has been increased,
	 * -1 if speed has been decreased.
	 * 
	 * @return
	 * 		The signum of the speed change between this and the previous way point
	 */
	public int getSpeedChange()
	{
		Waypoint previousWP = followBox.getPreviousWayPoint();
		Waypoint currentWP = followBox.getCurrentWayPoint();
		
		if(previousWP == null)
			return 0;
		else
			return (int) Math.signum(currentWP.getSpeed() - previousWP.getSpeed());
	}


	private boolean obstaclesInTheWay(ArrayList<TrafficObject> vehicleList)
	{
		// check distance from driving car
		if(obstacleTooClose(sim.getCar().getPosition()))
			return true;

		// check distance from other traffic (except oneself)
		for(TrafficObject vehicle : vehicleList)
		{
			if(obstacleTooClose(vehicle.getPosition()))
				return true;
		}
		
		// check if red traffic light ahead
		Waypoint nextWayPoint = followBox.getNextWayPoint();
		if(TrafficLightCenter.hasRedTrafficLight(nextWayPoint))
			if(obstacleTooClose(nextWayPoint.getPosition()))
				return true;
		
		return false;
	}


	private boolean obstacleTooClose(Vector3f obstaclePos)
	{
		float distanceToObstacle = obstaclePos.distance(getPosition());
		
		// angle between driving direction of traffic car and direction towards obstacle
		// (consider 3D space, because obstacle could be located on a bridge above traffic car)
		Vector3f carFrontPos = frontGeometry.getWorldTranslation();
		Vector3f carCenterPos = centerGeometry.getWorldTranslation();
		float angle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, obstaclePos, false);
		if(belowSafetyDistance(angle, distanceToObstacle))
			return true;

		// considering direction towards next way point (if available)
		Waypoint nextWP = followBox.getNextWayPoint();
		if(nextWP != null)
		{
			// angle between direction towards next WP and direction towards obstacle
			// (consider 3D space, because obstacle could be located on a bridge above traffic car)
			angle = Util.getAngleBetweenPoints(nextWP.getPosition(), carCenterPos, obstaclePos, false);
			if(belowSafetyDistance(angle, distanceToObstacle))
				return true;
		}
		return false;
	}
	
	
	private boolean belowSafetyDistance(float angle, float distance) 
	{	
		float lateralDistance = distance * FastMath.sin(angle);
		float forwardDistance = distance * FastMath.cos(angle);
		
		//if(name.equals("car1"))
		//	System.out.println(lateralDistance + " *** " + forwardDistance);
		
		float speedDependentForwardSafetyDistance = 0;
		
		//if(useSpeedDependentForwardSafetyDistance)
		//	speedDependentForwardSafetyDistance = 0.5f * getCurrentSpeedKmh();
		
		if((lateralDistance < minLateralSafetyDistance) && (forwardDistance > 0) && 
				(forwardDistance < Math.max(speedDependentForwardSafetyDistance , minForwardSafetyDistance)))
		{
			return true;
		}
		
		return false;
	}

	@Override
	public String getName() 
	{
		return "drivingCar";
	}


	@Override
	public void setToWayPoint(String wayPointID) 
	{
		int index = followBox.getIndexOfWP(wayPointID);
		if(index != -1)
			followBox.setToWayPoint(index);
		else
			System.err.println("Invalid way point ID: " + wayPointID);
	}

	
	// AutoPilot *****************************************************************


}
