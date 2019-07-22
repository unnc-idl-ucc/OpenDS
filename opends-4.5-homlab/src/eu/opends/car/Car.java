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

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.VehicleControl;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.light.SpotLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.shape.Box;

import eu.opends.audio.AudioCenter;
import eu.opends.car.LightTexturesContainer.TurnSignalState;
import eu.opends.customization.CriticalWayPoint;
import eu.opends.environment.GeoPosition;
import eu.opends.main.Simulator;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.SpeedControlCenter;
import eu.opends.tools.Util;
import eu.opends.tools.Vector3d;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficCar;
import eu.opends.traffic.TrafficObject;

/**
 * 
 * @author Rafael Math
 */
public abstract class Car
{
	protected Simulator sim;
	protected Vector3f initialPosition;
	protected Quaternion initialRotation;
	protected Geometry frontGeometry;
	protected Geometry centerGeometry;
	
	protected CarModelLoader carModel;
    protected VehicleControl carControl;
    protected Node carNode;
    protected VehicleControl trailerControl;
    protected Node trailerNode;
    protected LightTexturesContainer lightTexturesContainer;
    
    private float steeringWheelState;
    protected float steeringInfluenceByCrosswind = 0;
    protected float acceleratorPedalIntensity;
    protected float brakePedalIntensity;
    protected float clutchPedalIntensity;
    protected float traction = 0.0f;
    protected int resetPositionCounter;
    protected Vector3f previousPosition;
    private float distanceOfCurrentFrame = 0;
    protected float mileage;
    protected boolean engineOn;

    protected float mass;
    protected boolean isAutoAcceleration = true;
    protected float targetSpeedCruiseControl = 0;
    protected boolean isCruiseControl = false;
    protected float minSpeed;
    protected float maxSpeed;
    protected float acceleration;
    protected float accelerationForce;
    protected float decelerationBrake;
    protected float maxBrakeForce;
    protected float decelerationFreeWheel;
    protected float maxFreeWheelBrakeForce;
    
    protected Transmission transmission;
    protected PowerTrain powerTrain;
    
    protected SpotLight leftHeadLight;
    protected SpotLight rightHeadLight;
    protected float lightIntensity = 0;
    protected String modelPath = "Test";

    // added by Yelly
    protected int prevPastIndex = 0; // this value should be set within the range [-1, lastLegalIndex],
									// for our test the car position shouldn't be too far from the first or the last critical way point
									// or it would be the problem of lacking critical way points inputed
    protected float dts = 0f; // distance to start
    protected float lateralPos = 0f;
    protected int lookDir = 1; // 1 represents looking from negative y to positive y, left is left
								// -1 represents looking from positive y to negative y, left is right
	
    static public final float Near_Point_Distance_Ahead = 20f; // in meters
    static public final float Far_Point_Time_Ahead = 3f; // in seconds
    static public final float Road_Segment_Length = 20f; // in meters
    static public final float Max_Road_Radius = 1800f; // in meters. I adjusted this parameter through test-and-trail
    static protected final String newLine = System.getProperty("line.separator");
    static public final boolean Far_Point_Follow_Vehicle = false;
    
    public Simulator getSim() {
		return sim;
	}


	public Geometry getFrontGeometry() {
		return frontGeometry;
	}


	public Geometry getCenterGeometry() {
		return centerGeometry;
	}


	public VehicleControl getTrailerControl() {
		return trailerControl;
	}


	public Node getTrailerNode() {
		return trailerNode;
	}


	public LightTexturesContainer getLightTexturesContainer() {
		return lightTexturesContainer;
	}


	public int getResetPositionCounter() {
		return resetPositionCounter;
	}


	public Vector3f getPreviousPosition() {
		return previousPosition;
	}


	public float getDistanceOfCurrentFrame() {
		return distanceOfCurrentFrame;
	}


	public float getTargetSpeedCruiseControl() {
		return targetSpeedCruiseControl;
	}


	public float getAccelerationForce() {
		return accelerationForce;
	}


	public float getDecelerationBrake() {
		return decelerationBrake;
	}


	public float getMaxFreeWheelBrakeForce() {
		return maxFreeWheelBrakeForce;
	}


	public SpotLight getLeftHeadLight() {
		return leftHeadLight;
	}


	public SpotLight getRightHeadLight() {
		return rightHeadLight;
	}


	public float getLightIntensity() {
		return lightIntensity;
	}


	public String getModelPath() {
		return modelPath;
	}


	public int getPrevPastIndex() {
		return prevPastIndex;
	}


	public float getDts() {
		return dts;
	}


	public float getLateralPos() {
		return lateralPos;
	}


	public float getPreviousClutchPedalIntensity() {
		return previousClutchPedalIntensity;
	}


	protected void init()
    {
		previousPosition = initialPosition;
		resetPositionCounter = 0;
		mileage = 0;
		
        // load car model
		carModel = new CarModelLoader(sim, this, modelPath, mass);
		carControl = carModel.getCarControl();
		carNode = carModel.getCarNode();
		carNode.setShadowMode(ShadowMode.Cast);
		
		// generate path to light textures from model path
		File modelFile = new File(modelPath);
		String lightTexturesPath = modelFile.getPath().replace(modelFile.getName(), "lightTextures.xml");
		
		// load light textures
		lightTexturesContainer = new LightTexturesContainer(sim, this, lightTexturesPath);
		//lightTexturesContainer.printAllContent();
		
        // add car node to rendering node
        sim.getSceneNode().attachChild(carNode);
        
        // add car to physics node
        sim.getPhysicsSpace().add(carControl);

		// setup head light
        setupHeadlight(sim);
        
        // add trailer
        boolean hasTrailer = false;
        if(hasTrailer)
        	setupTrailer();

        // set initial position and orientation
        setPosition(initialPosition);
        setRotation(initialRotation);

        // apply continuous braking (simulates friction when free wheeling)
        resetPedals();
        
        setupReferencePoints();
    }

	
	private void setupHeadlight(Simulator sim) 
	{
		leftHeadLight = new SpotLight();
        leftHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
        leftHeadLight.setSpotRange(100);
        leftHeadLight.setSpotInnerAngle(11*FastMath.DEG_TO_RAD);
        leftHeadLight.setSpotOuterAngle(25*FastMath.DEG_TO_RAD);
        sim.getSceneNode().addLight(leftHeadLight);
        
        rightHeadLight = new SpotLight();
        rightHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
        rightHeadLight.setSpotRange(100);
        rightHeadLight.setSpotInnerAngle(11*FastMath.DEG_TO_RAD);
        rightHeadLight.setSpotOuterAngle(25*FastMath.DEG_TO_RAD);
        sim.getSceneNode().addLight(rightHeadLight);
	}
	
	
	private void setupTrailer() 
	{
		String trailerModelPath = "Models/Cars/drivingCars/CarRedTrailer/Car.scene";
		float trailerMass = 100;
		
		// load trailer (model and physics)
		CarModelLoader trailerModelLoader = new CarModelLoader(sim, this, trailerModelPath, trailerMass);
		trailerControl = trailerModelLoader.getCarControl();
		sim.getPhysicsSpace().add(trailerControl);
		trailerNode = trailerModelLoader.getCarNode();
		sim.getSceneNode().attachChild(trailerNode);
		
		/*
		// apply joint
		HingeJoint joint=new HingeJoint(carNode.getControl(VehicleControl.class),
				trailerNode.getControl(VehicleControl.class),
		        new Vector3f(0f, 0f, 3f),    // pivot point local to carNode
		        new Vector3f(0f, 0f, -1.5f), // pivot point local to trailerNode 
		        Vector3f.UNIT_Y, 			 // DoF Axis of carNode (Y axis)
		        Vector3f.UNIT_Y);        	 // DoF Axis of trailerNode (Y axis)
		joint.setCollisionBetweenLinkedBodys(false);
		joint.setLimit(-FastMath.HALF_PI, FastMath.HALF_PI);	        
		sim.getPhysicsSpace().add(joint);
		*/
		
		Box sphere = new Box(0.1f, 0.1f, 0.1f);
		Geometry spatial = new Geometry("box", sphere);
		CollisionShape boxShape = CollisionShapeFactory.createBoxShape(spatial);
		PhysicsRigidBody connector = new PhysicsRigidBody(boxShape, 1f);
		sim.getPhysicsSpace().add(connector);	
		
		
		// apply joint1
		HingeJoint joint1 = new HingeJoint(carNode.getControl(VehicleControl.class),
				connector,
		        new Vector3f(0f, 0f, 2.5f),  // pivot point local to carNode
		        new Vector3f(0f, 0f, 0f), 	 // pivot point local to connector 
		        Vector3f.UNIT_Y, 			 // DoF Axis of carNode (Y axis)
		        Vector3f.UNIT_Y);        	 // DoF Axis of connector (Y axis)
		joint1.setCollisionBetweenLinkedBodys(false);
		joint1.setLimit(-FastMath.HALF_PI, FastMath.HALF_PI);	        
		sim.getPhysicsSpace().add(joint1);
		
		// apply joint
		HingeJoint joint2 = new HingeJoint(connector,
				trailerNode.getControl(VehicleControl.class),
		        new Vector3f(0f, 0f, 0f),    // pivot point local to connector
		        new Vector3f(0f, 0f, -1.5f), // pivot point local to trailerNode 
		        Vector3f.UNIT_Y, 			 // DoF Axis of connector (Y axis)
		        Vector3f.UNIT_Y);        	 // DoF Axis of trailerNode (Y axis)
		joint2.setCollisionBetweenLinkedBodys(false);
		joint2.setLimit(-FastMath.HALF_PI, FastMath.HALF_PI);	        
		sim.getPhysicsSpace().add(joint2);
	}
	
	
	private void setupReferencePoints() 
	{
		// add node representing position of front box
		Box frontBox = new Box(0.01f, 0.01f, 0.01f);
		frontGeometry = new Geometry("frontBox", frontBox);
        frontGeometry.setLocalTranslation(0, 0, -1);
		Material frontMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
		frontMaterial.setColor("Color", ColorRGBA.Red);
		frontGeometry.setMaterial(frontMaterial);
		Node frontNode = new Node();
		frontNode.attachChild(frontGeometry);
		frontNode.setCullHint(CullHint.Always);
		getCarNode().attachChild(frontNode);
		
		// add node representing position of center box
		Box centerBox = new Box(0.01f, 0.01f, 0.01f);
		centerGeometry = new Geometry("centerBox", centerBox);
		centerGeometry.setLocalTranslation(0, 0, 0);
		Material centerMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
		centerMaterial.setColor("Color", ColorRGBA.Green);
		centerGeometry.setMaterial(centerMaterial);
		Node centerNode = new Node();
		centerNode.attachChild(centerGeometry);
		centerNode.setCullHint(CullHint.Always);
		getCarNode().attachChild(centerNode);
	}
	
	
	public float getMass()
	{
		return mass;
	}
	
	public float getMinSpeed()
	{
		return minSpeed;
	}
	
	public void setMinSpeed(float minSpeed)
	{
		this.minSpeed = minSpeed;
	}
	
	public float getMaxSpeed()
	{
		return maxSpeed;
	}
	
	public void setMaxSpeed(float maxSpeed)
	{
		this.maxSpeed = maxSpeed;
	}
	
	public float getAcceleration()
	{
		return acceleration;
	}
	
	public float getMaxBrakeForce()
	{
		return maxBrakeForce;
	}
	
	public float getDecelerationFreeWheel()
	{
		return decelerationFreeWheel;
	}
	
	
	public Node getCarNode()
	{
		return carNode;
	}
	
	
	public VehicleControl getCarControl()
	{
		return carControl;
	}
	
	
	public CarModelLoader getCarModel()
	{
		return carModel;
	}
	
	
	public Transmission getTransmission()
	{
		return transmission;
	}
	
	
	public PowerTrain getPowerTrain()
	{
		return powerTrain;
	}
	
	
	public void setToNextResetPosition() 
	{
		int numberOfResetPoints = Simulator.getResetPositionList().size();
		
		setToResetPosition(resetPositionCounter);
		resetPositionCounter = (resetPositionCounter + 1) % numberOfResetPoints;
	}

	
	public void setToResetPosition(int keyNumber) 
	{
		int numberOfResetPoints = Simulator.getResetPositionList().size();
		
		if (keyNumber < numberOfResetPoints) 
		{
			ResetPosition reset = Simulator.getResetPositionList().get(keyNumber);
			
			Vector3f location = reset.getLocation();
			Quaternion rotation = reset.getRotation();
			
			setPosition(location);
			setRotation(rotation);
		}
	}
	
	public void setPosition(Vector3f v) 
	{
		setPosition(v.x, v.y, v.z);
	}
	
	
	public void setPosition(float x, float y, float z) 
	{
		previousPosition = new Vector3f(x,y,z);
		
		carControl.setPhysicsLocation(previousPosition);
		carControl.setLinearVelocity(Vector3f.ZERO);
		carControl.setAngularVelocity(Vector3f.ZERO);
		carControl.resetSuspension();
		
		if(trailerControl != null)
		{
			trailerControl.setPhysicsLocation(previousPosition);
			trailerControl.setLinearVelocity(Vector3f.ZERO);
			trailerControl.setAngularVelocity(Vector3f.ZERO);
			trailerControl.resetSuspension();
		}
	}

	
	public Vector3f getPosition() 
	{
		return carControl.getPhysicsLocation();
	}
	
	
	public Vector3d getGeoPosition() 
	{
		return GeoPosition.modelToGeo(getPosition());
	}

	// Yelly: I guess the heading degree 0 corresponds to x coordinate 0.
	// after experiment, I found this heading begins from nagative z to positive x
	public float getHeadingDegree() 
	{
		// get Euler angles from rotation quaternion
		float[] angles = carControl.getPhysicsRotation().toAngles(null);
		
		// heading in radians
		float heading = -angles[1];
		
		// normalize radian angle
		float fullAngle = 2*FastMath.PI;
		float angle_rad = (heading + fullAngle) % fullAngle;
		
		// convert radian to degree
		return angle_rad * 180/FastMath.PI;
	}
	
	
	public float getSlope()
	{
		// get Euler angles from rotation quaternion
		float[] angles = carControl.getPhysicsRotation().toAngles(null);
		
		// slope in radians (with correction due to different suspension heights)
		return angles[0] - 0.031765f;
	}
	
	
	public float getSlopeDegree()
	{
		// convert radian to degree and round to one decimal
		return ((int)(getSlope() * 180/FastMath.PI *10f))/10f;
	}
	
	
	public void setRotation(Quaternion q) 
	{	
		setRotation(q.getX(), q.getY(), q.getZ(), q.getW());
	}
	
	
	public void setRotation(float x, float y, float z, float w) 
	{
		Quaternion rotation = new Quaternion(x,y,z,w);
		
		// compensate that car is actually driving backwards
		float[] angles = rotation.toAngles(null);
		angles[1] = -angles[1]; //FIXME
		rotation = new Quaternion().fromAngles(angles);
		
		carControl.setPhysicsRotation(rotation);
		carControl.setLinearVelocity(Vector3f.ZERO);
		carControl.setAngularVelocity(Vector3f.ZERO);
		carControl.resetSuspension();
		
		if(trailerControl != null)
		{
			trailerControl.setPhysicsRotation(rotation);
			trailerControl.setLinearVelocity(Vector3f.ZERO);
			trailerControl.setAngularVelocity(Vector3f.ZERO);
			trailerControl.resetSuspension();
		}
	}
	
	
	public Quaternion getRotation() 
	{
		return carControl.getPhysicsRotation();
	}
	
	
	/**
	 * Accelerates the car forward or backwards. Does it by accelerating both
	 * suspensions (4WD). If you want a front wheel drive, comment out the
	 * rearSuspension.accelerate(direction) line. If you want a rear wheel drive
	 * car comment out the other one.
	 * 
	 * @param intensity
	 *            -1 for full ahead and 1 for full backwards
	 */
	public void setAcceleratorPedalIntensity(float intensity) 
	{
		acceleratorPedalIntensity = intensity;
	}

	
	public float getAcceleratorPedalIntensity() 
	{
		return Math.abs(acceleratorPedalIntensity);
	}


	/**
	 * Brake pedal
	 * 
	 * @param intensity
	 *            1 for full brake, 0 no brake at all
	 */
	public void setBrakePedalIntensity(float intensity) 
	{
		brakePedalIntensity = intensity;
		SpeedControlCenter.stopBrakeTimer();
	}
	
	
	public float getBrakePedalIntensity() 
	{
		return brakePedalIntensity;
	}

	
	/**
	 * Clutch pedal
	 * 
	 * @param intensity
	 *            1 for fully pressed, 0 for fully released clutch pedal
	 */
	public void setClutchPedalIntensity(float intensity)
	{		
		clutchPedalIntensity = intensity;
	}
	
	
	public float getClutchPedalIntensity() 
	{
		return clutchPedalIntensity;
	}

	
	float previousClutchPedalIntensity = 0;
	public float getTraction() 
	{
		if(clutchPedalIntensity < 0.4f && isEngineOn())
		{
			if(FastMath.abs(previousClutchPedalIntensity-clutchPedalIntensity) < 0.05f)
				traction = Math.min(1.0f, traction + 0.0005f);
			else
				traction = (0.4f - clutchPedalIntensity)*2.5f;
		}
		else
			traction = 0;
		
		previousClutchPedalIntensity = clutchPedalIntensity;
		
		return traction;
	}
	
	
	/**
	 * Free wheel
	 */
	public void resetPedals()
	{
		// reset pedals to initial position
		acceleratorPedalIntensity = 0;
		brakePedalIntensity = 0;
	}
	
	
	/**
	 * Steers the front wheels.
	 * 
	 * @param direction
	 *            1 for right and -1 for left
	 */
	public void steer(final float direction) 
	{
		//System.out.println("in car.steer(), direction argument: " + direction);
		carControl.steer(direction + steeringInfluenceByCrosswind);
		setSteeringWheelState(direction);
	}

	
	public void setSteeringWheelState(float steeringWheelState) 
	{
		this.steeringWheelState = steeringWheelState;
	}

	
	public float getSteeringWheelState() 
	{
		return steeringWheelState;
	}
	
	
	public float getSteeringInfluenceByCrosswind() 
	{
		return steeringInfluenceByCrosswind;
	}
	

	/**
	 * Unsteer the front wheels
	 */
	public void unsteer() 
	{
		carControl.steer(steeringInfluenceByCrosswind);
		setSteeringWheelState(0);
	}

	
	/**
	 * To get the car speed for using in a HUD
	 * 
	 * @return velocity of the car
	 */
	public float getCurrentSpeedMs() 
	{
		return (getCurrentSpeedKmh()/3.6f);
	}
	

	public float getCurrentSpeedMsRounded()
	{
		return ((int)(getCurrentSpeedMs() * 100)) / 100f;
	}
	
	
	public float getCurrentSpeedKmh()
	{
		return FastMath.abs(carControl.getCurrentVehicleSpeedKmHour());
	}
	
	
	public float getCurrentSpeedKmhRounded()
	{
		return ((int)(getCurrentSpeedKmh() * 100)) / 100f;
	}
	
	
	public float getMileage()
	{
		updateDistanceOfCurrentFrame();
		
		if(distanceOfCurrentFrame > 0.001f)
			mileage += distanceOfCurrentFrame;
		
		return mileage;
	}

	
	private void updateDistanceOfCurrentFrame()
	{
		// compute distance since last measurement
		Vector3f currentPosition = getPosition();
		distanceOfCurrentFrame = previousPosition.distance(currentPosition);
		
		// update values
		previousPosition = currentPosition;
	}
	
	
	public float getDistanceOfCurrentFrameInKm()
	{
		return distanceOfCurrentFrame/1000f;
	}
	
	
	public String getMileageString()
	{
		float mileage = getMileage();
		if(mileage < 1000)
			return ((int)mileage) + " m";
		else
			return ((int)(mileage/10f))/100f + " km";
	}
	
	
	public void resetMileage()
	{
		mileage = 0;
	}


	public Vector3f getInitialPosition() 
	{
		return initialPosition;
	}

	
	public Quaternion getInitialRotation() 
	{
		return initialRotation;
	}
	
	
	public void toggleLight() 
	{
		if(lightIntensity < 1)
			lightIntensity = 1;
		else if(lightIntensity < 2)
			lightIntensity = 2;
		else
			lightIntensity = 0;
	}
	
	
	public boolean isLightOn()
	{
		return (lightIntensity != 0);
	}
	

	public boolean isEngineOn() 
	{
		return engineOn;
	}
	
	
	public void setEngineOn(boolean engineOn) 
	{
		this.engineOn = engineOn;
		resetPedals();
		
		showEngineStatusMessage(engineOn);
		
		if(engineOn)
			AudioCenter.startEngine();
		else
			AudioCenter.stopEngine();
	}


	protected void showEngineStatusMessage(boolean engineOn) 
	{
		if(this instanceof SteeringCar)
		{
			if(engineOn)
				PanelCenter.getMessageBox().addMessage("Engine on", 2);
			else
				PanelCenter.getMessageBox().addMessage("Engine off. Press 'e' to start.", 0);
		}
	}

	
	public void setAutoAcceleration(boolean isAutoAcceleration) 
	{
		this.isAutoAcceleration = isAutoAcceleration;
	}
	
	
	public boolean isAutoAcceleration() 
	{
		return isAutoAcceleration;
	}

	
	public void setCruiseControl(boolean isCruiseControl) 
	{
		this.targetSpeedCruiseControl = getCurrentSpeedKmh();
		this.isCruiseControl = isCruiseControl;
	}
	
	
	public boolean isCruiseControl() 
	{
		return isCruiseControl;
	}
	
	
	public Simulator getSimulator() 
	{
		return sim;
	}


	public String getLightState() 
	{
		if(lightIntensity == 2)
			return "HighBeam";
		else if(lightIntensity == 1)
			return "LowBeam";
		else
			return "Off";
	}
	
	
	public void setBrakeLight(boolean setToOn)
	{
		lightTexturesContainer.setBrakeLight(setToOn);
	}
	
	
	public boolean isBrakeLightOn()
	{
		return lightTexturesContainer.isBrakeLightOn();
	}

	
	public void setTurnSignal(TurnSignalState turnSignalState)
	{
		lightTexturesContainer.setTurnSignal(turnSignalState);
	}
	

	public TurnSignalState getTurnSignal() 
	{
		return lightTexturesContainer.getTurnSignal();
	}
	
	
	public void close()
	{
		lightTexturesContainer.close();
	}
	
	
	// below here comes distance to start utils:
	// implemented by Yelly on 02/08/2017
	
	// returns the (normalized£© perpendicular vector on right side of road direction
	// by saying "right side", I mean the direction vector from previous past way point to it should be smaller than road direction vector
	// but this might be problematic with transformed/rotated driver's view direction, which needs further experimentation
	// ADD-ON: I dealt with this problem in update method which calls this method, see the comment in that method
	
	//using left because left represents positive values, right represents negative
	protected Vector3f leftPerpendicular( Vector3f originVect) {
		// assume y coordinate constant
		// construct normalized 2D vector for vector Vr
		float scaler = (float) FastMath.sqrt(((originVect.x * originVect.x) + (originVect.z * originVect.z)));
		Vector3f Vnor = new Vector3f(originVect.x/scaler, 0, originVect.z/scaler); // this vector is actually in 2D, 
																	//I use 3D vector to represent just for ease to read
																	// same for the following two perpendicular vectors
		//get perpendicular
		
		// to say left/right, I have to decide the actual direction again,
		// ADD-ON: In 2D case, the driver should be looking from negative y to positive y so that I could treat 3D-x as 2D-x, and 3D-z as 2D-y, 
		// otherwise the result should be another vector with reversed sign.
		// maybe examine the driver's looking direction by examine sign of y of vector from car position to waypoint? (these computations are all based on 2D case and y points up)
		return new Vector3f(-Vnor.z, 0, Vnor.x).mult((float)lookDir);
	}
	
	private Vector3f leftEdgePt(Vector3f wp, Vector3f roadDir, float width) {
		// assume y coordinate constant
		// construct normalized 2D vector for vector Vr
		// use roadDir.normalize() instead?
		float scaler = (float) Math.sqrt((double)((roadDir.x * roadDir.x) + (roadDir.z * roadDir.z)));
		Vector3f Vnor = new Vector3f(roadDir.x/scaler, 0, roadDir.z/scaler); // this vector is actually in 2D, 
																	//I use 3D vector to represent just for ease to read
																	// same for the following two perpendicular vectors
		//get perpendicular
		
		// to say left/right, I have to decide the actual direction again,
		// ADD-ON: In 2D case, the driver should be looking from negative y to positive y so that I could treat 3D-x as 2D-x, and 3D-z as 2D-y, 
		// otherwise the result should be another vector with reversed sign.
		// maybe examine the driver's looking direction by examine sign of y of vector from car position to waypoint? (these computations are all based on 2D case and y points up)

		Vector3f Vp2d = new Vector3f(-Vnor.z, 0, Vnor.x).mult((float)lookDir);
		return new Vector3f(wp.x + (width/2)*Vp2d.x, wp.y, wp.z + (width/2)*Vp2d.z);
	}
	
	private Vector3f rightEdgePt( Vector3f wp, Vector3f roadDir, float width) {
		// assume y coordinate constant
		// construct normalized 2D vector for vector Vr
		float scaler = (float) Math.sqrt((double)((roadDir.x * roadDir.x) + (roadDir.z * roadDir.z)));
		Vector3f Vnor = new Vector3f(roadDir.x/scaler, 0, roadDir.z/scaler); // this vector is actually in 2D, 
																	//I use 3D vector to represent just for ease to read
																	// same for the following two perpendicular vectors
		//get perpendicular
		
		// to say left/right, I have to decide the actual direction again,
		// ADD-ON: In 2D case, the driver should be looking from negative y to positive y so that I could treat 3D-x as 2D-x, and 3D-z as 2D-y, 
		// otherwise the result should be another vector with reversed sign.
		// maybe examine the driver's looking direction by examine sign of y of vector from car position to waypoint? (these computations are all based on 2D case and y points up)

		Vector3f Vp2d = new Vector3f(Vnor.z, 0, -Vnor.x).mult((float)lookDir);
		return new Vector3f(wp.x + (width/2)*Vp2d.x, wp.y, wp.z + (width/2)*Vp2d.z);
	}
	
	protected void updateLookingDir() {
		//System.out.println("my position y: " + this.getPosition().y + ", waypoint position y: " + this.sim.getCriticalPoints().get(((prevPastIndex<0)?0:prevPastIndex)).getPos().y);
		if(this.getPosition().subtract(this.sim.getCriticalPoints().get(((prevPastIndex<0)?0:prevPastIndex)).getPos()).y < 0) {
			// looking from negative y to positive y, left is left
			this.lookDir = 1;
		}
		else {
			// looking from positive y to negative y, left is right
			this.lookDir = -1;
		}
	}
	
	// returns the updated value of prevPastIndex
	protected void updatePrevPastIndex() {
		// update prevPastIndex
		
		// logic: start from current prevPastIndex, determine the current position is at what side of it
		// then go examine next criticalWayPoint according to the side 
		int index = this.prevPastIndex;
		int cp_num = this.sim.getCp_num();
		
		if(index >= 0) {
			if(aheadCP(index)) {
				while(index < cp_num-1 && aheadCP(index+1)) index++;
			}
			else {
				index--;
				while(index>=0 && !aheadCP(index)) index--;
			}
		}
		else {
			if(aheadCP(0)) {
				index = 0;
				while(index < cp_num-1 && aheadCP(index+1)) index++;
			}
		}
		
		this.prevPastIndex = index;
	}
	
	protected void updateDistanceToStart() {
		// update distance to start (according to updated prevPastIndex)
		// logic: the updated dts is the dts of the prevPastIndex plus the distance from the current position to the line representing the dts of the previous past point (perpendicular distance)
		// compute the perpendicularDist equals to the project length the vector from previous past point to current position onto the vector pointing from previous past critical point to the next
		Vector3f roadVect = curRoadVect(prevPastIndex);
		// in my first step, as simplification, I assume y coordinate is constant, meaning the road has no ups and downs
		Vector3f fromPrevToCur;
		if(prevPastIndex == -1) {
			fromPrevToCur = pointingVect(this.sim.getCriticalPoints().get(0).getPos(), this.getPosition());
		}
		else {
			fromPrevToCur = pointingVect(this.sim.getCriticalPoints().get(prevPastIndex).getPos(), this.getPosition());
		}

		float dotproduct = roadVect.dot(new Vector3f(fromPrevToCur.x, roadVect.y, fromPrevToCur.z));// I don't use fromPrevToCur.dot(roadVect) because the waypoints could have a y diviation from car position
		float perpendicularDist = dotproduct/distBetweenNearestCPs();
		if(prevPastIndex == -1) {
			this.dts = perpendicularDist;
		}
		else this.dts =  (this.sim.getCriticalPoints().get(prevPastIndex).getDts() + perpendicularDist);
	}

	// assume this method is called in intervals small enough so that the road does not twist much in between
	protected void updateLateralPos() {
		Vector3f roadVect = curRoadVect(prevPastIndex);
		// in my first step, as simplification, I assume y coordinate is constant, meaning the road has no ups and downs
		Vector3f fromPrevToCur;
		if(prevPastIndex == -1) {
			fromPrevToCur = pointingVect(this.sim.getCriticalPoints().get(0).getPos(), this.getPosition());
		}
		else {
			fromPrevToCur = pointingVect(this.sim.getCriticalPoints().get(prevPastIndex).getPos(), this.getPosition());
		}
		// also use dot product to get the lateral position:
		
		this.lateralPos = fromPrevToCur.dot(leftPerpendicular(roadVect)); // notice: leftPerpendicular() produces normalized vector, so there's no need to divide further
	}
	
	private int fromDtsToLastIndex(float dts) {
		int index = this.prevPastIndex;
		int cp_num = this.sim.getCp_num();
		
		if(index >= 0) {
			if(this.sim.getCriticalPoints().get(index).getDts() <= dts) {
				while(index < cp_num-1 && this.sim.getCriticalPoints().get(index+1).getDts() < dts) { index++;}
			}
			else {
				index--;
				while(index>=0 && this.sim.getCriticalPoints().get(index).getDts() < dts) { index--;}
			}
		}
		else {
			if(this.sim.getCriticalPoints().get(0).getDts() < dts) {
				index = 0;
				while(index < cp_num-1 && this.sim.getCriticalPoints().get(index+1).getDts() < dts) { index++;}
			}
		}
		
		return index;
	}

	// use dot product to indicate current position is on which side
	// if dot product result is positive, then the current position is on forwarding direction of prevPastIndex, so the new value would be no less than the current one
	// otherwise the current position is on back direction of prevPastIndex, so the value would be smaller than the current one.
	private boolean aheadCP(int pastIndex) {
		float dotproduct = curRoadVect(pastIndex).dot(pointingVect(this.sim.getCriticalPoints().get(pastIndex).getPos(), this.getPosition()));
		if(dotproduct>=0) {
			return true;
		}
		else {
			return false;
		}
	}
	
	// this method returns distance between this critical point (indicated by myIndex) and the previous critical point
	// this method is based on the assumption that either of the following cases applies:
	// 1. the road between this waypoint and the previous waypoint is straight
	// 2. the distance between this two waypoints is small enough so that the road between the two points could be regarded straight
	private float distBetweenNearestCPs(){
		if(prevPastIndex < -1 || prevPastIndex >= this.sim.getCp_num()) {
			System.err.println("illegal prevPastIndex appears");
			return -1f;
		}
		if(prevPastIndex == -1) return this.sim.getCriticalPoints().get(1).getPos().distance(this.sim.getCriticalPoints().get(0).getPos()) ;
		if(prevPastIndex < this.sim.getCp_num() -1) return this.sim.getCriticalPoints().get(prevPastIndex+1).getPos().distance(this.sim.getCriticalPoints().get(prevPastIndex).getPos());
		else {
			//if(prevPastIndex == cp_num - 1) 
			return this.sim.getCriticalPoints().get(prevPastIndex).getPos().distance(this.sim.getCriticalPoints().get(prevPastIndex-1).getPos());
		}
	}
	
	// this method overloads distBetweenNearestCPs above,
	// difference is that it accepts further waypoint index that's not the prevPastIndex
	// in this case, it returns the distance between the critical waypoint indicated by the argument and the next waypoint
	/*private float distBetweenNearestCPs(int ppindex){
		if(ppindex <= -1 || ppindex >= cp_num) {
			// in this case we don't allow ppindex to be the last index for criticalPoints array
			// however this error is neglectable by the calling method
			//System.err.println("illegal argument to distBetweenNearestCPs(ppindex) method");
			return -1f;
		}
		if(prevPastIndex == -1) return dist(criticalPoints.get(1).getPos(), criticalPoints.get(0).getPos());
		else {
			//if(prevPastIndex < cp_num -1) 
			return dist(criticalPoints.get(prevPastIndex+1).getPos(), criticalPoints.get(prevPastIndex).getPos());
		}
	}*/
	
	// this method returns vector pointing from p1 to p2.
	private Vector3f pointingVect(Vector3f p1, Vector3f p2) {
		return p2.subtract(p1);
	}
	
	private Vector3f curRoadVect(int ppindex) {
		if(ppindex == this.sim.getCp_num()-1) {
			// for the last critical point, the vector points from the one before last to the last
			return pointingVect(this.sim.getCriticalPoints().get(ppindex-1).getPos(), this.sim.getCriticalPoints().get(ppindex).getPos());
		}
		else if(ppindex == -1){
			// for the last critical point, the vector points from the first one to the second
			return pointingVect(this.sim.getCriticalPoints().get(0).getPos(), this.sim.getCriticalPoints().get(1).getPos());
		}
		else {
			// for all other critical points, the vector points from itself to the next one
			return pointingVect(this.sim.getCriticalPoints().get(ppindex).getPos(), this.sim.getCriticalPoints().get(ppindex+1).getPos());
		}
	}


	protected void steerTowardsPosition(Vector3f wayPoint) 
	{
		// get relative position of way point --> steering direction
		// -1: way point is located on the left side of the vehicle
		//  0: way point is located in driving direction 
		//  1: way point is located on the right side of the vehicle
		int steeringDirection = getRelativePosition(wayPoint);
		
		// get angle between driving direction and way point direction --> steering intensity
		// only consider 2D space (projection of WPs to xz-plane)
		Vector3f carFrontPos = frontGeometry.getWorldTranslation();
		Vector3f carCenterPos = centerGeometry.getWorldTranslation();
		float steeringAngle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, wayPoint, true);
		
		// compute steering intensity in percent
		//  0     degree =   0%
		//  11.25 degree =  50%
		//  22.5  degree = 100%
		// >22.5  degree = 100%
		float steeringIntensity = Math.max(Math.min(4*steeringAngle/FastMath.PI,1f),0f);
		

		// Shi Cao, adding divided by a number to reduce steering intensity.
		
		if (   steeringIntensity > 0.5  ) steeringIntensity = steeringIntensity / 2 ;
		else {
			if ( getCurrentSpeedKmh() > 80 ) steeringIntensity = steeringIntensity / 10 ;
			else if ( getCurrentSpeedKmh() > 30 ) steeringIntensity = steeringIntensity / 3 ;			
		}
		

		
		
			
		// apply steering instruction
		steer(steeringDirection*steeringIntensity);
		
		//System.out.println(steeringDirection*steeringIntensity);
	}

	
	protected int getRelativePosition(Vector3f wayPoint)
	{
		// get vehicles center point and point in driving direction
		Vector3f frontPosition = frontGeometry.getWorldTranslation();
		Vector3f centerPosition = centerGeometry.getWorldTranslation();
		
		// convert Vector3f to Point2D.Float, as needed for Line2D.Float
		Point2D.Float centerPoint = new Point2D.Float(centerPosition.getX(),centerPosition.getZ());
		Point2D.Float frontPoint = new Point2D.Float(frontPosition.getX(),frontPosition.getZ());
		
		// line in direction of driving
		Line2D.Float line = new Line2D.Float(centerPoint,frontPoint);
		
		// convert Vector3f to Point2D.Float
		Point2D point = new Point2D.Float(wayPoint.getX(),wayPoint.getZ());

		// check way point's relative position to the line
		if(line.relativeCCW(point) == -1)
		{
			// point on the left --> return -1
			return -1;
		}
		else if(line.relativeCCW(point) == 1)
		{
			// point on the right --> return 1
			return 1;
		}
		else
		{
			// point on line --> return 0
			return 0;
		}
	}
	
	////////////////////////////////////////////////////////////////////////////////////////
	// below here comes the computation for near point(angle) and far point (angle)
	// created by Yelly on 31/07/2017
	public float getNearFarPointAngleDegree(Vector3f nearFarPoint) {
		return (float) Math.toDegrees(getNearFarPointAngle(nearFarPoint));
	}

	
	// @param selfHeading - possibly given by car.getHeadingDegree(), it is in degree
	public float getNearFarPointAngle(Vector3f nearFarPoint) {
//		float heading_in_radian = (float) Math.toRadians(this.getHeadingDegree());
//		// Yelly's guess is that the heading degree 0 corresponds to x coordinate 0.
//		// that is, it rotates from negative z coordinate to positive x 
//		Vector3f headDir = new Vector3f(FastMath.sin(heading_in_radian), 0, -FastMath.cos(heading_in_radian));
//		Vector3f curToNearPt = nearFarPoint.subtract(this.getPosition()).normalize();
//		//float absAngle =  headDir.angleBetween(curToNearPt);
//		float absAngle = FastMath.acos(headDir.dot(curToNearPt));
//		
//		// now I need to decide sign of the angle (recall how I do it for lateral position computation), 
//		// when curToNearPt on left of headDir, indicator is positive
//		// if curToNearPt on right of headDir, indicator is negative
//		// if headDir points to the same direction as curToNearPt do, angle is zero and sign of indicator doesn't affect anything
//		
//		float signIndicator = headDir.dot(leftPerpendicular(curToNearPt)); 
//
//		// if the angle take heading direction as the reference, the sign will be reversed.
//		//return (signIndicator<0 )? -absAngle : absAngle;	
//		return (signIndicator<0 )? absAngle : -absAngle;	
		

		// get relative position of way point --> steering direction
		// -1: way point is located on the left side of the vehicle
		//  0: way point is located in driving direction 
		//  1: way point is located on the right side of the vehicle
		int steeringDirection = getRelativePosition(nearFarPoint);
		
		// get angle between driving direction and way point direction --> steering intensity
		// only consider 2D space (projection of WPs to xz-plane)
		Vector3f carFrontPos = frontGeometry.getWorldTranslation();
		Vector3f carCenterPos = centerGeometry.getWorldTranslation();
		float steeringAngle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, nearFarPoint, true);
		return steeringAngle * steeringDirection;
	}
	
	public Vector3f getNearPointPos() {
		// we assume the steering car always goes from smaller waypoint index to bigger index
		
		//float distAhead = Near_Point_Distance_Ahead;
		
		int ppindex =  this.prevPastIndex;
		int cp_num = this.sim.getCp_num();
		
		while(ppindex < cp_num - 1 && this.sim.getCriticalPoints().get(ppindex+1).getDts() - dts < Near_Point_Distance_Ahead) ppindex++; // now ppindex represents either 
						// 1. the last waypoint within 10-meter range; or 
						// 2. the last waypoint in the array, in which case the road will simply be regarded straight, and it's okay for later implementation in this method

		// recalling that we assume road between each adjacent waypoint pair is straight
		Vector3f roadVect = curRoadVect(ppindex);
		
		CriticalWayPoint lastcp;
		if(ppindex < 0 ) {
			// maybe it's more reasonable to take center of the road at current position of the steering car when the car is behind the start waypoint, 
			// but I don't want to look into this issue right now as it is a corner case that rarely happens. So leave it to further improvement
			lastcp = this.sim.getCriticalPoints().get(0); 
		}
		else  lastcp = this.sim.getCriticalPoints().get(ppindex);
		
		float distFromPrevCP = dts + Near_Point_Distance_Ahead - lastcp.getDts();
		return lastcp.getPos().add(roadVect.normalize().mult(distFromPrevCP));
	}
	
	// speed given in m/s
	public Vector3f getFarPointPos() {
		float thresholdDist = this.getCurrentSpeedMs() * Far_Point_Time_Ahead;
		
		TrafficCar targetCar = null;
		if(this.Far_Point_Follow_Vehicle && ((targetCar = trafficWithinThresholdDist()) != null)) {
			return targetCar.getPosition();
		}
		else {
			// get the near point position first, then move the point according to road curve type
			// below is the same code for getNearPointPos(), except that the Near_Point_Distance is replaced by thresholdDist
			// I don't call that method directly because I wanna use the ppindex variable in its implementation as well
			
			// we assume the steering car always goes from smaller waypoint index to bigger index
			
			//float distAhead = Near_Point_Distance_Ahead;
			
			int ppindex = prevPastIndex + 1;
			
			while(ppindex < this.sim.getCp_num() && this.sim.getCriticalPoints().get(ppindex).getDts() - dts <= thresholdDist) ppindex++;
			// after the while loop, ppindex should be one of the following:
			// 1. the first waypoint beyond 10-meter ahead
			// 2. equals to cp_num.
			
			ppindex -= 1; // now ppindex represents either 
							// 1. the last waypoint within 10-meter range; or 
							// 2. the last waypoint in the array, in which case the road will simply be regarded straight, and it's okay for later implementation in this method

			// recalling that we assume road between each adjacent waypoint pair is straight
			Vector3f roadVect = curRoadVect(ppindex);
			
			CriticalWayPoint lastcp;
			if(ppindex < 0 ) {
				// maybe it's more reasonable to take center of the road at current position of the steering car when the car is behind the start waypoint, 
				// but I don't want to look into this issue right now as it is a corner case that rarely happens. So leave it to further improvement
				lastcp = this.sim.getCriticalPoints().get(0); 
			}
			else  lastcp = this.sim.getCriticalPoints().get(ppindex);
			
			float distFromPrevCP = dts + thresholdDist - lastcp.getDts();
			Vector3f nearPt = new Vector3f(lastcp.getPos().add(roadVect.normalize().mult(distFromPrevCP)));
			
			int curve = curveTypeMajority(dts);
			switch(curve) {
			case 0:
				// straight road
				return nearPt;
			case 1:
				// left-turn road
				return leftEdgePt(nearPt, roadVect, sim.getCriticalPoints().get(ppindex).getWidth());
			case -1:
				// right-turn road
				return rightEdgePt(nearPt, roadVect, sim.getCriticalPoints().get(ppindex).getWidth());
			default:
					System.err.println("error curve type");
			}
		}
		
		return null;
	}
	
	// this method seems like obstacleInTheWay method of class TrafficCar but still different
	private TrafficCar trafficWithinThresholdDist() {
		ArrayList<TrafficObject> trafficObjectList = PhysicalTraffic.getTrafficObjectList();
		
		// check distance from other traffic (except oneself)
		float dts_nearest = 0;
		TrafficCar ret = null;
		for(TrafficObject vehicle : trafficObjectList){
			// suppose all vehicles' names begin with "vehicle"
			if(vehicle.getName().startsWith("vehicle"))	{
				TrafficCar trafficCar = (TrafficCar)vehicle;
				float dts_traffic = trafficCar.getDts();
				if(dts_traffic > this.dts && dts_traffic <= (this.dts + Road_Segment_Length* 2)) {
					if(ret == null || dts_traffic < dts_nearest) {
						ret = trafficCar;
						dts_nearest = dts_traffic;
					}
				}
			}
		}
		
		return ret;
	}


    // added by Yelly, to generate critical waypoints
	// used ONLY by researchers
	int wp_ind = 1;
	private final int straight_interv = 500, curve_interv = 5;
	
	private void writeIntersectionWaypoint() {
			try {
				this.sim.getWaypoint_writer().write("\t\t<wayPoint id=\"WayPoint_");
				this.sim.getWaypoint_writer().write(this.wp_ind);
				this.sim.getWaypoint_writer().write("\">\n");
				this.sim.getWaypoint_writer().write("\t\t\t<translation><vector jtype = \"java_lang_Float\" size = \"3\">");
				this.sim.getWaypoint_writer().write("<entry>");
				this.sim.getWaypoint_writer().write(Float.toString(getPosition().x));
				this.sim.getWaypoint_writer().write("</entry><entry>0.0</entry>");
				this.sim.getWaypoint_writer().write("<entry>");
				this.sim.getWaypoint_writer().write(Float.toString(getPosition().z));
				this.sim.getWaypoint_writer().write("</entry></vector></translation>\n");
				this.sim.getWaypoint_writer().write("\t\t\t<speed>40</speed>\n\t\t</wayPoint>\n");
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			this.wp_ind++;
	}

	
	// examine curve type for three segment, use the type with majority
	public int curveTypeMajority(float dts) {
		int retType = 0;
		// modification 18/08/2017
		// I should turn when I see the road ahead is turning instead of waiting until road at my position is turning
		int mainType = curveType(dts+Road_Segment_Length);
		int forwardType = curveType(dts+2*Road_Segment_Length);
		if(mainType == forwardType) retType = mainType;
		else {
			int backType = curveType(dts);
			if(backType == mainType || backType == forwardType) retType = backType;
			else retType = mainType; // if we get three different types, use the one computed from accurate dts.
		}
		//System.out.println("mainType: " + mainType + ", forwardType: " + forwardType  + ",curve type after vote: " + retType);
		//System.out.println("prevPastIndex: " + this.prevPastIndex);
		if(this.sim.isResearcher_driving()) {
			if(retType!=0) {
				if(this.sim.getUpd_cnt() % this.curve_interv == 0) writeIntersectionWaypoint();
			}
			else if(this.sim.getUpd_cnt() % this.straight_interv == 0) writeIntersectionWaypoint();
		}
		return retType;
		/*int type = curveType(dts);
		//System.out.println("look-at: " + this.lookDir);
		//System.out.println("curve type (without vote): " +type);
		return type;*/
	}
	
	// this method examine the road segment 10 meters ahead the steering car
	// assume there's at most one turn in 10-meter-segment
	// for left-turn, return 1; for right-turn, return -1; for straight road return 0
	public int curveType(float dts) {
		// use algorithm similar to what I used when computing near point to get the start & end waypoint for the (possible) arc

		int lastIndex = fromDtsToLastIndex(dts);
		
		if(lastIndex >= this.sim.getCp_num() - 2) return 0; // if there's at most one waypoint ahead, it's safe to assume the road ahead is straight

		int ppindex = lastIndex + 1;
		
		while(ppindex < this.sim.getCp_num() && this.sim.getCriticalPoints().get(ppindex).getDts() - dts <= Road_Segment_Length) ppindex++;
		// after the while loop, ppindex should be one of the following:
		// 1. the first waypoint beyond Road_Segment_Length-meter ahead
		// 2. equals to cp_num.
		
		if(ppindex - lastIndex <=2 ) {
			//System.out.println("there're at most 2 waypoints within Road_Segment_Length meters ahead,");
			// there're at most 2 waypoints within Road_Segment_Length meters ahead,
			// assume the road as straight
			return 0;
		}
		else {
			int startIndex, endIndex;
			if (ppindex - lastIndex == 3) {
				startIndex = lastIndex + 1;
				endIndex = ppindex;
			}
			else {
				//ppindex - this.prevPastIndex > 3
				startIndex = lastIndex + 1;
				endIndex = ppindex-1;
			}
			
			// compute the radius of road circle
			// the center of the circle is found by emitting a ray that's perpendicular to the start vector from the start vector
			// towards a plane contains and is perpendicular to the end vector (point)
			Vector3f circle_center_pos = new Vector3f();
			
			Vector3f startPos = this.sim.getCriticalPoints().get(startIndex).getPos();
			// first test the startRay to the left
			Vector3f startRayDir = leftPerpendicular(pointingVect(startPos, this.sim.getCriticalPoints().get(startIndex+1).getPos()));
			
			Vector3f endPos = this.sim.getCriticalPoints().get(endIndex).getPos();
			Vector3f endDir = pointingVect(this.sim.getCriticalPoints().get(endIndex-1).getPos(), endPos);
			
			float denominator = startRayDir.dot(endDir);
			if(denominator > -FastMath.FLT_EPSILON && denominator < FastMath.FLT_EPSILON) {
				//System.out.println("start vector is approximately parrallel to end vector,");
				// start vector is approximately parrallel to end vector, 
				// safe to assume the road segment as straight
				return 0;
			}

			Ray start_ray = new Ray(startPos, startRayDir);
			Plane end_plane = new Plane(endDir, endDir.dot(endPos));
			int ret;
			
			if(!start_ray.intersectsWherePlane(end_plane, circle_center_pos)) {
				// this is the case for right turn 
				// the ray does not intersect with the plane, means that the ray went to the wrong direction
				start_ray.setDirection(startRayDir.negateLocal()); // mind that by this method the direction of startRayDir changes too.
				boolean inter = start_ray.intersectsWherePlane(end_plane, circle_center_pos);
				assert(inter); // they should intersect because they're not parrallel
				ret = -1;
			}
			else {
				// this is the case for left turn
				ret = 1;
			}
			
			// define the radius as the average of the distance between the circle_center (at the intersection point) to start/end point
			float radius = (startPos.distance(circle_center_pos) + endPos.distance(circle_center_pos))/2f;
			//System.out.println("road radius computed: " + radius);
			
			if(radius < Max_Road_Radius) {
				// curly road
				return ret;
			}
			else {
				// straight road 
				return 0;
			}
			
		}
	}
}
