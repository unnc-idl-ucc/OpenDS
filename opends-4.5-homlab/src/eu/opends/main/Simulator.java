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


package eu.opends.main;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import eu.opends.profiler.BasicProfilerState;

import com.jme3.app.StatsAppState;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
//import com.jme3.app.state.VideoRecorderAppState;
import com.jme3.input.Joystick;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.niftygui.NiftyJmeDisplay;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.sun.javafx.application.PlatformImpl;

import de.lessvoid.nifty.Nifty;
import eu.opends.analyzer.DrivingTaskLogger;
import eu.opends.analyzer.DataWriter;
import eu.opends.audio.AudioCenter;
import eu.opends.basics.InternalMapProcessing;
import eu.opends.basics.SimulationBasics;
import eu.opends.camera.SimulatorCam;
import eu.opends.cameraFlight.CameraFlight;
import eu.opends.cameraFlight.NotEnoughWaypointsException;
import eu.opends.canbus.CANClient;
import eu.opends.car.ResetPosition;
import eu.opends.car.SteeringCar;
import eu.opends.customization.CriticalWayPoint;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.effects.EffectCenter;
import eu.opends.environment.TrafficLightCenter;
import eu.opends.eyetracker.EyetrackerCenter;
import eu.opends.hmi.HMICenter;
import eu.opends.input.ForceFeedbackJoystickController;
import eu.opends.input.KeyBindingCenter;
import eu.opends.knowledgeBase.KnowledgeBase;
import eu.opends.multiDriver.MultiDriverClient;
import eu.opends.niftyGui.DrivingTaskSelectionGUIController;
import eu.opends.oculusRift.OculusRift;
import eu.opends.reactionCenter.ReactionCenter;
import eu.opends.settingsController.SettingsControllerServer;
import eu.opends.taskDescription.contreTask.SteeringTask;
import eu.opends.taskDescription.tvpTask.MotorwayTask;
import eu.opends.taskDescription.tvpTask.ThreeVehiclePlatoonTask;
import eu.opends.tools.CollisionListener;
import eu.opends.tools.ObjectManipulationCenter;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.SpeedControlCenter;
import eu.opends.tools.Util;
import eu.opends.traffic.FollowBoxSettings;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.Waypoint;
import eu.opends.trigger.TriggerCenter;
import eu.opends.visualization.LightningClient;
import eu.opends.visualization.MoviePlayer;

/**
 * 
 * @author Rafael Math
 */
public class Simulator extends SimulationBasics
{
	
	public native static String GetPropertyValue(String aName) throws Exception;
	
	private final static Logger logger = Logger.getLogger(Simulator.class);

    private Nifty nifty;
    private int frameCounter = 0;
    private boolean drivingTaskGiven = false;
    private boolean initializationFinished = false;
    
    // added by Yelly:
    // critical waypoints marking center of the road
	private ArrayList<CriticalWayPoint> criticalPoints = new ArrayList<CriticalWayPoint>();
	private int cp_num = 0; // redundant variable, used to improve performance
	static final public float LaneWidth = 3.5f;
	private static BitmapText  testxt;

	// added by Yelly
	// researcher-driving, to generate way points (especially at intersection)
	private boolean researcher_driving = false;
	private FileWriter waypoint_writer;
	
    public boolean isResearcher_driving() {
		return researcher_driving;
	}

	public FileWriter getWaypoint_writer() {
		return waypoint_writer;
	}

	public static Logger getLogger() {
		return logger;
	}

	public Nifty getNifty() {
		return nifty;
	}

	public int getFrameCounter() {
		return frameCounter;
	}

	public boolean isDrivingTaskGiven() {
		return drivingTaskGiven;
	}

	public boolean isInitializationFinished() {
		return initializationFinished;
	}

	public ArrayList<CriticalWayPoint> getCriticalPoints() {
		return criticalPoints;
	}

	public int getCp_num() {
		return cp_num;
	}

	public boolean isDataWriterQuittable() {
		return dataWriterQuittable;
	}

	public DataWriter getDataWriter() {
		return dataWriter;
	}

	public boolean isShowStats() {
		return showStats;
	}

	public String getInstructionScreenID() {
		return instructionScreenID;
	}

	public static boolean isOculusRiftAttached() {
		return oculusRiftAttached;
	}

	private static Float gravityConstant;
	public static Float getGravityConstant()
	{
		return gravityConstant;
	}
	
	private SteeringCar car;
    public SteeringCar getCar()
    {
    	return car;
    }
    
    private PhysicalTraffic physicalTraffic;
    public PhysicalTraffic getPhysicalTraffic()
    {
    	return physicalTraffic;
    }
	
	private static DrivingTaskLogger drivingTaskLogger;
	public static DrivingTaskLogger getDrivingTaskLogger()
	{
		return drivingTaskLogger;
	}
	
	private boolean dataWriterQuittable = false;
	private DataWriter dataWriter;
	public DataWriter getMyDataWriter() 
	{
		return dataWriter;
	}
	
	private LightningClient lightningClient;
	public LightningClient getLightningClient() 
	{
		return lightningClient;
	}
	
	private static CANClient canClient;
	public static CANClient getCanClient() 
	{
		return canClient;
	}
	
	private MultiDriverClient multiDriverClient;
	public MultiDriverClient getMultiDriverClient() 
	{
		return multiDriverClient;
	}
	
	private TriggerCenter triggerCenter = new TriggerCenter(this);
	public TriggerCenter getTriggerCenter()
	{
		return triggerCenter;
	}

	private static List<ResetPosition> resetPositionList = new LinkedList<ResetPosition>();
	public static List<ResetPosition> getResetPositionList() 
	{
		return resetPositionList;
	}

	private boolean showStats = false;	
	public void showStats(boolean show)
	{
		showStats = show;
		setDisplayFps(show);
    	setDisplayStatView(show);
    	
    	if(show)
    		getCoordinateSystem().setCullHint(CullHint.Dynamic);
    	else
    		getCoordinateSystem().setCullHint(CullHint.Always);
	}
	
	public void toggleStats()
	{
		showStats = !showStats;
		showStats(showStats);
	}
	
	private CameraFlight cameraFlight;
	public CameraFlight getCameraFlight()
	{
		return cameraFlight;
	}
	
	private SteeringTask steeringTask;
	public SteeringTask getSteeringTask()
	{
		return steeringTask;
	}
	
	private ThreeVehiclePlatoonTask threeVehiclePlatoonTask;
	public ThreeVehiclePlatoonTask getThreeVehiclePlatoonTask()
	{
		return threeVehiclePlatoonTask;
	}
	
	private MotorwayTask motorwayTask;
	public MotorwayTask getMotorwayTask()
	{
		return motorwayTask;
	}
	
	private MoviePlayer moviePlayer;
	public MoviePlayer getMoviePlayer()
	{
		return moviePlayer;
	}
	
	private ReactionCenter reactionCenter;
	public ReactionCenter getReactionCenter()
	{
		return reactionCenter;
	}
	
	private EffectCenter effectCenter;
	public EffectCenter getEffectCenter()
	{
		return effectCenter;
	}
	
	private ObjectManipulationCenter objectManipulationCenter;
	public ObjectManipulationCenter getObjectManipulationCenter()
	{
		return objectManipulationCenter;
	}
	
	private String instructionScreenID = null;
	public void setInstructionScreen(String ID)
	{
		instructionScreenID = ID;
	}
	
	private SettingsControllerServer settingsControllerServer;
	public SettingsControllerServer getSettingsControllerServer()
	{
		return settingsControllerServer;
	}	
	
	private EyetrackerCenter eyetrackerCenter;
	public EyetrackerCenter getEyetrackerCenter()
	{
		return eyetrackerCenter;
	}
	
	
	private static String outputFolder;
	public static String getOutputFolder()
	{
		return outputFolder;
	}
	
	public static boolean oculusRiftAttached = false;/*
    private static OculusRift oculusRift;
	public static OculusRift getOculusRift()
	{
		return oculusRift;
	}
	*/
	
	private ForceFeedbackJoystickController joystickSpringController;
	public ForceFeedbackJoystickController getJoystickSpringController()
	{
		return joystickSpringController;
	}

	
    @Override
    public void simpleInitApp()
    {
    	showStats(false);
    	
    	if(drivingTaskGiven)
    		simpleInitDrivingTask(SimulationDefaults.drivingTaskFileName, SimulationDefaults.driverName);
    	else
    		initDrivingTaskSelectionGUI();
    }
    
    
	private void initDrivingTaskSelectionGUI() 
	{
		NiftyJmeDisplay niftyDisplay = new NiftyJmeDisplay(assetManager, inputManager, audioRenderer, guiViewPort);
    	
    	// Create a new NiftyGUI object
    	nifty = niftyDisplay.getNifty();
    		
    	String xmlPath = "Interface/DrivingTaskSelectionGUI.xml";
    	
    	// Read XML and initialize custom ScreenController
    	nifty.fromXml(xmlPath, "start", new DrivingTaskSelectionGUIController(this, nifty));
    		
    	// attach the Nifty display to the gui view port as a processor
    	guiViewPort.addProcessor(niftyDisplay);
    	
    	// disable fly cam
    	flyCam.setEnabled(false);
	}
	
	
	public void closeDrivingTaskSelectionGUI() 
	{
		nifty.exit();
        inputManager.setCursorVisible(false);
        flyCam.setEnabled(true);
	}


    public void simpleInitDrivingTask(String drivingTaskFileName, String driverName)
    {
    	stateManager.attach(new BasicProfilerState(false));
    	
    	SimulationDefaults.drivingTaskFileName = drivingTaskFileName;
    	
    	Util.makeDirectory("analyzerData");
    	outputFolder = "analyzerData/" + Util.getDateTimeString();
    	
    	initDrivingTaskLayers();
    	
    	// show stats if set in driving task
    	showStats(settingsLoader.getSetting(Setting.General_showStats, false));
    	
    	// check Oculus Rift mode: auto, enabled, disabled
    	String oculusAttachedString = settingsLoader.getSetting(Setting.OculusRift_isAttached, 
    			SimulationDefaults.OculusRift_isAttached);
		if(oculusAttachedString.equalsIgnoreCase("enabled"))
			oculusRiftAttached = true;
		else if(oculusAttachedString.equalsIgnoreCase("disabled"))
			oculusRiftAttached = false;
		
    	// sets up physics, camera, light, shadows and sky
    	super.simpleInitApp();
		
    	// set gravity
    	gravityConstant = drivingTask.getSceneLoader().getGravity(SimulationDefaults.gravity);
    	getPhysicsSpace().setGravity(new Vector3f(0, -gravityConstant, 0));	
    	//getPhysicsSpace().setAccuracy(0.008f); //TODO comment to set accuracy to 0.0166666 ?
    	//getPhysicsSpace().setAccuracy(0.011f); // new try
    	
    	PanelCenter.init(this);
	
        Joystick[] joysticks = inputManager.getJoysticks();
        if(joysticks != null)
        	for (Joystick joy : joysticks)
        		System.out.println("Connected joystick: " + joy.toString());
        
    	//load map model
		new InternalMapProcessing(this);
		
		// start trafficLightCenter
		trafficLightCenter = new TrafficLightCenter(this);
		
		// Yelly changed the order of initializing physical vehicles and steering car
		// to properly form the critical element list in constructor of SteeringCar
		
		// initialize physical vehicles
		physicalTraffic = new PhysicalTraffic(this);
		//physicalTraffic.start(); //TODO
		
		// create and place steering car
		car = new SteeringCar(this);
		
		// open TCP connection to KAPcom (knowledge component) [affects the driver name, see below]
		if(settingsLoader.getSetting(Setting.KnowledgeManager_enableConnection, SimulationDefaults.KnowledgeManager_enableConnection))
		{
			String ip = settingsLoader.getSetting(Setting.KnowledgeManager_ip, SimulationDefaults.KnowledgeManager_ip);
			if(ip == null || ip.isEmpty())
				ip = "127.0.0.1";
			int port = settingsLoader.getSetting(Setting.KnowledgeManager_port, SimulationDefaults.KnowledgeManager_port);
					
			//KnowledgeBase.KB.setConnect(true);
			KnowledgeBase.KB.setCulture("en-US");
			KnowledgeBase.KB.Initialize(this, ip, port);
			KnowledgeBase.KB.start();
		}
		
		// sync driver name with KAPcom. May provide suggestion for driver name if NULL.
		//driverName = KnowledgeBase.User().initUserName(driverName);  
		
		if(driverName == null || driverName.isEmpty())
			driverName = settingsLoader.getSetting(Setting.General_driverName, SimulationDefaults.driverName);
    	SimulationDefaults.driverName = driverName;
		
        // setup key binding
		keyBindingCenter = new KeyBindingCenter(this);
        
        AudioCenter.init(this);

        // setup camera settings
        cameraFactory = new SimulatorCam(this, car);
        

		
		// init trigger center
		triggerCenter.setup();
		
		// init HMICenter
		HMICenter.init(this);



		// open TCP connection to Lightning
		if(settingsLoader.getSetting(Setting.ExternalVisualization_enableConnection, SimulationDefaults.Lightning_enableConnection))
		{
			lightningClient = new LightningClient();
		}
		
		// open TCP connection to CAN-bus
		if(settingsLoader.getSetting(Setting.CANInterface_enableConnection, SimulationDefaults.CANInterface_enableConnection))
		{
			canClient = new CANClient(this);
			canClient.start();
		}
		
		if(settingsLoader.getSetting(Setting.MultiDriver_enableConnection, SimulationDefaults.MultiDriver_enableConnection))
		{
			multiDriverClient = new MultiDriverClient(this, driverName);
			multiDriverClient.start();
		}
		
		drivingTaskLogger = new DrivingTaskLogger(outputFolder, driverName, drivingTask.getFileName());
		
		SpeedControlCenter.init(this);
		
		try {
			
			// attach camera to camera flight
			cameraFlight = new CameraFlight(this);
			
		} catch (NotEnoughWaypointsException e) {

			// if not enough way points available, attach camera to driving car
			car.getCarNode().attachChild(cameraFactory.getMainCameraNode());
		}
		
		reactionCenter = new ReactionCenter(this);
		
		steeringTask = new SteeringTask(this, driverName);
		
		threeVehiclePlatoonTask = new ThreeVehiclePlatoonTask(this, driverName);
		
		motorwayTask = new MotorwayTask(this);
		
		moviePlayer = new MoviePlayer(this);
		
		// start effect center
		effectCenter = new EffectCenter(this);
		
		objectManipulationCenter = new ObjectManipulationCenter(this);
		
		if(settingsLoader.getSetting(Setting.SettingsControllerServer_startServer, SimulationDefaults.SettingsControllerServer_startServer))
		{
			settingsControllerServer = new SettingsControllerServer(this);
			settingsControllerServer.start();
		}
		
		StatsAppState statsAppState = stateManager.getState(StatsAppState.class);
    	if (statsAppState != null && statsAppState.getFpsText() != null && statsAppState.getStatsView() != null) 
    	{
    		statsAppState.getFpsText().setLocalTranslation(3, getSettings().getHeight()-145, 0);
    		statsAppState.getStatsView().setLocalTranslation(3, getSettings().getHeight()-145, 0);
    		statsAppState.setDarkenBehind(false);
        }
    	
    	// add physics collision listener
    	CollisionListener collisionListener = new CollisionListener();
        getPhysicsSpace().addCollisionListener(collisionListener);
        
        String videoPath = settingsLoader.getSetting(Setting.General_captureVideo, "");
        if((videoPath != null) && (!videoPath.isEmpty()) && (Util.isValidFilename(videoPath)))
        {
        	System.err.println("videoPath: " + videoPath);
        	File videoFile = new File(videoPath);
        	stateManager.attach(new VideoRecorderAppState(videoFile));
        }
        
		if(settingsLoader.getSetting(Setting.Eyetracker_enableConnection, SimulationDefaults.Eyetracker_enableConnection))
		{
			eyetrackerCenter = new EyetrackerCenter(this);
		}
		
		joystickSpringController = new ForceFeedbackJoystickController(this);
		
		initializationFinished = true;
		
		// added by Yelly:
		// initialize critical waypoints

		FollowBoxSettings followBoxSettings = SimulationBasics.getDrivingTask().getScenarioLoader().getAutoPilotFollowBoxSettings();
		
		// if followBoxSettings == null, then the critical point array size equals to 0
		if(followBoxSettings!=null) {
			Iterator<Waypoint> it = followBoxSettings.getWayPoints().iterator();
			while(it.hasNext()) {
				CriticalWayPoint point = new CriticalWayPoint(); // all fields initialized to 0
				point.setPos(it.next().getPosition());
				point.setWidth(LaneWidth);
				
				// update the point's distance-to-start field
				int pointnum = criticalPoints.size();
				if(pointnum>0) {
					CriticalWayPoint prevCP = criticalPoints.get(pointnum-1);
					point.setDts(prevCP.getDts() + prevCP.getPos().distance(point.getPos()));
				}
				//else: the constructor initializer of waypoint has set distToStart to 0

				criticalPoints.add(point);
			}
			this.cp_num = this.criticalPoints.size();
		}
		
		if(this.researcher_driving) {
			try {
				this.waypoint_writer = new FileWriter("waypoints.txt");
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
    }

	
	private void initDrivingTaskLayers()
	{
		String drivingTaskFileName = SimulationDefaults.drivingTaskFileName;
		File drivingTaskFile = new File(drivingTaskFileName);
		drivingTask = new DrivingTask(this, drivingTaskFile);

		sceneLoader = drivingTask.getSceneLoader();
		scenarioLoader = drivingTask.getScenarioLoader();
		interactionLoader = drivingTask.getInteractionLoader();
		settingsLoader = drivingTask.getSettingsLoader();
	}
	
	
	/**
	 * That method is going to be executed, when the dataWriter is
	 * <code>null</code> and the S-key is pressed.
	 * 
	 * @param trackNumber 
	 *			Number of track (will be written to the log file).
	 */
	public void initializeDataWriter(int trackNumber) 
	{
		dataWriter = new DataWriter(outputFolder, car, SimulationDefaults.driverName, 
				SimulationDefaults.drivingTaskFileName, trackNumber);
	}
	

    // added by Yelly
    // for debuging and researchers generating waypoints
	int upd_cnt = 0;
    public int getUpd_cnt() {
		return upd_cnt;
	}

	@Override
    public void simpleUpdate(float tpf) 
    {
    	if(initializationFinished)
    	{
			super.simpleUpdate(tpf);
			
			// updates camera
			cameraFactory.updateCamera();
		
			if(!isPause())
				car.getTransmission().updateRPM(tpf);
		
			PanelCenter.update();
		
			triggerCenter.doTriggerChecks();
		
			updateDataWriter();
			
			// send camera data via TCP to Lightning
			if(lightningClient != null)
				lightningClient.sendCameraData(cam);
			
			// send car data via TCP to CAN-bus
			if(canClient != null)
				canClient.sendCarData();
				
			if(multiDriverClient != null)
				multiDriverClient.update();
			
			if(!isPause())
				car.update(tpf, PhysicalTraffic.getTrafficObjectList());
			
			// TODO start thread in init-method to update traffic
			physicalTraffic.update(tpf); 
			
			SpeedControlCenter.update();
			
			// update necessary even in pause
			AudioCenter.update(tpf, cam);
			
			if(!isPause())
				steeringTask.update(tpf);
			
			//if(!isPause())
				//getCameraFlight().play();
			
			threeVehiclePlatoonTask.update(tpf);
			
			motorwayTask.update(tpf);
			
			moviePlayer.update(tpf);
			
			if(cameraFlight != null)
				cameraFlight.update();
			
			reactionCenter.update();
			
			// update effects
			effectCenter.update(tpf);
			
			// forward instruction screen if available
			if(instructionScreenID != null)
			{
				instructionScreenGUI.showDialog(instructionScreenID);
				instructionScreenID = null;
			}
			
			if(eyetrackerCenter != null)
				eyetrackerCenter.update();

    		if(frameCounter == 5)
    		{
    			if(settingsLoader.getSetting(Setting.General_pauseAfterStartup, SimulationDefaults.General_pauseAfterStartup))
    				setPause(true);
    		}
    		frameCounter++;
    		
    		joystickSpringController.update(tpf);
    		
    		updateCoordinateSystem();
    		
    		this.upd_cnt++;
    	}
    }

    
	private void updateCoordinateSystem()
	{
		getCoordinateSystem().getChild("x-cone").setLocalTranslation(car.getPosition().getX(), 0, 0);
		getCoordinateSystem().getChild("y-cone").setLocalTranslation(0, car.getPosition().getY(), 0);
		getCoordinateSystem().getChild("z-cone").setLocalTranslation(0, 0, car.getPosition().getZ());
	}
	

	private void updateDataWriter() 
	{
		if (dataWriter != null && dataWriter.isDataWriterEnabled()) 
		{
			if(!isPause())
				dataWriter.saveAnalyzerData();

			if (!dataWriterQuittable)
				dataWriterQuittable = true;
		} 
		else 
		{
			if (dataWriterQuittable) 
			{
				dataWriter.quit();
				dataWriter = null;
				dataWriterQuittable = false;
			}
		}
	}
	
	
	/**
	 * Cleanup after game loop was left.
	 * Will be called when pressing any close-button.
	 * destroy() will be called subsequently.
	 */
	/*
	@Override
    public void stop()
    {
		logger.info("started stop()");		
		super.stop();
		logger.info("finished stop()");
    }
	*/
	
	
	/**
	 * Cleanup after game loop was left
	 * Will be called whenever application is closed.
	 */
	
	@Override
	public void destroy()
    {
		logger.info("started destroy()");

		if(initializationFinished)
		{
			if(lightningClient != null)
				lightningClient.close();
			
			if(canClient != null)
				canClient.requestStop();
				
			if(multiDriverClient != null)
				multiDriverClient.close();
			
			trafficLightCenter.close();
			
			steeringTask.close();
			
			threeVehiclePlatoonTask.close();
			
			moviePlayer.stop();
			
			reactionCenter.close();
			
			HMICenter.close();
			
			KnowledgeBase.KB.disconnect();
			
			car.close();
			
			physicalTraffic.close();
			
			if(settingsControllerServer != null)
				settingsControllerServer.close();
			
			if(eyetrackerCenter != null)
				eyetrackerCenter.close();
			
			joystickSpringController.close();
			//initDrivingTaskSelectionGUI();
		}

		super.destroy();
		logger.info("finished destroy()");
		
		if (physicalTraffic.getMultiThreadingEnable()){
			physicalTraffic.executorShutdown();
		}
		
		PlatformImpl.exit();
		//System.exit(0);
    }
	

    public static void main(String[] args) 
    {    
    	
    	/*// License library, list of codes returned from the function
    	String licenseCode0 = "ERROR_SUCCESS";
		String licenseCode234 = "ERROR_MODE_DATA";
		String licenseCode13 = "ERROR_INVALID_DATA";
		String licenseCode5 = "ERROR_ACCESS_DENIED";
		
		
		String daysLeft;
		// License library, list of properties to extract
		String getTrialName = "TrialName";
		
		// License implementation might be extended to include following features
		String getLicReqContact = "LicReqContact";
		String getbuildDate = "BuildDate";
		String getBuyUrl = "BuyUrl";
		String getTrialExtendContract = "TrialExtendContract";
		String getLicenseKey = "LicenseKey";
		String getTrialLeft = "TrialLeft";
		String getQuantity = "Quantity";
		String getTrialMU = "TrialMU";
		String getCompId = "CompId";
		
		
		
		// proceed with the loop if license is OK, other ways skip the loop
		try {
	          String value = GetPropertyValue(getTrialName); 
	          if ( value == licenseCode0 ){
	        	  System.out.println(licenseCode0);
	          }
	          else if ( value == licenseCode5){
	        	  System.out.println(licenseCode5);
	        	  System.exit(0);
	          }
	          else if ( value == licenseCode234 ){
	        	  System.out.println(licenseCode234);
	        	  System.exit(0);
	          }
	          else if ( value == licenseCode13 ){
	        	  System.out.println(licenseCode13);
	        	  System.exit(0);
	          }
	     } catch (Exception e) {
	         e.printStackTrace();
	         logger.fatal("Could not initialize the license", e);
	     }
	    */
    	
    	try
    	{
    		// copy native files of force feedback joystick driver
    		boolean isWindows = System.getProperty("os.name").toLowerCase().indexOf("win") >= 0;
    		if(isWindows)
    		{
    			boolean is64Bit = System.getProperty("sun.arch.data.model").equalsIgnoreCase("64");
    			if(is64Bit)
    			{
    				copyFile("lib/ffjoystick/native/win64/ffjoystick.dll", "ffjoystick.dll");
    				copyFile("lib/ffjoystick/native/win64/SDL.dll", "SDL.dll");
    			}
    			else
    			{
    				copyFile("lib/ffjoystick/native/win32/ffjoystick.dll", "ffjoystick.dll");
    				copyFile("lib/ffjoystick/native/win32/SDL.dll", "SDL.dll");
    			}
    		}
    	    
    		 
    		
    		// load logger configuration file
    		PropertyConfigurator.configure("assets/JasperReports/log4j/log4j.properties");
    		
    		/*
    		logger.debug("Sample debug message");
    		logger.info("Sample info message");
    		logger.warn("Sample warn message");
    		logger.error("Sample error message");
    		logger.fatal("Sample fatal message");
    		*/
    		
    		oculusRiftAttached = OculusRift.initialize();
    		
    		// only show severe jme3-logs
    		java.util.logging.Logger.getLogger("").setLevel(java.util.logging.Level.SEVERE);
    		
    		PlatformImpl.startup(() -> {});
    		
	    	Simulator sim = new Simulator();
    		
	    	StartPropertiesReader startPropertiesReader = new StartPropertiesReader();

			sim.setSettings(startPropertiesReader.getSettings(sim));

			// show/hide settings screen
			sim.setShowSettings(startPropertiesReader.showSettingsScreen());
			
			if(!startPropertiesReader.getDrivingTaskPath().isEmpty() &&
					DrivingTask.isValidDrivingTask(new File(startPropertiesReader.getDrivingTaskPath())))
    		{
    			SimulationDefaults.drivingTaskFileName = startPropertiesReader.getDrivingTaskPath();
    			sim.drivingTaskGiven = true;
    		}
			
			if(!startPropertiesReader.getDriverName().isEmpty())
				SimulationDefaults.driverName = startPropertiesReader.getDriverName();
			
			
	    	if(args.length >= 1)
	    	{
	    		if(DrivingTask.isValidDrivingTask(new File(args[0])))
	    		{
	    			SimulationDefaults.drivingTaskFileName = args[0];
	    			sim.drivingTaskGiven = true;
	    		}
	    	}
	
	    	if(args.length >= 2)
	    	{
	    		SimulationDefaults.driverName = args[1];
	    	}
	    	
			
	    	sim.setPauseOnLostFocus(false);
	    	
			sim.start();
    	}
    	catch(Exception e1)
    	{
    		logger.fatal("Could not run main method:", e1);
    	}
    }
    
	private static void copyFile(String sourceString, String targetString) 
	{
		try {
			
			Path source = Paths.get(sourceString);
			Path target = Paths.get(targetString);
			
			if(Files.exists(source, LinkOption.NOFOLLOW_LINKS))
				Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
			else
				System.err.println("ERROR: '" + sourceString + "' does not exist.");
		
		} catch (IOException e) {

			e.printStackTrace();
		}
	}
	public void traff_num_init(Simulator sim) {
		Node guiNode = sim.getGuiNode();
		BitmapFont guiFont = sim.getAssetManager().loadFont("Interface/Fonts/Default.fnt");
		testxt = new BitmapText(guiFont, false);
		testxt.setName("currentGearText");
		testxt.setSize(guiFont.getCharSet().getRenderedSize());
		testxt.setColor(ColorRGBA.Green);
		testxt.setLocalTranslation(1077, 668, 0);
        guiNode.attachChild(testxt);
		
	}
	
	public BitmapText get_Bitmap () {
		return testxt;
	}
}
