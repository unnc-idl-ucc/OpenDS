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

package eu.opends.analyzer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URI;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;

import eu.opends.car.Car;
import eu.opends.car.SteeringCar;
import eu.opends.main.Simulator;
import eu.opends.oculusRift.OculusRift;
import eu.opends.tools.Util;
//////////////
//import eu.opends.codriver.ScenarioMessage;
import eu.opends.main.Simulator;

/**
 * 
 * That class is responsible for writing drive-data. At the moment it is a
 * ripped down version of similar classes used in CARS.
 * 
 * @author Saied
 * 
 */
public class DataWriter 
{
	private Calendar startTime = new GregorianCalendar();

	/**
	 * An array list for not having to write every row directly to file.
	 */
	private ArrayList<DataUnit> arrayDataList;
	private BufferedWriter out;
	private File outFile;
	private String newLine = System.getProperty("line.separator");
	private long lastAnalyzerDataSave;
	private long lastRecordTime; ///////////////new added
	private float lastSpeed = 0;
	private float lastX;
	private float lastY;
	private float lastZ;
	private Car car;
	private File analyzerDataFile;
	private boolean dataWriterEnabled = false;
	private String relativeDrivingTaskPath;

private Simulator sim;




	public DataWriter(String outputFolder, Car car, String driverName, String absoluteDrivingTaskPath, int trackNumber) 
	{
		this.car = car;
		this.relativeDrivingTaskPath = getRelativePath(absoluteDrivingTaskPath);
		
		Util.makeDirectory(outputFolder);

		if(trackNumber >= 0)
			analyzerDataFile = new File(outputFolder + "/carData_track" + trackNumber + ".txt");
		else
			analyzerDataFile = new File(outputFolder + "/carData.txt");

		
		if (analyzerDataFile.getAbsolutePath() == null) 
		{
			System.err.println("Parameter not accepted at method initWriter.");
			return;
		}
		
		outFile = new File(analyzerDataFile.getAbsolutePath());
		
		int i = 2;
		while(outFile.exists()) 
		{
			if(trackNumber >= 0)
				analyzerDataFile = new File(outputFolder + "/carData_track" + trackNumber + "(" + i + ").txt");
			else
				analyzerDataFile = new File(outputFolder + "/carData(" + i + ").txt");
			
			outFile = new File(analyzerDataFile.getAbsolutePath());
			i++;
		}
		
	String occulusHeader = "";
		if(Simulator.oculusRiftAttached)
			occulusHeader = " : OculusRift Horizontal Angle (- = left, + = right): OculusRift Vertical Angle (- = down, + = up)";
	
		
		try {
			out = new BufferedWriter(new FileWriter(outFile));
			out.write("Driving Task: " + relativeDrivingTaskPath + newLine);
			out.write("Date-Time: "
					+ new SimpleDateFormat("yyyy_MM_dd-HH_mm_ss")
							.format(new Date()) + newLine);
			out.write("Driver: " + driverName + newLine);
			out.write("Used Format = Time (ms): Position (x,y,z) : Rotation (x,y,z,w) :"
					+ " Speed (km/h) : Steering Wheel Position [-1,1] : Gas Pedal Position :"
					+ " Brake Pedal Position : Engine Running : Distance Ahead (meters): Time To Collision (sec) : Headway Time (sec)" +occulusHeader  + newLine);

		} catch (IOException e) {
			e.printStackTrace();
		}
		arrayDataList = new ArrayList<DataUnit>();
		lastAnalyzerDataSave = (new Date()).getTime();
		lastRecordTime = lastAnalyzerDataSave;//////////////////new added
	}
	
	
	private String getRelativePath(String absolutePath)
	{
		URI baseURI = new File("./").toURI();
		URI absoluteURI = new File(absolutePath).toURI();
		URI relativeURI = baseURI.relativize(absoluteURI);
		
		return relativeURI.getPath();
	}


	/**
	 * Save the car data at a frequency of 20Hz. That class should be called in
	 * the update-method <code>Simulator.java</code>.
	 */
	public void saveAnalyzerData() 
	{
		int updateInterval = 50; // = 1000/20
		
		Date curDate = new Date();

		if (curDate.getTime() - lastAnalyzerDataSave/*.getTime()*/ >= 2*updateInterval) 
		{
			lastAnalyzerDataSave = curDate.getTime() - 2*updateInterval;
		}
		
		
		if (curDate.getTime() - lastAnalyzerDataSave/*.getTime()*/ >= updateInterval) 
		{
			float currSpeed = car.getCurrentSpeedKmhRounded();
			float x=(Math.round(car.getPosition().x * 1000) / 1000.0f)-lastX, y=(Math.round(car.getPosition().y * 1000) / 1000.0f)-lastY, z = (Math.round(car.getPosition().z * 1000) / 1000.0f)-lastZ;
			
			//System.err.println("diff: " + (curDate.getTime() - lastAnalyzerDataSave));
			write(
					curDate,
					Math.round(car.getPosition().x * 1000) / 1000.0f,
					Math.round(car.getPosition().y * 1000) / 1000.0f,
					Math.round(car.getPosition().z * 1000) / 1000.0f,
					Math.round(car.getRotation().getX() * 10000) / 10000.0f,
					Math.round(car.getRotation().getY() * 10000) / 10000.0f,
					Math.round(car.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(car.getRotation().getW() * 10000) / 10000.0f,
					car.getCurrentSpeedKmhRounded(), 
					Math.round(car.getSteeringWheelState() * 100000) / 100000.0f, 
					car.getAcceleratorPedalIntensity(), 
					car.getBrakePedalIntensity(), 
					car.isEngineOn(), 
					0,
					0,
					0,
					//car.getAcceleration(),
					//curDate.getTime() - lastRecordTime,
					((((currSpeed - lastSpeed)/3.6f)/(curDate.getTime() - lastRecordTime))*1000)/(float)Math.sqrt(x*x+y*y+z*z)*Math.abs(x), 
					OculusRift.getOrientation().clone());
			//System.out.println("distancetoHead and speed: " + ((SteeringCar)car).getDistanceToCar() + " " + car.getCurrentSpeedKmhRounded());
			

			//lastAnalyzerDataSave = curDate;         //(((currSpeed - lastSpeed)/3.6f)/(curDate.getTime() - lastRecordTime))*1000 is the acceleration
			lastAnalyzerDataSave += updateInterval;
			lastRecordTime = curDate.getTime();
			lastSpeed = car.getCurrentSpeedKmhRounded(); 
			lastX = Math.round(car.getPosition().x * 1000) / 1000.0f;
			lastY = Math.round(car.getPosition().y * 1000) / 1000.0f;
			lastZ = Math.round(car.getPosition().z * 1000) / 1000.0f;
		}

	}

	
	// see eu.opends.analyzer.IAnalyzationDataWriter#write(float,
	//      float, float, float, java.util.Date, float, float, boolean, float)
	public void write(Date curDate, float x, float y, float z, float xRot,
			float yRot, float zRot, float wRot, float linearSpeed,
			float steeringWheelState, float gasPedalState, float brakePedalState,
			boolean isEngineOn, float distanceAhead, float timeToCollide, float headWayTime, float acceleration, Quaternion OculusOrientation) 
	{
		DataUnit row = new DataUnit(curDate, x, y, z, xRot, yRot, zRot, wRot,
				linearSpeed, steeringWheelState, gasPedalState, brakePedalState,
				isEngineOn, distanceAhead, timeToCollide, headWayTime, acceleration, OculusOrientation);
		this.write(row);

	}
	

	/**
	 * Write data to the data pool. After 50 data sets, the pool is flushed to
	 * the file.
	 * 
	 * @param row
	 * 			Datarow to write
	 */
	public void write(DataUnit row)
	{
		arrayDataList.add(row);
		if (arrayDataList.size() > 50)
			flush();
	}
	

	public void flush() 
	{
		try {
			StringBuffer sb = new StringBuffer();
			for (DataUnit r : arrayDataList) {

				String occulusData = "";
				if(Simulator.oculusRiftAttached)
				{
					float[] angles = r.getOculusRiftOrientation().toAngles(null);
					float verticalRot = Math.round(-angles[0]*FastMath.RAD_TO_DEG*100)/100f;
					float horizontalRot = Math.round(-angles[1]*FastMath.RAD_TO_DEG*100)/100f;
					occulusData = ":" + horizontalRot + ":" + verticalRot;
				}

				sb.append(r.getDate().getTime() + ":" + r.getXpos() + ":"
						+ r.getYpos() + ":" + r.getZpos() + ":" + r.getXrot()
						+ ":" + r.getYrot() + ":" + r.getZrot() + ":"
						+ r.getWrot() + ":" + r.getSpeed() + ":"
						+ r.getSteeringWheelPos() + ":" + r.getAcceleratorPedalPos() + ":"
						+ r.getBrakePedalPos() + ":" + r.isEngineOn() + ":" + r.getDistanceAhead() + ":" + r.getTimeToCollide() + ":" + r.getHeadWay() + ":" + r.getAcceleration()  + occulusData + newLine
						);
				//System.out.println("acceleration: " + r.getSpeed());
				//System.out.println("acceleration: " + car.getAcceleration());
				//System.out.println("acceleration vector is: " +  ((SteeringCar)car).getDistanceToCar());
			}
			out.write(sb.toString());
			arrayDataList.clear();
			out.flush();
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(0);
		}
	}

	
	public void quit() 
	{
		dataWriterEnabled = false;
		flush();
		try {
			if (out != null)
				out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	

	public boolean isDataWriterEnabled() 
	{
		return dataWriterEnabled;
	}

	
	public void setDataWriterEnabled(boolean dataWriterEnabled) 
	{
		this.dataWriterEnabled = dataWriterEnabled;
	}

	
	public void setStartTime() 
	{
		this.startTime = new GregorianCalendar();
	}
	
	
	public String getElapsedTime()
	{
		Calendar now = new GregorianCalendar();
		
		long milliseconds1 = startTime.getTimeInMillis();
	    long milliseconds2 = now.getTimeInMillis();
	    
	    long elapsedMilliseconds = milliseconds2 - milliseconds1;
	    
	    return "Time elapsed: " + new SimpleDateFormat("mm:ss.SSS").format(elapsedMilliseconds);
	}

}
