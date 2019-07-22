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

package eu.opends.qn;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import eu.opends.tools.Util;

/**
 * This class is used for logging qn results 
 * 
 * @author Yelly
 */
public class QNLogger 
{
	private String outputFolder;
	private File outFile;
	private BufferedWriter output;
	private String newLine = System.getProperty("line.separator");


	/**
	 * Creates a new driving task logger and initializes it. A file with the given
	 * parameters driver name, driving task file name and type of progress bar will
	 * be written.
	 * 
	 * @param outputFolder
	 * 			Indicates the folder the log file will be written to. 
	 * 
	 * @param driverName
	 * 			Name of the current driver.
	 * 
	 * @param drivingTask
	 * 			Name of the driving task file.
	 */
	public QNLogger(String outputFolder) {
		this.outputFolder = outputFolder;

		Util.makeDirectory(outputFolder);
		initWriter();
	}
	
	
	/**
	 * Adds a string with time stamp to the output file.
	 * 
	 * @param string
	 * 			String to add.
	 * 
	 * @param timestamp
	 * 			Time stamp for output line.
	 */
	public void reportText(String string, Date timestamp) 
	{
		// format current time stamp
		String timestampString = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(timestamp);

		// write data to file
		try {
			output.write(timestampString + " --> " + string + newLine);
			output.flush();
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	
	/**
	 * Adds a string to the output file.
	 * 
	 * @param string
	 * 			String to add.
	 */
	public void reportText(String string) 
	{
		// write data to file
		try {
			output.write(string + newLine);
			output.flush();
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}	
	
	/**
	 * Closes the file when the simulator has been halted.
	 */
	public void quit() 
	{
		try {
			
			if (output != null)
				output.close();
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}


	/**
	 * Initializes the log file by setting up its file name and some further 
	 * information like: driver's name, name of driving task file, type of 
	 * progress bar and start time.
	 */
	private void initWriter() 
	{
		// create a valid, nonexistent file name
		//String fileName = createFileName();
		File analyzerDataFile = new File(outputFolder + "/qnLogger.txt");
		
		if (analyzerDataFile.getAbsolutePath() == null) 
		{
			System.err.println("Parameter not accepted at method initWriter.");
			return;
		}
		
		outFile = new File(analyzerDataFile.getAbsolutePath());
		
		// write date to file
		try {
			
			String timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(new Date());
			
			output = new BufferedWriter(new FileWriter(outFile));
			output.write("Start Time: "	+ timestamp + newLine + newLine);
			output.flush();

		} catch (IOException e) {

			e.printStackTrace();
		}
	}

}
