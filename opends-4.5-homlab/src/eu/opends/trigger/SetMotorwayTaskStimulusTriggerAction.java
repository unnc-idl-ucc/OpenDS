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

package eu.opends.trigger;

import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Node;

import eu.opends.environment.TrafficLightCenter.TrafficLightMode;
import eu.opends.main.Simulator;

/**
 * 
 * @author Rafael Math
 */
public class SetMotorwayTaskStimulusTriggerAction extends TriggerAction
{
	private Simulator sim;
	private String stimulusID;
	
	
	public SetMotorwayTaskStimulusTriggerAction(float delay, int maxRepeat, Simulator sim,	String stimulusID) 
	{
		super(delay, maxRepeat);
		this.sim = sim;
		this.stimulusID = stimulusID;
		if(maxRepeat == 0) {
			sim.traff_num_init(sim);
			maxRepeat =1;
		}
			
	}
	

	@Override
	protected void execute()
	{
		if(!isExceeded())
		{
			    int r = Integer.parseInt(stimulusID);
			    if(r == 0) {
			    	sim.get_Bitmap().setText("");
			    }
			    else {
			    	sim.get_Bitmap().setText(stimulusID);
			    }
				
		}
	}
	
	


}