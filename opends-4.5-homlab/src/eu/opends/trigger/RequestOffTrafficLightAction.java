package eu.opends.trigger;

import eu.opends.environment.TrafficLightCenter.TriggerType;
import eu.opends.main.Simulator;

/**
 * 
 * @author Rafael Math
 */
public class RequestOffTrafficLightAction extends TriggerAction 
{
	private Simulator sim;
	private String trafficLightName;
	
	
	public RequestOffTrafficLightAction(float delay, int maxRepeat, Simulator sim, String trafficLightID)
	{
		super(delay, maxRepeat);
		
		this.sim = sim;
		this.trafficLightName = trafficLightID;
	}
	
	
	@Override
	protected void execute() 
	{
		if(!isExceeded())
		{
			sim.getTrafficLightCenter().reportCollisionOff(trafficLightName, TriggerType.REQUEST);
				
			updateCounter();
		}
	}

}
