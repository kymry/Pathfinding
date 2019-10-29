package algorithms.Astar.SMAStar;

import geometry.coords.AgentPosition;
import geometry.coords.PointInterface;

public class SMAStarStateFixedWings extends SMAStarState {
	protected PointInterface _coordinatesReal;
	
	public SMAStarStateFixedWings(AgentPosition position, PointInterface coordinatesReal) {
		super(position);
		_coordinatesReal = coordinatesReal;
	}
	
	public SMAStarStateFixedWings(double gVal, double hVal, double fVal, AgentPosition position, PointInterface coordinatesReal) {
		super(gVal, hVal, fVal, position);
		_coordinatesReal = coordinatesReal;
	}
	
	//add expand methods to the subclasses of this class

}
