package algorithms.Astar.SMAStar;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import algorithms.Astar.AStarState;
import algorithms.trees.State;
import geometry.coords.AgentPosition;

public class SMAStarState extends AStarState {

	private HashMap<SMAStarState, Double> _forgottenTable = new HashMap<SMAStarState, Double>();

	public SMAStarState(AgentPosition position) {
		super(position);
	}
	
	public SMAStarState(double gVal, double hVal, double fVal, AgentPosition position) {
		super(gVal, hVal, fVal, position);
	}

	public HashMap<SMAStarState, Double> getForgottenTable() {
		return _forgottenTable;
	}
	
	public List<State> getForgottenNodes(){
		Set<SMAStarState> set = _forgottenTable.keySet();
		List<State> res = new ArrayList<State>(set);
		return res;
	}
	
	//add expand methods to the subclasses of this class

}
