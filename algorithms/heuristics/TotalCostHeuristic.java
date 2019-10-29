package algorithms.heuristics;

import algorithms.CBS.CBSState;
import algorithms.trees.State;

public class TotalCostHeuristic implements HeuristicInterface {

	@Override
	public float calcHeuristic(State current, State goal) {
		return calcHeuristic(current);
	}
	
	public float calcHeuristic(State current) {
		return ((CBSState)current).getTotalCost();
	}

}
