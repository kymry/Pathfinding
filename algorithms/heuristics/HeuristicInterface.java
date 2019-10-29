package algorithms.heuristics;

import algorithms.trees.State;

public interface HeuristicInterface {
	public float calcHeuristic(State current, State goal);
}
