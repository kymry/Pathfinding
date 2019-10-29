package algorithms.trees;

import algorithms.heuristics.HeuristicInterface;

public abstract class State implements Comparable<State>{

	protected double _fVal;
	protected double _gVal;
	protected double _hVal;
	protected HeuristicInterface _heuristic;
	
	public State() {
		_gVal = 0;
		_hVal = 0;
		_fVal = 0;		
	}
	
	public State(double gVal, double hVal, double fVal) {
		_gVal = gVal;
		_hVal = hVal;
		_fVal = fVal;
	}
	
	/**
	 * @param map
	 * @return a list of ALL possible neighbors. Meaning all verifications for validity to move from this node to the neighbors should be done here
	 */
//	public abstract List<State> expand(Map map, double agentRadius, double agentSpeed) ;
//	public abstract List<State> expand(Map map, HeuristicInterface heuristic) ;
	
	@Override
	public abstract int compareTo(State o);

	@Override
	public String toString() {
		return "F value: " + _fVal;
	}
	
	public abstract State clone();
	public abstract double calculateDistance(State state);
	public abstract double getLowerBound();
	
	public double getFVal() {
		return _fVal;
	}
	
	public double getGVal() {
		return _gVal;
	}

	public double getHVal() {
		return _hVal;
	}
	
	public void setFVal(double fVal) {
		_fVal = fVal;
	}
	
	public void setGVal(double gVal) {
		_gVal = gVal;
	}

	public void setHVal(double hVal) {
		_hVal = hVal;
		//update fVal;
		_fVal = _gVal + _hVal;
	}
}
