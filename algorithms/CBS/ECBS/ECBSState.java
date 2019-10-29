package algorithms.CBS.ECBS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import agents.Agent;
import agents.path.Path;
import agents.path.PathLine;
import algorithms.AlgorithmName;
import algorithms.CBS.CBSState;
import algorithms.conflicts.AgentConflict;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import exceptions.SolutionNotFoundException;
import exceptions.TimeoutException;
import map.Map;
import utils.AgentUtils;
import utils.AlgorithmUtils;

public class ECBSState extends CBSState {
	
	private List<Double> _fMinAgents;
	private double _lowerBound;
	private double _weight;

	public ECBSState(double gVal, double hVal, double fVal, float totalCost,
			HashMap<Integer, HashSet<PathLine>> agentObstacles, AlgorithmName singleSearch,
			PriorityQueue<AgentConflict> conflicts, Agent agent, HeuristicInterface heuristic, double weight) {
		super(gVal, hVal, fVal, totalCost, agentObstacles, singleSearch, conflicts, agent, heuristic);
		_fMinAgents = new ArrayList<Double>();
		_lowerBound = Double.MAX_VALUE;
		_weight = weight;
	}
	
	@Override
	public CBSState getNewState(HashMap<Integer, HashSet<PathLine>> agentObstacles, PriorityQueue<AgentConflict> conflicts, Agent agent) {
		ECBSState state = new ECBSState(_gVal, Double.MAX_VALUE, _fVal, Float.MAX_VALUE, agentObstacles, _singleSearch, conflicts, agent, _heuristic, _weight);
		state.setAllSingleAgentPaths(AgentUtils.clonePaths(_allSingleAgentPaths));	
		state.computeLowerBound();
		return state;
	}
	
	@Override
	public void otherUpdateds() {
		computeFMinAgents();
		computeLowerBound();
	}
	
	@Override
	public void runAlgorithm(Map map, List<Agent> agents, Node root, long timelimit, long initialTime) throws SolutionNotFoundException, TimeoutException{
		AlgorithmUtils.runAlgorithm(map, agents, root, _singleSearch, _singleSearch, timelimit, initialTime, _weight, null);
	}
	
	@Override
	public void setAllSingleAgentPaths(ArrayList<Path> allSingleAgentPaths) {
		super.setAllSingleAgentPaths(allSingleAgentPaths);
		computeFMinAgents();
	}

	public List<Double> getFMinAgents() {
		return _fMinAgents;
	}
	
	public void addToFMinAgents(double fMin) {
		_fMinAgents.add(fMin);
	}

	public void setFMinAgents(List<Double> fMinAgents) {
		this._fMinAgents = fMinAgents;
	}
	
	public double computeLowerBound() {
		_lowerBound = 0;
		for(Double fMin : _fMinAgents) {
			_lowerBound += fMin;
		}
		return _lowerBound;
	}

	public double getLowerBound() {
		//check if lower bound was already computed and compute if not
		if(_lowerBound == Double.MAX_VALUE) {
			computeLowerBound();
		}
		_fVal = _lowerBound;
		return _lowerBound;
	}
	
	public void computeFMinAgents() {
		_fMinAgents.clear();
		for(Path path : _allSingleAgentPaths) {
			_fMinAgents.add(path.getCost());
		}
	}

}
