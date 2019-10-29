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
import algorithms.CBS.CBSSearch;
import algorithms.CBS.CBSState;
import algorithms.comparators.FValueComparator;
import algorithms.comparators.HValueComparator;
import algorithms.conflicts.AgentConflict;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import exceptions.SolutionNotFoundException;
import map.Map;

public class ECBSSearch extends CBSSearch {	
	private double _weight;

	public ECBSSearch(Map map, List<Agent> agents, Node root, HeuristicInterface heuristic, AlgorithmName singleSearch, double weight) {
		super(map, agents, root, heuristic, singleSearch);
		_focalList = new PriorityQueue<Node>(new HValueComparator());
		_openComparator = new FValueComparator();
		_weight = weight;
	}
	
	@Override
	public State getNewState(float cost, HashMap<Integer, HashSet<PathLine>> agentObstacles, PriorityQueue<AgentConflict> conflictsStart) {
		ECBSState newState = new ECBSState(-1, Double.MAX_VALUE, -1, cost, agentObstacles, _singleSearch, conflictsStart, null, _heuristic, _weight);
		return newState;
	}
	
	@Override
	public ArrayList<Path> getInitialPaths(State start) {
		ArrayList<Path> allSingleAgentPaths = new ArrayList<Path>();
		for (int i = 0; i < _numberOfAgents; i++) {
			allSingleAgentPaths.add(_agents.get(i).getPath().clone());
			double cost = _agents.get(i).getPath().getEndTimestep() - _agents.get(i).getPath().getStartTimestep();
			_initialCost += cost;
			((ECBSState)start).addToFMinAgents(cost);
		}
		((CBSState)start).setTotalCost(_initialCost);
		return allSingleAgentPaths;
	}
	
	@Override
	public void search() throws SolutionNotFoundException{
		searchFocal(_weight);
	}
	
	@Override
	public Node getNextBestNode() {
		Node bestNode = _focalList.poll();
		_openList.remove(bestNode);
		return bestNode;
	}
	
	@Override
	public void addChildNode(Node parent, CBSState state) {
		Node childNode = new Node(parent, state);
		_openList.add(childNode);
		if(((ECBSState)childNode.getState()).getLowerBound() <= _weight * _lowerBound) {
			_focalList.add(childNode);
		}
	}
	
	@Override
	public void createStartState() {
		super.createStartState();
		((ECBSState)_root.getState()).computeFMinAgents();
		((ECBSState)_root.getState()).computeLowerBound();
	}
	
	@Override
	public String getAlgorithmName() {
		return "ECBS";
	}

	public double getWeight() {
		return _weight;
	}

}
