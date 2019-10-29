package algorithms.CBS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;

import agents.Agent;
import agents.path.Path;
import agents.path.PathLine;
import algorithms.AlgorithmName;
import algorithms.conflicts.AgentConflict;
import algorithms.conflicts.AlgorithmConflict;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import exceptions.SolutionNotFoundException;
import exceptions.TimeoutException;
import main.ConflictDetection;
import map.Map;
import utils.AgentUtils;
import utils.AlgorithmUtils;
import utils.Debug;
import utils.DebugEnums;
import utils.GeneralUtils;
import utils.MapUtils;
import utils.Utils;

public class CBSState extends State{
	
	protected Agent _agent; //agent that was modified - DO NOT MODIFY THE PATH DIRECTLY
	protected float _totalCost;
	protected List<Path> _allSingleAgentPaths;
	protected AgentConflict _earliestConflict;
	protected PriorityQueue<AgentConflict> _conflicts;
	protected HashMap<Integer, HashSet<PathLine>> _agentObstacles;
	protected AlgorithmName _singleSearch;

	public CBSState(double gVal, double hVal, double fVal, float totalCost, HashMap<Integer, HashSet<PathLine>> agentObstacles,
					AlgorithmName singleSearch, PriorityQueue<AgentConflict> conflicts, Agent agent, HeuristicInterface heuristic) {
		super(gVal, hVal, fVal);
		_allSingleAgentPaths = new ArrayList<Path>();
		_totalCost = totalCost;
		_agentObstacles = agentObstacles;
		_singleSearch = singleSearch;
		_conflicts = conflicts;
		_agent = agent;
		_heuristic = heuristic;
	}
	
	public List<State> expand(Map map, long timeLimit, long initialTime) {
		List<State> children = new ArrayList<State>();
		//create both possible states
		for(int i = 1; i <= 2; i++) {
			long timelimit = timeLimit - (GeneralUtils.getCurrentTimeInMilliseconds() - initialTime);
			State child = createChildState(i, map.clone(), timelimit, initialTime);
			//If a child is null then there was no solution found from the single agent search
			if(child != null) {
				children.add(child);
			}			
		}
		//return states
		return children;
	}
	
	@Override
	public State clone() {
		PriorityQueue<AgentConflict> conflicts = cloneConflicts();
		CBSState state = new CBSState(_gVal, _hVal, _fVal, _totalCost, MapUtils.cloneAgentObstacles(_agentObstacles), _singleSearch, conflicts, _agent, _heuristic);
		state._allSingleAgentPaths = _allSingleAgentPaths;
		state._totalCost = _totalCost;
		state._earliestConflict = _earliestConflict;
		return state;
	}
	
	@Override
	public double calculateDistance(State state) {
		return Math.abs(((CBSState)state).getNumberOfConflicts() - getNumberOfConflicts());
	}

	/**
	 * Hard coded to return state with fewest conflicts
	 */
	@Override
	public int compareTo(State state) {
		CBSState cbsState = (CBSState) state;
		if(getNumberOfConflicts() < cbsState.getNumberOfConflicts()) {
			return -1;
		}
		else if(getNumberOfConflicts() > cbsState.getNumberOfConflicts()) {
			return 1;
		}
		return 0;
	}
	
	@Override
	public String toString() {
		return "Total cost: " + _totalCost + "; All paths: " + _allSingleAgentPaths.toString();
	}
	
	@Override
	public double getLowerBound() {
		return -1;
	}
	
	public PriorityQueue<AgentConflict> cloneConflicts(){
		PriorityQueue<AgentConflict> conflicts = new PriorityQueue<AgentConflict>(_conflicts.comparator());
		conflicts.addAll(_conflicts);
		return conflicts;
	}
	
	/**
	 * Checks all conflicts at the given state
	 */
	public void findConflict(Agent agentI, int index) {
		// for each pair of current agent and the other agents paths that belong to the same path time interval
		for(Agent agentJ : agentI.getAgentsInSharedTime()) {
			//avoid repetition
			if(agentJ.getIndex() >= index) {
				Path pathJ = agentJ.getPath();
				// check for conflicts
				List<AlgorithmConflict> conflicts = new ArrayList<AlgorithmConflict>();
				ConflictDetection.checkPathConflict(_allSingleAgentPaths.get(agentI.getIndex()), pathJ, null, null, false, conflicts, true);
				// if there are no conflicts, continue
				if (conflicts.isEmpty()) {
					continue;
				}
				// add the pairs earliest conflicts to the conflicts priority queue
				_conflicts.add((AgentConflict) conflicts.get(0));
			}
		}		
	}
	
	public void findAgentConflict() {
		//remove conflicts involving the agent	
		Utils.removeAgentFromConflicts(_conflicts, _agent.getId());
		findConflict(_agent, 0);
		//update heuristic value;
		updateHeuristicValue();
		_earliestConflict = _conflicts.poll();
		
	}
	
	public void findAllConflicts() {
		for(int i = 0; i < _allSingleAgentPaths.size(); i++) {
			Path pathI = _allSingleAgentPaths.get(i);
			findConflict(pathI.getAgent(), i+1);
		}
		//update heuristic value;
		updateHeuristicValue();
		//get the earliest conflict between all agents
		_earliestConflict = _conflicts.poll();
	}
	
	public void updateHeuristicValue() {
		_hVal = _heuristic.calcHeuristic(this, null);
	}
	
	public CBSState createChildState(int id, Map map, long timelimit, long initialTime) {		
		HashMap<Integer, HashSet<PathLine>> agentObstacles = MapUtils.cloneAgentObstacles(_agentObstacles);
		//add location of the other agent to the agent obstacles
		int conflictTimestep = (int)Math.floor(_earliestConflict.getTimestep());
		Agent agent = null;
		switch (id) {
		case 1:
			if(agentObstacles.containsKey(conflictTimestep)) {
				agentObstacles.get(conflictTimestep).add(_earliestConflict.getLocation2());
			}
			else {
				HashSet<PathLine> newPathLines = new HashSet<PathLine>();
				newPathLines.add(_earliestConflict.getLocation2());
				agentObstacles.put(conflictTimestep, newPathLines);				
			}
			map.addAgentObstacle(conflictTimestep, _earliestConflict.getLocation2());
			agent = _earliestConflict.getAgent1().clone();
			break;
		case 2:
			if(agentObstacles.containsKey(conflictTimestep)) {
				agentObstacles.get(conflictTimestep).add(_earliestConflict.getLocation1());
			}
			else {
				HashSet<PathLine> newPathLines = new HashSet<PathLine>();
				newPathLines.add(_earliestConflict.getLocation1());
				agentObstacles.put(conflictTimestep, newPathLines);
			}
			map.addAgentObstacle(conflictTimestep, _earliestConflict.getLocation1());
			agent = _earliestConflict.getAgent2().clone();
			break;
		default:
			Debug.print(DebugEnums.ERROR, "Wrong State Id: must be 1 or 2", this.getClass().getSimpleName());
			break;
		}
		
		//add previous agent obstacles to the map
		map.addAgentObstacles(_agentObstacles);
		
		PriorityQueue<AgentConflict> conflicts = cloneConflicts();
		removeAgentConflicts(conflicts, agent.getId());
		CBSState state = getNewState(agentObstacles, conflicts, agent);		
		//find and update path for this agent 
		Node root = AlgorithmUtils.getRootNode(_singleSearch);
		List<Agent> agents = new ArrayList<Agent>();
		agents.add(agent);
		long timeLimit = timelimit - (GeneralUtils.getCurrentTimeInMilliseconds() - initialTime);
		try {
			runAlgorithm(map, agents, root, timeLimit, initialTime);
		} catch (SolutionNotFoundException | TimeoutException e) {
			return null;
		} 
		//update path
		state.getAllSingleAgentPaths().get(AgentUtils.getPathIndexFromAgentId(_allSingleAgentPaths, agent.getId())).setWaypoints(agent.getPath().getWaypoints(), true);
		//find conflicts
		state.findAgentConflict();
		//perform other updates
		state.otherUpdateds();
		//return
		return state;
	}
	
	public void runAlgorithm(Map map, List<Agent> agents, Node root, long timelimit, long initialTime) throws SolutionNotFoundException, TimeoutException{
		AlgorithmUtils.runAlgorithm(map, agents, root, _singleSearch, _singleSearch, timelimit, initialTime, 0, null);
	}
	
	public void otherUpdateds() {
		//to use as an abstract method
	}
	
	public CBSState getNewState(HashMap<Integer, HashSet<PathLine>> agentObstacles, PriorityQueue<AgentConflict> conflicts, Agent agent) {
		CBSState state = new CBSState(_gVal, Double.MAX_VALUE, _fVal, Float.MAX_VALUE, agentObstacles, _singleSearch, conflicts, agent, _heuristic);
		state.setAllSingleAgentPaths(AgentUtils.clonePaths(_allSingleAgentPaths));	
		return state;
	}
	
	/**
	 * Removes all conflicts which is between the given agent and another agent
	 * @param conflicts
	 * @param agentId
	 */
	public static void removeAgentConflicts(PriorityQueue<AgentConflict> conflicts, int agentId) {
		Iterator<AgentConflict> iter = conflicts.iterator(); 
		while (iter.hasNext()) {
			AgentConflict conflict = iter.next();
			if (conflict.getAgent1().getId() == agentId || conflict.getAgent2().getId() == agentId) {
				iter.remove();
			}	
		}
	}

	public List<Path> getAllSingleAgentPaths() {
		return _allSingleAgentPaths;
	}

	public float getTotalCost() {
		return _totalCost;
	}
	
	public int getNumberOfConflicts() {
		int result =  _conflicts.size();
		if(_earliestConflict != null) {
			result++;
		}
		return result;
	}

	public void setAllSingleAgentPaths(ArrayList<Path> allSingleAgentPaths) {
		this._allSingleAgentPaths = allSingleAgentPaths;
	}

	public void setTotalCost(float totalCost) {
		this._totalCost = totalCost;
	}

	public AlgorithmConflict getEarliestConflict() {
		return _earliestConflict;
	}

	public void setEarliestConflict(AgentConflict earliestConflict) {
		this._earliestConflict = earliestConflict;
	}

	public Agent getAgent() {
		return _agent;
	}

	public void setAgent(Agent agent) {
		this._agent = agent;
	}

	public PriorityQueue<AgentConflict> getConflicts() {
		return _conflicts;
	}

}
