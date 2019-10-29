package algorithms.CBS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import agents.Agent;
import agents.path.Path;
import agents.path.PathLine;
import algorithms.AlgorithmName;
import algorithms.BFSearch;
import algorithms.comparators.ConflictComparator;
import algorithms.comparators.HValueComparator;
import algorithms.conflicts.AgentConflict;
import algorithms.conflicts.AlgorithmConflict;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import exceptions.SolutionNotFoundException;
import map.Map;
import utils.AlgorithmUtils;

public class CBSSearch extends BFSearch {
	protected float _initialCost;
	protected float _finalCost;
	protected int _conflictCounter;
	protected int _numberOfAgents;
	protected List<Agent> _agents;
	protected AlgorithmName _singleSearch;
	protected ConflictComparator _conflictComparator;
	
	public CBSSearch(Map map, List<Agent> agents, Node root, HeuristicInterface heuristic, AlgorithmName singleSearch) {
		super(map, root, heuristic, new HValueComparator());
		_agents = agents;
		_numberOfAgents = agents.size();
		_singleSearch = singleSearch;
		_conflictComparator = new ConflictComparator();
		AlgorithmUtils.performPruning(_agents);
	}
	
	@Override 
	public void reset(long timeLimit, long initialTime) {
		super.reset(timeLimit, initialTime);
		_initialCost = 0;
		_finalCost = 0;
		_conflictCounter = 0;
	}
	
	@Override
	public void createStartState() {
		//map the positions of the other agents that should be considered as obstacles 
		HashMap<Integer, HashSet<PathLine>> agentObstacles = new HashMap<Integer, HashSet<PathLine>>();
		//get the initial paths of the agents and total cost of the initial state
		PriorityQueue<AgentConflict> conflictsStart = new PriorityQueue<AgentConflict>(_conflictComparator);
		State startState = getNewState(_initialCost, agentObstacles, conflictsStart);
		((CBSState)startState).setAllSingleAgentPaths(getInitialPaths(startState));
		((CBSState)startState).findAllConflicts();
		_root.setState(startState);
	}
	
	public State getNewState(float cost, HashMap<Integer, HashSet<PathLine>> agentObstacles, PriorityQueue<AgentConflict> conflictsStart) {
		State startState = new CBSState(-1, Double.MAX_VALUE, -1, cost, agentObstacles, _singleSearch, conflictsStart, null, _heuristic);
		return startState;
	}
	
	public ArrayList<Path> getInitialPaths(State start) {
		ArrayList<Path> allSingleAgentPaths = new ArrayList<Path>();
		for (int i = 0; i < _numberOfAgents; i++) {
			allSingleAgentPaths.add(_agents.get(i).getPath().clone());
			_initialCost += (_agents.get(i).getPath().getEndTimestep() - _agents.get(i).getPath().getStartTimestep());
		}
		((CBSState)start).setTotalCost(_initialCost);
		return allSingleAgentPaths;
	}
	
	@Override
	public boolean isGoalNode(Node current) {
		//if there are no conflicts then the search has reached a goal state
		return ((CBSState)current.getState()).getEarliestConflict() == null;
	}
	
	@Override
	public void createGoalState() {
		PriorityQueue<AgentConflict> conflicts = new PriorityQueue<AgentConflict>(_conflictComparator);
		_goal = new CBSState(-1, 0, -1, -1, null, _singleSearch, conflicts, null, _heuristic);
	}
	
	@Override
	public void expandNode() {
		//expand and create children nodes base
		List<State> newStates = ((CBSState)_bestNode.getState()).expand(_map, _timeLimit, _initialTime);
		for(State newState : newStates) {
			addChildNode(_bestNode, (CBSState)newState);
		}	
	}

	@Override
	public void returnFromGoalState() {
		//if there was no solution
		List<Agent> agentsInConflict = new ArrayList<Agent>();
		if(!((CBSState)_bestNode.getState()).getConflicts().isEmpty() || ((CBSState)_bestNode.getState()).getEarliestConflict() != null) {
			agentsInConflict.add(((CBSState)_bestNode.getState()).getEarliestConflict().getAgent1());
			agentsInConflict.add(((AgentConflict)((CBSState)_bestNode.getState()).getEarliestConflict()).getAgent2());
			for(AlgorithmConflict conflict : ((CBSState)_bestNode.getState()).getConflicts()) {
				agentsInConflict.add(conflict.getAgent1());
				agentsInConflict.add(((AgentConflict)conflict).getAgent2());
			}
		}
		//update agents paths
		List<Path> finalPaths = ((CBSState)_bestNode.getState()).getAllSingleAgentPaths();
		for(int i = 0; i < _agents.size(); i++) {
			//check if conflicts where unsolved - no general solution/timeout
			if(agentsInConflict.contains(_agents.get(i))) {
				finalPaths.get(i).setFreeOfConflicts(false);
				continue;
			}
			//in a goal state there should be no conflicts
			finalPaths.get(i).setFreeOfConflicts(true);
			_agents.get(i).setPath(finalPaths.get(i));
		}
	}
	
	@Override
	public String getAlgorithmName() {
		return "CBS";
	}
	
	/* (non-Javadoc)
	 * Updates the paths waypoints of solved agents and indicates the ones that are still with conflicts to be rejected
	 */
	@Override
	public void updateNoSolution() {
		returnFromGoalState();
	}
	
	@Override
	public void checkBestNodeValidity(boolean goalReached) throws SolutionNotFoundException {
		//
	}
	
	public void addChildNode(Node parent, CBSState state) {
		_openList.add(new Node(parent, state));
	}

	/***********************************************************/
	/* Getters & Setters									   */
	/***********************************************************/
	public double getInitial_cost() {
		return _initialCost;
	}
	public double getFinal_cost() {
		return _finalCost;
	}
	public int getConflict_counter() {
		return _conflictCounter;
	}
	public List<Agent> getAgents() {
		return _agents;
	}
}
