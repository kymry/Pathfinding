package algorithms.Astar.SMAStar;

import java.util.List;

import agents.Agent;
import algorithms.Astar.AStarSearch;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import map.Map;
import utils.Debug;
import utils.DebugEnums;
import utils.GeometryUtils;

public class SMAStarSearch extends AStarSearch {
	
	private int _nodesInMemory;
	private final int _maxMemory = 10;
	
	public SMAStarSearch(Map map, Agent agent, Node root, HeuristicInterface heuristic) {
		super(map, agent, root, heuristic);
	}
	
	@Override
	public boolean checkForGoalState() {
		if(isGoalNode(_bestNode)) {
			returnFromGoalState();
			Debug.print(DebugEnums.INFO, getAlgorithmName() + ": Solution found");
			return true;
		}
		if(_bestNode.getState().getFVal() == Double.POSITIVE_INFINITY) {
			Debug.print(DebugEnums.ERROR, "There is no solution that fits in the given memory");
		}
		return false;
	}
	
	@Override 
	public void exploredNode() {
		//Do nothing
	}
	
	@Override
	public boolean isGoalNode(Node current) {
		return _goal.getHVal() <= this._agent.getSpeed();
	}
	
	@Override
	public String getAlgorithmName() {
		return "SMA*";
	}
	
	@Override
	public boolean updateGoal(Node goal) {
		SMAStarState goalState = (SMAStarState) goal.getState();
		SMAStarState parentState = (SMAStarState) goal.getParent().getState();
		updateTimestep(goalState, parentState, GeometryUtils.getDistanceBetweenPoints3D(goalState.getPosition(), parentState.getPosition()));
		return false;
	}
	
	@Override
	public void expandNode() {
		List<State> neighbors;
		if(_expanded.containsValue(_bestNode)) {
			neighbors = ((SMAStarState)_bestNode.getState()).getForgottenNodes();
		}
		else {
			//get all possible neighbors
			neighbors = ((SMAStarState)_bestNode.getState()).expand(_map, _agent);
		}
		//create nodes for the neighbor states and add them to the open list
		processNeighbors(neighbors, _bestNode, _goal);
		_nodesInMemory += neighbors.size();
		while(_nodesInMemory > _maxMemory) {
			cutWorst();
		}
		
	}
	
	@Override
	public void processNeighbors(List<State> neighbors, Node current, State goal) {
		_expanded.put(_bestNode, _bestNode);
		SMAStarState bestState = (SMAStarState) _bestNode.getState();
		for(State neighbor : neighbors) {
			if(bestState.getForgottenTable().containsKey(neighbor)) {
				neighbor.setFVal(bestState.getForgottenTable().get(neighbor));
				bestState.getForgottenTable().remove(neighbor);
			}
			else if(!isGoalNode(_bestNode) && ((SMAStarState)neighbor).expand(_map, _agent).isEmpty()) {
				neighbor.setFVal(Float.POSITIVE_INFINITY);
			}
			else {
				neighbor.setGVal(bestState.getGVal() + _agent.getSpeed());
				neighbor.setHVal(_heuristic.calcHeuristic(neighbor, goal));
			}
			//compute the cost until this node
			double distance = computeAndUpdateGCost(neighbor, _bestNode.getState(), goal);
			addNewNode(_bestNode, neighbor, goal, distance);
		}		
	}
	
	@Override
	public void setInitialValues() {
		super.setInitialValues();
		_nodesInMemory = 1;
	}
	
	public void cutWorst() {
		Node worst = (Node)_openList.pollLast();
		Node parent = worst.getParent();
		if(parent != null) {
			((SMAStarState)parent.getState()).getForgottenTable().put((SMAStarState) worst.getState(), worst.getState().getFVal());
			
			//TODO check exactly what the comparison is about 
			if(!_openList.contains(parent) && !parent.getState().equals(_bestNode.getState())) {
				_openList.add(parent);
			}
		}
		_nodesInMemory = _openList.size();
	}

}
