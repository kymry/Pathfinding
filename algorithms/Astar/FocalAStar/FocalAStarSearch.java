package algorithms.Astar.FocalAStar;

import agents.Agent;
import algorithms.Astar.AStarSearch;
import algorithms.Astar.AStarState;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import exceptions.SolutionNotFoundException;
import geometry.coords.AgentPosition;
import map.Map;

public class FocalAStarSearch extends AStarSearch {
	
	protected double _weight;

	public FocalAStarSearch(Map map, Agent agent, Node root, HeuristicInterface heuristic, double weight) {
		super(map, agent, root, heuristic);
		_weight = weight;
	}
	
	@Override
	public void search() throws SolutionNotFoundException {
		searchFocal(_weight);
	}
	
	@Override
	public void reset(long timeLimit, long initialTime) {
		super.reset(timeLimit, initialTime);
		_lowerBound = Float.MAX_VALUE;
	}
	
	@Override
	public void setInitialValues() {
		super.setInitialValues();
		// set the minimum value of f
		_lowerBound = _root.getState().getFVal();
	}
	
	@Override
	public Node getNextBestNode() {
		Node node = _focalList.poll();
		_openList.remove(node);
		return node;
	}
	
	@Override
	public void otherNodeUpdates(Node node) {
		if(node.getState().getLowerBound() <= _lowerBound * _weight) {
			_focalList.add(node);
		}
	}
	
	@Override
	public boolean isGoalNode(Node current) throws SolutionNotFoundException {
		//check coordinates
		boolean goalReached = ((AStarState)current.getState()).getPosition().equals(_goalVoxel);
		//check time of arrival
		if(goalReached && checkGoalNodeValidity()) {
			goalReached = !checkGoalConflicts(current);
		}		
		return goalReached;
	}
	
	/**
	 * @return true if goal node is valid
	 * @throws SolutionNotFoundException 
	 */
	public boolean checkGoalNodeValidity() throws SolutionNotFoundException {
		boolean oneValid = false;
		boolean validGoal = true;
		try {
			checkETAValidity((AgentPosition)((AStarState)_bestNode.getParent().getState()).getPosition());
			oneValid = true;
		}
		catch(SolutionNotFoundException e) {
			validGoal = false;
			//check ETA validity for all nodes in the focal list
			for(Node node : _focalList) {
				try {
					checkETAValidity((AgentPosition)((AStarState)node.getParent().getState()).getPosition());
					//if at least one was found, continue the search
					oneValid = true;
					break;
				} catch (SolutionNotFoundException e1) {
					//do nothing
				}
			}
		}	
		if(!oneValid) {
			throw new SolutionNotFoundException(getAlgorithmName());
		}
		return validGoal;
	}
	
	@Override
	public String getAlgorithmName() {
		return "Focal A*";
	}

}
