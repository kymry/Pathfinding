package algorithms.Astar;

import java.util.HashMap;
import java.util.List;

import agents.Agent;
import agents.path.PathLine;
import algorithms.BFSearch;
import algorithms.comparators.FValueComparator;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import exceptions.SolutionNotFoundException;
import geometry.coords.AgentPosition;
import geometry.coords.PointInterface;
import geometry.coords.UTMCoord;
import map.Map;
import utils.AgentUtils;
import utils.AlgorithmUtils;
import utils.GeometryUtils;
import utils.Utils;

public class AStarSearch extends BFSearch {
	
	protected HashMap<Node, Node> _expanded;
	protected Agent _agent;
	protected UTMCoord _goalVoxel;

	public AStarSearch(Map map, Agent agent, Node root, HeuristicInterface heuristic) {
		super(map, root, heuristic, new FValueComparator());
		_agent = agent;
	}
	
	@Override
	public void reset(long timeLimit, long initialTime) {
		super.reset(timeLimit, initialTime);
		_expanded = new HashMap<Node, Node>();
	}
	
	@Override
	public void createStartState() {
		State start = createStartStatePosition();
		//compute and set heuristic
		start.setHVal(_heuristic.calcHeuristic(start, _goal));
		//add state to root node
		_root.setState(start);
	}
	
	public State createStartStatePosition() {
		return new AStarState(0, -1, -1, (AgentPosition)_agent.getStart());
	}
	
	@Override
	public void createGoalState() {		
		_goal = new AStarState((AgentPosition)_agent.getGoal());
		_goalVoxel = (UTMCoord) Utils.convertPositionToVoxel((UTMCoord)((AStarState)_goal)._position);
	}
	
	@Override
	public void returnFromGoalState() {
		updatePath();
	}

	@Override
	public void expandNode() {
		//get all possible neighbors
		List<State> neighbors = ((AStarState)_bestNode.getState()).expand(_map, _agent);
		
		//create nodes for the neighbor states and add them to the open list
		processNeighbors(neighbors, _bestNode, _goal);
	}
	
	@Override
	public boolean isGoalNode(Node current) throws SolutionNotFoundException {
		//check coordinates
		boolean goalReached = ((AStarState)current.getState()).getPosition().equals(_goalVoxel);
		//check time of arrival
		checkBestNodeValidity(goalReached);
		//check if it is possible to form the parent to the real goal position
		if(goalReached) {
			goalReached = !checkGoalConflicts(current);
		}		
		return goalReached;
	}
	
	@Override
	public String getAlgorithmName() {
		return "A*";
	}
	
	@Override
	public void updateNoSolution() {
		// solution is already updated
	}
	
	@Override
	public void checkBestNodeValidity(boolean goalReached) throws SolutionNotFoundException{
		AgentPosition current;
		// Check if still possible to arrive in time
		if(goalReached) {
			//if it has reached the goal voxel, the real path line is between its parent and the real goal position
			current = (AgentPosition)((AStarState)_bestNode.getParent().getState()).getPosition();
		}
		else {
			current = (AgentPosition)((AStarState)_bestNode.getState()).getPosition();
		}
		checkETAValidity(current);
	}
	
	public void checkETAValidity(AgentPosition current) throws SolutionNotFoundException {
		// get the minimum time it takes to reach the goal position
		double timeToGoal = GeometryUtils.getTimeFromDistance(GeometryUtils.getDistanceBetweenPoints3D(current, ((AStarState) _goal).getPosition()), _agent.getSpeed());

		if (((AgentPosition) ((AStarState) _bestNode.getState()).getPosition()).getEta() + timeToGoal > ((AgentPosition) ((AStarState) _goal).getPosition()).getMaxETA()) {
			throw new SolutionNotFoundException("Cannot reach Region by ETA + delta. " + getAlgorithmName());
		}
	}
	
	/**
	 * @param current
	 * @return true if there are conflicts, false if not
	 */
	public boolean checkGoalConflicts(Node current) {
		if(current.getParent() == null) {
			return false;
		}
		Node goal = current.clone();
		return !updateGoal(goal);
	}
	
	public void updatePath() {
		updateGoal(_bestNode);
		List<PointInterface> newPath = AlgorithmUtils.reconstructPath(_bestNode);
		_agent.setPath(newPath, true);
		_agent.getPath().cleanPath();
	}
	
	/**
	 * @param goal
	 * @return true if updated correctly
	 */
	public boolean updateGoal(Node goal) {
		AStarState goalState = (AStarState) goal.getState();
		AStarState parentState = (AStarState) goal.getParent().getState();
		goalState.setPosition(_agent.getGoal());
		double goalETA = ((AgentPosition)((AStarState)_goal).getPosition()).getEta();
		double minETA = ((AgentPosition)((AStarState)_goal).getPosition()).getMinETA();
		updateTimestep(goalState, parentState, GeometryUtils.getDistanceBetweenPoints3D(goalState.getPosition(), parentState.getPosition()));
		//check move validity - NOTE: map can only be null during tests
		if(_map != null && goal.getParent() != null) {
			if(!_map.isMoveValid(new PathLine(((AStarState)goal.getParent().getState())._position, ((AStarState)goal.getState())._position, _agent), _goalVoxel)){
				return false;
			}
		}		
		//check if the ETA is too early - add new waypoint at the goal if so
		if(((AgentPosition)goalState.getPosition()).getEta() < minETA) {
			Node goalAux = goal.clone();
			goal.setParent(goalAux);
			((AgentPosition)((AStarState)goal.getState()).getPosition()).setEta(goalETA);
			AlgorithmUtils.addWaitStates(goalAux, goal);
			//check wait validity
			Node current = goal;
			while(((AgentPosition)((AStarState)current.getState()).getPosition()).getEta() != ((AgentPosition)((AStarState)goalAux.getState()).getPosition()).getEta()) {
				if(!_map.isMoveValid(new PathLine(((AStarState)current.getParent().getState())._position, ((AStarState)current.getState())._position, _agent), _goalVoxel)){
					return false;
				}
				current = current.getParent();
			}
			return true;
		}
		return true;
	}
	
	public void processNeighbors(List<State> neighbors, Node current, State goal) {
		for(State neighbor : neighbors) {
			//check if this neighbor was already expanded
			Node auxNode = new Node(null, neighbor);
			boolean goalNode = false;
			if(((AStarState)neighbor).getPosition().equals(_goalVoxel)) {
				goalNode = true;
			}
			if(!goalNode && _closedList.contains(auxNode) && !((AStarState)neighbor)._waitMove) {
				continue;
			}
			//compute the cost until this node
			double distance = computeAndUpdateGCost(neighbor, current.getState(), goal);
			//add node if it was never visited or if it was but this is a better path
			boolean isVisited = _expanded.containsKey(auxNode);
			if(goalNode || !isVisited || (isVisited && neighbor.getGVal() < _expanded.get(auxNode).getState().getGVal())) {
				addNewNode(current, neighbor, goal, distance);
			}
		}
	}
	
	/**
	 * To use with sub classes
	 */
	public void otherNodeUpdates(Node node) {
		//
	}
	
	public void addNewNode(Node current, State neighbor, State goal, double distance) {

		//update timestep
		AgentUtils.updateTimestep((AgentPosition)((AStarState)neighbor).getPosition(), (AgentPosition)((AStarState)current.getState()).getPosition(), _agent.getSpeed());
		Node newNode = createNeighborNode(neighbor, current, goal, distance);
		_expanded.put(newNode, newNode);
		_openList.add(newNode);
		otherNodeUpdates(newNode);
	}
	
	/**
	 * Computes the cost between the start and the neighbor (updates the value on the neighbor state)
	 * @param neighbor
	 * @param current
	 * @param goal
	 * @return distance between current and neighbor
	 */
	public double computeAndUpdateGCost(State neighbor, State current, State goal) {
		double distance = stateDistance(neighbor, current, goal);
		//if distance is zero then it is the same position - wait movement
		if(distance == 0) {
			distance++;
		}
		neighbor.setGVal(current.getGVal() + distance);
		return distance;
	}
	
	public double stateDistance(State neighbor, State current, State goal) {
		//this variable is needed for when the agent is waiting at the same position between two timesteps;
		int distanceInTime = 0;
		PointInterface currentPosition = ((AStarState)current).getPosition();
		PointInterface neighborPosition = ((AStarState)neighbor).getPosition();
		PointInterface goalPosition = ((AStarState)goal).getPosition();
		if(currentPosition.equals(neighborPosition) &&  !currentPosition.equals(goalPosition)) {
			distanceInTime++;
		}
		return distanceInTime + current.calculateDistance(neighbor);
	}
	
	public double stateDistance(State neighbor, State current) {
		return current.calculateDistance(neighbor);
	}
	
	public Node createNeighborNode(State neighbor, Node current, State goal, double distance) {
		updateTimestep(neighbor, current.getState(), distance);
		updateFValue(neighbor, goal);
		return new Node(current, neighbor);
	}
	
	public void updateTimestep(State neighbor, State current, double distance) {
		((AStarState)neighbor).setTimestep(((AStarState)current).getTimestep() + GeometryUtils.getTimeFromDistance(distance, _agent.getSpeed()));
		((AgentPosition)((AStarState)neighbor).getPosition()).setEta(((AStarState)neighbor).getTimestep());
	}
	
	public void updateFValue(State neighbor, State goal) {
		//F value is being updated together with the H value
		neighbor.setHVal(_heuristic.calcHeuristic(neighbor, goal));
	}
	
	/***********************************************************/
	/* Getters & Setters									   */
	/***********************************************************/
	
	public HashMap<Node, Node> getExpanded() {
		return _expanded;
	}

	public Agent getAgent() {
		return _agent;
	}
}
