package algorithms;

import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import agents.Agent;
import algorithms.Astar.ODStar.ODStarSearch;
import algorithms.Astar.ODStar.ODStarState;
import algorithms.CBS.GreedyCBS.GreedyCBSState;
import algorithms.comparators.AgentConflictComparator;
import algorithms.comparators.ConflictCountComparator;
import algorithms.comparators.PairCountComparator;
import com.google.common.collect.MinMaxPriorityQueue;
import algorithms.heuristics.HeuristicInterface;
import dataStructures.ThreePriorityQueue;
import utils.Debug;
import utils.DebugEnums;
import utils.TestUtils;
import algorithms.trees.Node;
import algorithms.trees.State;
import exceptions.SolutionNotFoundException;
import map.Map;
import utils.Utils;

public abstract class BFSearch extends Search{
	protected int _numExpanded;
	protected int _numGenerated;
	protected double _duration;
	protected double _lowerBound; //for focal searches
	protected Map _map;	
	protected Node _root;	
	protected Node _bestNode;
	protected State _goal;
	protected MinMaxPriorityQueue<Node> _openList;
	protected PriorityQueue<Node> openList;
	protected PriorityQueue<Node> _focalList = new PriorityQueue<Node>(); //for focal searches
	protected HashSet<Node> _closedList;
	protected HeuristicInterface _heuristic;
	protected HeuristicInterface _secondHeuristic;
	protected Comparator<Node> _openComparator;
	protected ThreePriorityQueue<Node> greedyCBSOpenList;

	//Force a search thread to shut down after a timeout 
	private volatile boolean _shutdown = false;
	
	//No need to implement a default BFS algorithm
	public BFSearch(Map map, Node root, HeuristicInterface heuristic, Comparator<Node> openComparator) {
		_map = map;
		_root = root;
		_heuristic = heuristic;
		_openComparator = openComparator;
		setBestNode(null);
	}

	public abstract void createStartState();
	public abstract void createGoalState();
	public abstract void expandNode();
	public abstract void returnFromGoalState();
	public abstract void updateNoSolution();
	public abstract void checkBestNodeValidity(boolean goalReached) throws SolutionNotFoundException;
	public abstract boolean isGoalNode(Node current) throws SolutionNotFoundException;
	public abstract String getAlgorithmName();
	
	
	
	/**
	 * @param timeLimit
	 * @return true if there was a timeout
	 */
	@Override
	public boolean findPath(long timeLimit, long initialTime){
		//this should never happen
		if(timeLimit < 0) {
			_noSolution = true;
			getBestNode().setTimeout(true);
			updateNoSolution();
			Debug.print(DebugEnums.ERROR, "TIMEOUT ERROR: timelimit is negative: " + timeLimit + "; Returning from search!");	
			return true;
			
		}
		final ExecutorService service = Executors.newSingleThreadExecutor();
		service.submit(() -> {
			try {
				// set all variables
				reset(timeLimit, initialTime);
				// create start and goal states
				createGoalState();
				createStartState();
				setInitialValues();

				search();
			}
			catch(Exception e) {
				TestUtils.printExceptionMessage(e.getMessage(), "");
				_noSolution = true;
			}

		});
		
		try {
			service.shutdown();
			// wait for thread to finish
			boolean finished = service.awaitTermination(timeLimit, TimeUnit.MILLISECONDS);
			if (!finished) {
				// force thread to finish
				_shutdown = true;
				service.shutdownNow();
				
				// thread should finish immediately this time
				finished = service.awaitTermination(timeLimit, TimeUnit.MILLISECONDS);
				if (!finished) {
					Debug.print(DebugEnums.ERROR, "Thread for findPath did not finish after being forced to finish, we waited " + timeLimit);					
				} else {
					Debug.print(DebugEnums.DEBUG, "Thread for findPath finished after being forced, we waited " + timeLimit + " or less");
				}
				
				//return best state for CBS based searches
				getBestNode().setTimeout(true);
				updateNoSolution();
				return true;
			}
		}
		catch (InterruptedException e) {
			// This should never happen, except if the program using the PFCDR library interrupts our thread
			Debug.print(DebugEnums.ERROR, "Interrupted while waiting for findPath to finish.\n" + e.toString());
			
			//return best state for CBS based searches
			getBestNode().setTimeout(true);
			updateNoSolution();
			return true;
		}
		finally {
            service.shutdown();
        }
		return false;
	}
	
	public void search() throws SolutionNotFoundException {

		// A*+OD
		if ( _root.getState() instanceof ODStarState ){
			while ( !openList.isEmpty() ) {
				if ( getBestNodeAndExpand() || _shutdown || Thread.interrupted() ) {
					return;
				}
			}
		}

		// greedy CBS
		else if ( _root.getState() instanceof GreedyCBSState){
			while ( !greedyCBSOpenList.isEmpty() ) {
				if ( getBestNodeAndExpand() || _shutdown || Thread.interrupted() ) {
					return;
				}
			}
		}

		// everything else
		else {
			while ( !_openList.isEmpty() ) {
				if ( getBestNodeAndExpand() || _shutdown || Thread.interrupted() ) {
					return;
				}
			}
		}

		//if it reaches here, a solution was not found
		throw new SolutionNotFoundException("Num nodes expanded: " +  _numExpanded + ". Num nodes generated: " + _numGenerated + " "  + getAlgorithmName());
	}

	// true if a solution is found
	public boolean getBestNodeAndExpand() throws SolutionNotFoundException{
		_bestNode = getNextBestNode();
		_numGenerated++;
		
		//check if reached the goal. If so, the search is finished
		if(checkForGoalState()) {
			/*
			if ( _bestNode.getState() instanceof  ODStarState){
				// TODO: this method takes longer than the algorithm itself
				smoothPaths( _map, _bestNode.getState() );
			}*/
			return true;
		}
		exploredNode();
		//expand the node
		expandNode();
		
		return false;
	}
	
	public Node getNextBestNode() {
		if ( _root.getState() instanceof ODStarState ){
			return openList.poll();
		} else if (_root.getState() instanceof GreedyCBSState ) {
			return greedyCBSOpenList.poll();
		} else {
			return _openList.poll();
		}
	}
	
	/**
	 * Some searches might need to perform extra verifications/computations
	 */
	public void exploredNode() throws SolutionNotFoundException{
		//do extra checks/computations
		checkBestNodeValidity(false);

		if ( _bestNode.getState() instanceof ODStarState && !(((ODStarState) _bestNode.getState()).isStandardState()) ){
			return;
		} else {
			//close the node
			_closedList.add( getBestNode() );
		}
	}
	
	public boolean checkForGoalState() throws SolutionNotFoundException {
		if(isGoalNode(_bestNode)) {
			returnFromGoalState();
			Debug.print(DebugEnums.INFO, getAlgorithmName() + ": Solution found");
			return true;
		}
		return false;
	}
	
	/**
	 * This method serves only for searches with focal lists. It is defined here since some sub-classes of sub-classes that do not
	 * share a super class, besides this one, have this code in common.
	 * @param weight
	 * Currently hardcoded to remove the node from focal list with the least number of conflicts.
	 */
	public void searchFocal(double weight) throws SolutionNotFoundException{
		_focalList.add(_root);
		_lowerBound = _root.getState().getLowerBound();
		while (!_focalList.isEmpty()) {
			if(getBestNodeAndExpand() || _shutdown || Thread.interrupted()) {
				return;
			}	
			//update focal list and lower bound
			if(!_openList.isEmpty()) {
				State stateAux = _openList.peek().getState();
				double openHead = stateAux.getLowerBound();
				//update focal list every time fMin increases as a lower bound
				if(openHead > _lowerBound) {
					for(Node node : _openList) {
						State state = node.getState();
						// if the new LB of the state is not in the focal list yet but is now less than the new lowest
						// cost * weight, add it to focal list.
						if(state.getLowerBound() <= weight * openHead && state.getLowerBound() > weight * _lowerBound) {
							// necessary to attempt remove first because Java priority queue allows duplicates
							_focalList.remove(node);
							_focalList.add(node);
						}
					}
					_lowerBound = openHead;
				}
			}
		}
		//if it reaches here, a solution was not found
		throw new SolutionNotFoundException(getAlgorithmName());
	}

	public void reset(long timeLimit, long initialTime) {
		_openList = MinMaxPriorityQueue.orderedBy(_openComparator).create();
		//openList = new PriorityQueue<>( Collections.reverseOrder(_openComparator) ); // prioirty queue for OD star search
		openList = new PriorityQueue<>( _openComparator );
		greedyCBSOpenList = new ThreePriorityQueue<>(new AgentConflictComparator(), new ConflictCountComparator(), new PairCountComparator() );
		_closedList = new HashSet<Node>();
		_numExpanded = 0;
		_numGenerated = 0;
		_duration = 0;		
		_timeLimit = timeLimit;
		_initialTime = initialTime;
	}
	
	public void setInitialValues() {
		// add root node with start state to the open list
		if ( _root.getState() instanceof ODStarState ) {
			openList.add( _root );
		} else {
			_openList.add( _root );
		}
		_numGenerated++;
	}

	@SuppressWarnings( "Duplicates" )
	public void smoothPaths(Map map, State state) {
		List<Agent> agents =  ((ODStarState)state).getAgents();
		//for each changed path
		for(int i = 0; i < agents.size(); i++) {
				//add all remaining agents paths to the map as they are.
				Map mapAux = map.clone();
				for(int j = i+1; j < agents.size(); j++) {
					Utils.addAgentObstacleFromPart(mapAux.getAgentObstacles(), agents.get(j).getPath());
				}
				//smooth
				Utils.smoothPath(agents.get(i).getPath(), map);

			//add result to the obstacles
			Utils.addAgentObstacleFromPart(map.getAgentObstacles(), agents.get(i).getPath());
		}
	}
	
	/***********************************************************/
	/* Getters & Setters									   */
	/***********************************************************/

	public HeuristicInterface getHeuristic() {
		return _heuristic;
	}

	public HeuristicInterface getSecondHeuristic() {
		return _secondHeuristic;
	}

	public Comparator<Node> getOpenComparator() {
		return _openComparator;
	}

	public MinMaxPriorityQueue<Node> getOpenList() {
		return _openList;
	}

	public HashSet<Node> getClosedList() {
		return _closedList;
	}
	
	public int getNumExpanded() {
		return _numExpanded;
	}

	public int getNumGenerated() {
		return _numGenerated;
	}

	public Map getMap() {
		return _map;
	}

	public double getDuration() {
		return _duration;
	}

	public Node getRoot() {
		return _root;
	}

	public Node getBestNode() {
		return _bestNode;
	}

	public void setBestNode(Node _bestNode) {
		this._bestNode = _bestNode;
	}

	public State getGoal() {
		return _goal;
	}
}

/*********************************************************************/
/*						Algorithm structure							 */
/*********************************************************************/

/*
public boolean findPath(){
	//set all variables for a new search
	reset();
	// create start and goal states - each subclass has its own implementation
	State goal = createGoalState();
	createStartState(goal);
	_openList.add(_root);
	while (!_openList.isEmpty()) {
		setBestNode(_openList.poll());
		_numGenerated++;
		
		//check if reached the goal. If so, the search is finished
		if(checkForGoalState(goal)) {
			return true;
		}
		//normally add node to the closed list
		exploredNode();
		//expand the node and get its children - each subclass has its own implementation
		expandNode(getBestNode(), goal);
	}
	return false;
}
 */
