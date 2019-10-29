package algorithms.Astar.ODStar;

import agents.Agent;
import agents.path.PathLine;
import algorithms.Astar.AStarState;
import algorithms.BFSearch;
import algorithms.comparators.AgentPathStartTimeComparator;
import algorithms.comparators.FValueComparator;
import algorithms.comparators.WeightedFValueComparator;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import exceptions.SolutionNotFoundException;
import geometry.coords.AgentPosition;
import geometry.coords.PointInterface;
import main.DefaultParameters;
import map.Map;
import utils.GeometryUtils;

import java.awt.*;
import java.util.*;
import java.util.List;

/**
 * A* with operator decomposition and independence detection (A*+OD)
 */
public class ODStarSearch extends BFSearch {

    protected HashMap<Integer, Agent> agents;
    private int numAgents;
    private Agent currentAgent;
    private HashMap<Node, Node> createdNodes = new HashMap<>();

    public ODStarSearch( Map map, Node root, HeuristicInterface heuristic, List<Agent> agents, HashMap<Integer,
            HashSet<PathLine>> conflictAvoidanceTable) {
        super(map, root, heuristic, new WeightedFValueComparator(DefaultParameters.aStarWeight));
        this.agents = createAgentMap( agents );
        numAgents = agents.size();
        // start with the agent with min start time
        currentAgent = Collections.min( this.agents.values(), new AgentPathStartTimeComparator() );
    }

    @Override
    public void createStartState(){

        // creat agent positions map
        HashMap<Integer, PointInterface> agentPositions = createAgentPositionsMap();

        // create the root state - all agents lack a path of any kind
        State start = new ODStarState(0, -1, -1, agentPositions, 0, true, agents, _map);

        //compute and set heuristic
        start.setHVal(_heuristic.calcHeuristic(start, _goal));

        //add state to root node
        _root.setState(start);
    }

    @Override
    public void expandNode(){
        List<State> neighbors = ((ODStarState)_bestNode.getState()).expand(_map);
        processNeighbors( neighbors, _bestNode, _goal );
    }

    /**
     * @param current node
     * @return true if every agent's goal position is equal to their current position
     */
    @Override
    public boolean isGoalNode(Node current) {
        HashMap<Integer, PointInterface> agentPositions = ((ODStarState) current.getState()).getPositions();

        for ( Agent agent : agents.values() ){
            if ( !(agent.getGoal().equals(agentPositions.get(agent.getId()))) ){
                return false;
            }
        }
        return true;
    }

    /**
     * Calculates the states f/h/g costs and adds legal states to open list
     * @param states - all states to be processed
     * @param current - current state in the search tree
     * @param goal - goal state
     */
    private void processNeighbors(List<State> states, Node current, State goal){

        for ( State neighbor : states ){
            // create new node
            Node newNode = createNewNode(current, neighbor, goal, neighbor.getGVal());

            // check if goal state or node already expanded
            boolean goalNode = isGoalNode( newNode );
            boolean expanded = _closedList.contains( newNode );

            boolean standardState = ((ODStarState)neighbor).isStandardState();

            // intermediate nodes that are duplicate standard nodes are not expanded
            if (!goalNode && expanded && !standardState) { continue; }

            // compute the gValue of node
            computeAndUpdateGCost( neighbor, current.getState(), goal );
            boolean isduplicate = createdNodes.containsKey( newNode ); // intermediate and standard nodes

            // add to open list IF conditions met
            try {
                if ( expanded && !isduplicate ) { ; }
                else if ( goalNode || !isduplicate || standardState || newNode.getState().getGVal() < createdNodes.get( newNode ).getState().getGVal() ) {
                    createdNodes.put(newNode, newNode);
                    //_openList.add( newNode );
                    openList.add( newNode );
                }
            } catch (NullPointerException e){
                System.out.println(e);
            }
        }
    }

    /**
     * creates a hashmap from agent id (Integer) to current agent position (PointInterface)
     * @return agent positions map
     */
    private HashMap<Integer, PointInterface> createAgentPositionsMap(){
        HashMap<Integer, PointInterface> agentPositions = new HashMap<>();
        for (Agent agent : agents.values()){
            agentPositions.put(agent.getId(), agent.getPath().getStart());
        }
        return agentPositions;
    }

    private Node createNewNode( Node current, State neighbor, State goal, double newGvalue ){
        setNodeTimeStep( neighbor );
        neighbor.setHVal( _heuristic.calcHeuristic( neighbor, goal ) );
        return new Node(current, neighbor);
    }

    private void setNodeTimeStep(State neighbor){
        int currentAgentId = ((ODStarState)neighbor).getCurrentAgentId();
        double newTimestep = ((AgentPosition)((ODStarState)neighbor).getPositions().get(currentAgentId)).getEta();
        ((ODStarState)neighbor).setTimestep(newTimestep);
    }

    /**
     * @return true if agent is active during current time step
     */
    private boolean  isAgentActive(Agent agent){

        // iof agent start time step > current time step - agent is not active
        if ( agent.getPath().getStartTimestep() > ((ODStarState)_bestNode.getState()).getTimestep()
            || agent.getGoal().equals(((ODStarState)_bestNode.getState()).getPositions().get(agent.getId())) ) {
            return false;
        } else if ( agent.getGoal().equals(((ODStarState)_bestNode.getState()).getPositions().get(agent.getId())) ) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * @param agents - hash map will be created from all agents
     * @return hash map of agent ids to agents
     */
    private HashMap<Integer, Agent> createAgentMap(List<Agent> agents){
        HashMap<Integer, Agent> agentMap = new HashMap<>();
        preprocessAgentPaths( agents );
        for (Agent agent : agents){
            agentMap.put(agent.getId(), agent);
        }
        return agentMap;
    }

    /**
     * Agent paths must be reduced to a single starting waypoint. Paths will then be built off this path
     * @param agents - list of agents to preprocess
     */
    private void preprocessAgentPaths(List<Agent> agents){
        int id = 0;
        for ( Agent agent : agents ){
            AgentPosition startPosition =  (AgentPosition)agent.getStart().clonePoint();
            AgentPosition goalPosition =  (AgentPosition)agent.getGoal().clonePoint();
            List<PointInterface> waypoints =  new ArrayList<>();
            waypoints.add(startPosition);

            agent.getPath().setWaypoints( waypoints, true );
            agent.getPath().setGoalPosition( goalPosition );
            agent.getPath().setStartPosition( startPosition );

            agent.setIndex( id++ );
        }
    }

    /**
     * Computes and updates the G cost for the state
     * @param neighbor - state for which to compute G cost
     * @param current - current state used in calculation of G cost
     * @param goal - goal state used in calculation of G cost
     */
    private void computeAndUpdateGCost(State neighbor, State current, State goal){

        // at each new state, only the cost of the current agent changes
        int distanceInTime = 0;
        int currentAgentId = ((ODStarState)current).getCurrentAgentId();

        PointInterface currentPosition = ((ODStarState)current).getPositions().get(currentAgentId);
        PointInterface neighborPosition = ((ODStarState)neighbor).getPositions().get(currentAgentId);
        PointInterface goalPosition = ((ODStarState)current).getAgents().get(currentAgentId).getGoal();

        if(currentPosition.equals(neighborPosition) &&  !currentPosition.equals(goalPosition)) {
            distanceInTime++;
        }

        double newGValue = current.getGVal()
                + distanceInTime
                + ((ODStarState)current).calculateChangeInGvalue( neighbor, currentAgentId );

        neighbor.setGVal( newGValue );
    }

    @Override
    public void checkBestNodeValidity(boolean goalReached) throws SolutionNotFoundException{
        // Simply checks if a solution exists. If not, the algorithm ceases and no solution error is returned
        // TODO: implement this if time (not strictly necessary)
    }

    public void checkETAValidity(AgentPosition current) throws SolutionNotFoundException {
        // TODO: implement this if time (not strictly necessary)
    }

    @Override
    public String getAlgorithmName(){return "A*+OD";}

    @Override
    public void returnFromGoalState(){;}

    @Override
    public void updateNoSolution(){;}

    @Override
    public void createGoalState(){ }
}