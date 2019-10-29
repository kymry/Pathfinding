package algorithms.Astar.ODStar;

import agents.Agent;
import agents.path.PathLine;
import algorithms.Astar.AStarState;
import algorithms.trees.State;
import geometry.coords.AgentPosition;
import geometry.coords.PointInterface;
import geometry.coords.UTMCoord;
import main.ConflictDetection;
import main.DefaultParameters;
import map.Map;
import utils.AgentUtils;
import utils.GeneralUtils;
import utils.GeometryUtils;
import utils.Utils;

import javax.swing.*;
import java.util.*;

public class ODStarState extends State{

    protected HashMap<Integer, Agent> agents;                      // map of agent ids to agents
    private HashMap<Integer, PathLine> moves;                      // pre-move and post-move positions of all agents in intermediate nodes
    private HashMap<Integer, HashSet<PathLine>> agentObstacles;    // all occupied positions of agents in group
    private HashMap<Integer, PointInterface> positions;            // current positions of all agents
    private boolean standardState;                                 // true if standard node, false if intermediate node
    private double timestep;                                       // current time step
    private int currentAgentId;                                    // agent moving during this state
    private int numAgents;                                         // number of agents in total
    private int minAgentTime;                                      // minimum time of any agent (i.e. the time of the agent who's path is the minimum in time)
    private Map map;
    private double weight = DefaultParameters.aStarWeight;

    // Constructor for start state
    public ODStarState(double gval, double hval, double fval, HashMap<Integer, PointInterface> positions, int currentAgentId,
                       boolean standardState, HashMap<Integer, Agent> agents, Map map) {
        super(gval, hval, fval);
        this.timestep = 0;
        this.positions = positions;
        this.standardState = standardState;
        this.currentAgentId = currentAgentId;
        this.agents = agents;
        numAgents = agents.size();
        moves = new HashMap<>();
        this.map = map;
        this.agentObstacles = new HashMap<>();
        this.minAgentTime = 0;
    }

    // Constructor for standard state
    public ODStarState(double gval, double hval, double fval, HashMap<Integer, PointInterface> positionsCopy, int newAgentId,
                       boolean standardState, HashMap<Integer, Agent> agents, Map map, HashMap<Integer, HashSet<PathLine>> agentObstacles) {
        super(gval, hval, fval);
        this.timestep = (int) ( (AgentPosition)positionsCopy.get(newAgentId) ).getEta();
        this.positions = positionsCopy;
        this.standardState = standardState;
        this.currentAgentId = newAgentId;
        this.agents = agents;
        numAgents = agents.size();
        moves = new HashMap<>();
        this.map = map;
        this.agentObstacles = agentObstacles;
        minAgentTime = getMinAgentPathTime(positionsCopy);
    }

    // Constructor for intermediate state
    public ODStarState(double gval, double hval, double fval, HashMap<Integer, PointInterface> positions, int newAgentId,
                       boolean standardState, HashMap<Integer, PathLine> moves, HashMap<Integer, Agent> agents, Map map,
                       HashMap<Integer, HashSet<PathLine>> agentObstacles) {
        super(gval, hval, fval);
        this.timestep = (int) ( (AgentPosition)positions.get(newAgentId) ).getEta();
        this.positions = positions;
        this.standardState = standardState;
        this.currentAgentId = newAgentId;
        this.agents = agents;
        numAgents = agents.size();
        this.moves = moves;
        this.map = map;
        this.agentObstacles = agentObstacles;
        minAgentTime = getMinAgentPathTime(positions);
    }

    /**
     * Gets all legal neighbor states of a node - ignores illegal states
     * @param map to be used during expansion
     * @return list of all legal neighbor states
     */
    public List<State> expand( Map map ){

            Agent currentAgent = agents.get(currentAgentId);
            // convertPositionToVoxel returns a UTM coordinate in integer form (i.e. infinite number of points can be the same voxel)
            PointInterface goalPosition = Utils.convertPositionToVoxel( (UTMCoord) currentAgent.getGoal() );
            List<PointInterface> neighborPositions = getPossibleNeighborPositions( map, currentAgent.getRadius(), goalPosition );
            return getLegalNeighborStates( neighborPositions, currentAgent, map, goalPosition );
    }

    /**
     * Removes all neighbor states that are illegal
     * @param neighborPositions - all possible neighbor states
     * @param agent - agent to be expanded
     * @param map - map to be used in testing for state legality
     * @param goalPosition - position of the agents goal
     * @return list of all legal neighbor states
     */
    public List<State> getLegalNeighborStates(List<PointInterface> neighborPositions, Agent agent, Map map, PointInterface goalPosition){
        List<State> legalNeighborStates = new ArrayList<>();

        // filter out all the illegal neighbor states
        for(PointInterface neighborAgentPosition : neighborPositions) {

            // update the time step of the agent position (also sets wait variable in neighbor object)
            AgentUtils.updateTimestep((AgentPosition)neighborAgentPosition, (AgentPosition)positions.get(currentAgentId), agent.getSpeed());

            // this line represents the move from one voxel to the next - is only used to check for conflicts
            PathLine line = new PathLine(positions.get(currentAgentId), neighborAgentPosition, agent);

            if ( map.isMoveValid(line, goalPosition) && isLegalMove(line, agent) ) {
                //newState._waitMove = ((AgentPosition)neighbor).isWait();
                ODStarState newState = createChildState( neighborAgentPosition, line );
                legalNeighborStates.add(newState);
            }
        }
        return legalNeighborStates;
    }

    /**
     * Creates a child state
     * @param neighbor to create state from
     * @param line from pre-move to post-move position (to be used for agent obstacles)
     * @return new ODStarState
     */
    private ODStarState createChildState(PointInterface neighbor, PathLine line){

        // copy agent obstacles
        HashMap<Integer, HashSet<PathLine>> agentObstaclesCopy = copyAgentObstacles( line );

        // clone map and add agent obstacles
        Map mapCopy  =  map.clone();
        mapCopy.addAgentObstacle( (int)timestep, line );

        // clone agents, create agent from neighbor, add new agent to clone
        HashMap<Integer, Agent> copyAgents = updateAgents( (AgentPosition) neighbor );

        // clone agent positions and add update current agent position
        HashMap<Integer, PointInterface> copyPositions = new HashMap<>( positions );
        copyPositions.replace( currentAgentId, neighbor );

        // set next agent to be processed
        int nextAgentId = getNextAgentId(copyPositions);

        // create new state
        return createNewState( copyAgents, mapCopy, line, copyPositions, agentObstaclesCopy, nextAgentId);
    }


    /**
     * creates a new ODStarState
     * @param copyAgents
     * @param mapCopy
     * @param line
     * @return new ODStarState
     */
    private ODStarState createNewState( HashMap<Integer, Agent> copyAgents, Map mapCopy,
                                        PathLine line, HashMap<Integer, PointInterface> copyPositions,
                                        HashMap<Integer, HashSet<PathLine>> agentObstaclesCopy, int nextAgentId){
        ODStarState childState;

        // standard state
        if ( nextAgentId <= currentAgentId ){
            childState = new ODStarState( _gVal, _hVal, _fVal, copyPositions, nextAgentId, true, copyAgents, mapCopy, agentObstaclesCopy );
        }
        // intermediate state
        else {
            HashMap<Integer, PathLine> movesCopy = new HashMap<>(moves);
            if ( movesCopy.containsKey( currentAgentId ) ) { movesCopy.replace(currentAgentId, line); }
            else { movesCopy.replace(currentAgentId, line);}
            childState = new ODStarState( _gVal, _hVal, _fVal, copyPositions, nextAgentId, false, movesCopy, copyAgents, mapCopy, agentObstaclesCopy );
        }
        return childState;
    }

    /**
     * creates a copy of the agent, adds the new waypoint to the path and adds the agent to the new child state.
     * @param neighborPoint neighbor to be converted into agent
     */
    private HashMap<Integer, Agent> updateAgents(AgentPosition neighborPoint){

        // deep copy (unfortunately this is how it was originally implemented)
        AgentPosition goalPosition =  (AgentPosition)agents.get(currentAgentId).getGoal().clonePoint();
        Agent agentCopy = agents.get(currentAgentId).clone();

        // add new point to waypoints - doing so overrides goal position so need to temporarily store goal position
        List<PointInterface> points = agentCopy.getPath().getWaypoints();
        points.add( neighborPoint );
        agentCopy.getPath().setWaypoints( points, true );

        // reset goal position back to goal position
        agentCopy.getPath().setGoalPosition( goalPosition );

        // check if agent reached goal set hasPath == true if so
        if ( goalPosition.equals( neighborPoint ) ){
            agentCopy.getPath().setHasPath( true );
        }

        return copyAgentMap( agentCopy );
    }

    /**
     * Copies the agent map and updates the provided agent
     * @param agent to be updated in the agent map
     * @return shallow copy of agent map
     */
    private HashMap<Integer, Agent> copyAgentMap(Agent agent){
        HashMap<Integer, Agent> copyAgents = new HashMap<>( agents );
        copyAgents.replace( currentAgentId, agent );
        return copyAgents;
    }

    /**
     * copies the agent obstacles map and adds the new agent obstacles
     * @param line new agent obstacle to be added
     * @return shallow copy of agent obstacles map
     */
    private HashMap<Integer, HashSet<PathLine>> copyAgentObstacles(PathLine line){
        HashMap<Integer, HashSet<PathLine>> agentObstaclesCopy = GeneralUtils.cloneTimePathLineMap( agentObstacles );
        if ( agentObstaclesCopy.containsKey( (int)timestep ) ) {
            agentObstaclesCopy.get( (int)timestep ).add( line );
        } else {
            HashSet<PathLine> h = new HashSet<>();
            h.add(line);
            agentObstaclesCopy.put((int)timestep, h);
        }
        return agentObstaclesCopy;
    }

    /**
     * checks if a Pathline is a legal move
     * @param lineA  - path to be checked
     * @param agent - agent making the move
     * @return true if path is a legal move
     */
    private boolean isLegalMove( PathLine lineA, Agent agent ){

        // check for conflicts with pre/post moves table
        for ( PathLine lineB : moves.values()){
            if ( lineB.getAgent().equals(lineA.getAgent()) ){
                continue;
            }
            if ( ConflictDetection.checkPathLinesIntersection( lineA, lineB,true) ) {
                return false;
            }
        }
        return true;
    }

    /**
     * @return all the voxels around the state position that are inside the map. Does not check for any kind of obstacles
     */
    private List<PointInterface> getPossibleNeighborPositions(Map map, double agentRadius, PointInterface goal){
        List<PointInterface> possibleNeighbors = new ArrayList<PointInterface>();

        for(int x = -1; x <= 1; x++) {
            for(int y = -1; y <= 1; y++) {
                for(int z = -1; z <= 1; z++) {

                    AgentPosition newPosition = new AgentPosition(
                            (int)(positions.get(currentAgentId)).getX() + x,
                            (int)(positions.get(currentAgentId)).getY() + y,
                            (int)(positions.get(currentAgentId)).getZ() + z,
                            -1, -1, ((UTMCoord)positions.get(currentAgentId)).getZone(),
                            ((UTMCoord)positions.get(currentAgentId)).getLatitudeBand());
                    if(map.inMap(newPosition, goal, agentRadius)) {
                        possibleNeighbors.add(newPosition);
                    }
                }
            }
        }
        return possibleNeighbors;
    }

    /**
     * @param points - to check for minimum agent in time
     * @return id of agent that will be expanded in the next state
     * Agent returned will also be currently active (i.e. not yet finished it's route)
     */
    private int getNextAgentId(HashMap<Integer, PointInterface> points) {
        int minTime = getMinAgentPathTime(points);
        int startAgentId = (currentAgentId == numAgents - 1)
                ? 0
                : currentAgentId;

        for (int i = startAgentId; i < numAgents; ++i){
            // if agent is min time AND if agent has not finished
            if ( !agents.get(i).getPath().hasPath() && (int) ((AgentPosition)points.get(i)).getEta() <= minTime) {
                return i;
            }
            if ( i == numAgents - 1 ) { i = -1; }
        }
        return 0;
    }

    /**
     * @param points to check
     * @return time step of agent whose path is minimum in time
     */
    private int getMinAgentPathTime(HashMap<Integer, PointInterface> points){
        int minTime = Integer.MAX_VALUE;
        for (  java.util.Map.Entry<Integer, PointInterface> entry : points.entrySet() ){
            boolean hasPath = agents.get(entry.getKey()).getPath().hasPath();
            if ( !hasPath && ((AgentPosition)entry.getValue()).getEta() < minTime ){
                minTime = (int)((AgentPosition)entry.getValue()).getEta();
            }
        }
        return minTime;
    }

    /**
     * Gets the change (difference) in gvalues between one state and the next.
     * @param neighborState for which the change in gvalue will be calculated
     * @return the overall difference in gvalues (NOT the gvalue itself)
     */
    public double calculateChangeInGvalue(State neighborState, int currentAgentId){
        return GeometryUtils.getDistanceBetweenPoints3D( positions.get(currentAgentId),
                ((ODStarState)neighborState).getPositions().get(currentAgentId));
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof ODStarState)){ return false;}
        ODStarState other = (ODStarState)o;

        // check every agent position
        for ( java.util.Map.Entry<Integer, PointInterface> entry : other.getPositions().entrySet() ){
            int agentId = entry.getKey();

            UTMCoord thisPositionTemp =  (UTMCoord)positions.get(agentId);
            UTMCoord thisPosition = new UTMCoord((int)thisPositionTemp.getX(), (int)thisPositionTemp.getY(),
                    (int)thisPositionTemp.getZ(), thisPositionTemp.getZone(), thisPositionTemp.getLatitudeBand());

            UTMCoord otherPositionTemp = (UTMCoord)entry.getValue();
            UTMCoord otherPosition = new UTMCoord((int)otherPositionTemp.getX(), (int)otherPositionTemp.getY(),
                    (int)otherPositionTemp.getZ(), otherPositionTemp.getZone(), otherPositionTemp.getLatitudeBand());

            if ( !(thisPosition.equals(otherPosition)) ) {
                return false;
            }
        }
        return true ;
    }

    @Override
    @SuppressWarnings( "Duplicates" )
    public int compareTo(State o) {
        ODStarState other = (ODStarState)o;
        return Double.compare(_gVal + weight * _hVal, other._gVal + weight * other._hVal);
        /*if(_gVal + (weight * _hVal) < other._gVal + (weight * other._hVal)) {
            return -1;
        }
        if(_gVal + (weight * _hVal) > other._gVal + (weight * other._hVal)) {
            return 1;
        }
        return 0;*/
    }

    @Override
    public String toString() {
        StringBuilder out =  new StringBuilder();
        for ( PointInterface pos : positions.values() ){
            out.append(pos.toString() + "\n");
        }
        return out + super.toString();
    }

    @Override
    public int hashCode() {
        int hashValue = 0;
        for ( PointInterface p : positions.values() ){
            hashValue += ((AgentPosition)p).hashCode();
        }
        return hashValue;
    }

    @Override
    public State clone() {
        //TODO: probably need to shallow copy the map and agents. Currently just reference copy which violates invariant
        return new ODStarState(_gVal, _hVal, _fVal, new HashMap<>(positions), currentAgentId, standardState, agents, map, agentObstacles);
    }


    @Override
    public double calculateDistance(State state){
        return 0;
    }

    @Override
    public double getLowerBound(){
        return _fVal;
    }

    public List<Agent> getAgents(){
        return new ArrayList<>(agents.values());
    }

    public HashMap<Integer, PointInterface> getPositions(){
        return positions;
    }

    public PointInterface getPosition() { return positions.get(currentAgentId); }

    public int getCurrentAgentId(){ return currentAgentId; }

    public Agent getCurrentAgent(){ return agents.get(currentAgentId); }

    public double getTimestep(){ return timestep; }

    public boolean isStandardState() { return standardState; }

    public void setTimestep(double timestep) { this.timestep = timestep; }
}
