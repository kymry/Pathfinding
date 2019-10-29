package algorithms.IndependenceDetecetion;

import agents.*;
import agents.path.*;
import algorithms.*;
import algorithms.conflicts.AgentConflict;
import algorithms.conflicts.AlgorithmConflict;
import algorithms.trees.*;
import dataStructures.DisjointSetMap;
import exceptions.SolutionNotFoundException;
import exceptions.TimeoutException;
import main.ConflictDetection;
import main.DefaultParameters;
import map.Map;
import operations.ClientOperation;
import utils.*;

import java.util.*;

public class IndependenceDetectionSearch {

    protected Map map;
    private List<Agent> agents;
    private List<Path> paths;
    private AlgorithmName sapfAlgorithm;
    private AlgorithmName mapfAlgorithm;
    private DisjointSetMap agentGroups =  new DisjointSetMap(); // groups of agents with conflicting paths
    protected long timeLimit = DefaultParameters.timeLimit;
    protected long initialTime; // Only used to set the IndependenceDetectionSearch search algorithm time limit
    protected List<AlgorithmConflict> conflicts =  new ArrayList<>();
    private double weight = DefaultParameters.OIDWeight; // Allowed deviation from optimal path sum of costs
    private HashMap<Integer, ClientOperation> operationAgentMap;

    public IndependenceDetectionSearch( Map map, List<Agent> agents, AlgorithmName sapfAlgorithm, AlgorithmName mapfAlgorithm, HashMap<Integer, ClientOperation> operationAgentMap ) {
        this.map = map;
        this.agents = agents;
        this.sapfAlgorithm = sapfAlgorithm;
        this.mapfAlgorithm = mapfAlgorithm;
        this.operationAgentMap = operationAgentMap;

    }

    public void search(){

        //AgentUtils.setAgentsIndexes( agents );
        paths = InFlightUtils.getAgentPaths( agents );
        initialTime = GeneralUtils.getCurrentTimeInMilliseconds();
        agentGroups.makeSet( agents );

        // get initial conflicts between all agents
        findConflicts( null, true );

        // use the independence detection framework to find conflict free paths for all agents
        IndependenceDetection id = new IndependenceDetection( conflicts, agents, agentGroups );
        id.setFindPathMethod( new findPath() );
        id.findPaths();
    }

    /**
     * Finds paths for provided agents. If sum of costs (SOC) for new paths is less than optimal costs * weight then
     * agent paths are set and method returns true. Else paths are discarded and method returns false.
     */
    private class findPath implements Command {

        /*
         * @param updateOriginalPaths If true, agent paths will be altered regardless of final sum of costs(SOC)
         * If false, agent paths will only be altered if SOC <= optimalCost * weight
         * @param findAgentGroup new paths for these agents ONLY will be found
         * @param avoidAgentGroup paths for these agents will be set as obstacles in the MAPF algorithm. new paths will NOT be found
         * @return true if SOC <= optimal costs * weight
        */
        public boolean execute(Boolean updateOriginalPaths, Integer findAgentGroup, Integer avoidAgentGroup){

            // clone map and set agent obstacles for avoidAgentGroup
            Map tempMap = map.clone();
            if ( avoidAgentGroup != -1 ) {
                InFlightUtils.convertAgentsToAgentObstacles( agentGroups.getGroupAgents( avoidAgentGroup ), tempMap );
            }

            // get findAgentGroup agents
            List<Agent> currentAgents = agentGroups.getGroupAgents( findAgentGroup );

            // if false, do not alter original paths
            List<Agent> agentsToFindPaths = ( updateOriginalPaths )
                    ? currentAgents
                    : InFlightUtils.cloneAgents( currentAgents );
            // find new paths
            Node root = new Node( null, null );

            // if single agent, run sapAlgorithm, else run mapfAlgorithm
            boolean algorithmSuccessful = runPathFindingAlgorithm( agentsToFindPaths, tempMap, root );
            if ( !algorithmSuccessful ) { return false; }

            // set cost variables
            double maxSOC = agentGroups.getGroupCost( findAgentGroup ) * weight;
            double newSOC = calculateSumOfCosts( agentsToFindPaths );

            // check if SOC <= optimal costs * weight
            if ( newSOC <= maxSOC && updateOriginalPaths ) {
                return true;
            } else if ( newSOC <= maxSOC ) {
                updateAgentPaths( agentsToFindPaths, currentAgents );
                return true;
            } else {
                return false;
            }
        }

        /**
         * execute the path finding algorithm (SAPF and/or MAPF)
         * @param agentsToFindPaths - agents sent to algorithm
         * @param tempMap - map used in search
         * @param root - root node of algorithm
         * @return true if successfully found path, false if no solution found
         */
        private boolean runPathFindingAlgorithm(List<Agent> agentsToFindPaths, Map tempMap, Node root){

            // run single agent path finding algorithm
            if ( agentsToFindPaths.size() == 1 ) {
                try { //TODO need to always find a solution
                    AlgorithmUtils.runAlgorithm( tempMap, agentsToFindPaths, root, sapfAlgorithm, sapfAlgorithm, DefaultParameters.sapftimelimit, initialTime, weight, new ArrayList<AlgorithmConflict>() );
                } catch ( SolutionNotFoundException | TimeoutException e ) {
                    return false;
                }
            }

            // run multi agent path finding algorithm
            else {
                List<Integer> tempIDs = new ArrayList<>();
                List<HashSet<Agent>> tempAST = new ArrayList<>(  );
                for (Agent agent : agentsToFindPaths){
                    tempIDs.add(agent.getId());
                    tempAST.add(agent.getAgentsInSharedTime());
                }
                try {
                    AgentUtils.resetAgentIds( agentsToFindPaths );
                    AgentUtils.setAgentsIndexes( agentsToFindPaths );
                    AgentUtils.setAgentsInSharedTime( agents );
                    AlgorithmUtils.runAlgorithm( tempMap, agentsToFindPaths, root, mapfAlgorithm, sapfAlgorithm, timeLimit, initialTime, weight, new ArrayList<AlgorithmConflict>() );

                    for (int i = 0; i < tempIDs.size(); i++){
                        agentsToFindPaths.get(i).setId( tempIDs.get(i) );
                        agentsToFindPaths.get(i).setIndex( tempIDs.get(i) );
                        agentsToFindPaths.get(i).setAgentsInSharedTime( tempAST.get( i ) );
                    }

                } catch ( SolutionNotFoundException | TimeoutException e ) {
                    for (int i = 0; i < tempIDs.size(); i++){
                        agentsToFindPaths.get(i).setId( tempIDs.get(i) );
                        agentsToFindPaths.get(i).setIndex( tempIDs.get(i) );
                        agentsToFindPaths.get(i).setAgentsInSharedTime( tempAST.get( i ) );
                    }
                    return false;
                }
            }
            return true;
        }

        /**
         * @return sum of costs for provided agents
         */
        private double calculateSumOfCosts(List<Agent> agents){
            double SOC = 0;
            for ( Agent agent : agents ) {
                SOC += agent.getPath().getCost();
            }
            return SOC;
        }

        public void updateConflicts(Integer group){
            IndependenceDetectionSearch.this.updateConflicts( group );
        }

        public void findConflicts(Integer group, Boolean findAllConflicts){
            IndependenceDetectionSearch.this.findConflicts( group, findAllConflicts );
        }
    }

    // update agent paths
    private void updateAgentPaths(List<Agent> newAgents, List<Agent> oldAgents){

        for ( Agent agentNew : newAgents ) {
            for ( Agent agentOld : oldAgents ){
                if ( agentOld.getId() == agentNew.getId() ) {
                    agentOld.setPath( agentNew.getPath().clone() );
                    break;
                }
            }
        }
    }

    // sets the initial costs of the agent paths
    public void setAgentPathCosts(){
        for ( Agent agent : agents ) {
            agent.getPath().setCost( GeometryUtils.getDistancePath( agent.getPath().getWaypoints() ) );
        }
    }

    /**
     * Removes all conflicts containing agent(s) in the provided group
     * Sets Agent path variable freeOfConflicts to true
     * @param agentGroup group of agents whose conflicts to remove
     */
    private void updateConflicts(Integer agentGroup){

        List<AlgorithmConflict> toRemove =  new ArrayList<>();
        for ( Integer agentId : agentGroups.getGroupAgentIds( agentGroup ) ) {
            for ( AlgorithmConflict conflict : conflicts ) {
                if ( conflict.getAgent1().getId() == (int)agentId ){
                    toRemove.add(conflict);
                    conflict.getAgent1().getPath().setFreeOfConflicts( true );
                } else if ( conflict instanceof AgentConflict && ( (AgentConflict) conflict ).getAgent2().getId() == (int)agentId ){
                    toRemove.add(conflict);
                    ( (AgentConflict) conflict ).getAgent2().getPath().setFreeOfConflicts( true );
                }
            }
        }
        conflicts.removeAll( toRemove );
    }

    private void findConflicts(Integer group, boolean findAllAgentConflicts){

        // Find conflicts among all agents or between a single group
        List<Agent> currentAgents = ( findAllAgentConflicts )
                ? agents
                : agentGroups.getGroupAgents( group );

        for ( Agent agentI : currentAgents ) {
            for ( Agent agentJ : agentI.getAgentsInSharedTime() ) {
                if ( agentJ.getIndex() >= agentI.getIndex() ) {
                    // check for conflicts
                    List<AlgorithmConflict> newConflicts = new ArrayList<>();
                    ConflictDetection.checkPathConflict( agentI.getPath(), agentJ.getPath(), operationAgentMap.get(agentI.getId()),
                            operationAgentMap.get(agentJ.getId()), false, newConflicts, true );
                    // if there are no conflicts, continue
                    if ( newConflicts.isEmpty() ) { continue; }
                    // add the pairs earliest conflicts to the conflicts priority queue
                    conflicts.addAll( newConflicts );
                }
            }
        }
    }

    public List<Agent> getAgents(){ return agents; }

    public DisjointSetMap getGroups(){ return agentGroups; }
}

/* Algorithm Outline

    // Agent groups data structure
    DisjointSet G;

    // find path for each group of agents in conflict
    for each agent a in conflict{
        find.path(a);
    }

    // find all conflicts between groups
    findAllConflicts(G);

    // enhancement - when solving conflicts among a group, find new paths for the group
    // of agents who have the largest number of alterations to their paths.
    // EX: g1 has three agents and none of those agents are new (they were not impacted by the emergency NFZ)
    // g2 has three agents and 2 are new (impacted by NFZ)
    // attempt to reroute group g2 first, then g1, then combine

    // order of execution
    let CBS fully execute. Compute the total cost based on agent path costs
    if the sum of costs is above the allowed figured, move onto next if statement.
    The final search will accept any cost as long as there's a path.


    // While conflicts between groups exists
    while (!conflicts.isEmpty()) do{
         if (g1 and g2 conflicted before)
            newCBS(g1 + g2);
         else if (newCBS(g1) < preDefinedCost)
            continue;
         else if (newCBS(g2) < preDefinedCost)
            continue;
         else
            newCBS(g1 + g2);
         findAllConflicts(G);
    }

 */
































