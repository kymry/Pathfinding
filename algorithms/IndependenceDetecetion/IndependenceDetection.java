package algorithms.IndependenceDetecetion;

import agents.Agent;
import algorithms.conflicts.AgentConflict;
import algorithms.conflicts.AlgorithmConflict;
import dataStructures.DisjointSetMap;
import map.Map;

import java.util.ArrayList;
import java.util.List;

/**
 * General framework for independence detection that can be used by any low level or high level search algorithm
 * Designed around the "command pattern". This allows any type of findPath() method to be used with the framework
 */
public class IndependenceDetection {

    private List<AlgorithmConflict> conflicts;
    private DisjointSetMap agentGroups = new DisjointSetMap();
    private Command pathFindingDriver;

    public IndependenceDetection(List<AlgorithmConflict> conflicts, List<Agent> agents, DisjointSetMap agentGroups){
        this.conflicts = conflicts;
        this.agentGroups = agentGroups;
    }

    /**
     * @param command method to be used for finding paths
     */
    public void setFindPathMethod(Command command){
        pathFindingDriver = command;
    }

    /**
     * finds conflict free paths for all agents using independence detection
     */
    public void findPaths(){

        // Perform the iterative search until all conflicts are resolved
        while ( !conflicts.isEmpty() )
        {
            AlgorithmConflict conflict =  conflicts.get(0);
            Integer groupI, groupJ;
            groupI = agentGroups.find( conflict.getAgent1().getId() );
            if ( !conflict.isSolvable() ) {
                groupI = agentGroups.find( conflict.getAgent1().getId() );
                pathFindingDriver.updateConflicts( groupI );
            }
            else if (conflict.getOperation2() == null) {
                groupI = agentGroups.find( conflict.getAgent1().getId() );
                pathFindingDriver.execute( true, groupI, -1 );
                pathFindingDriver.updateConflicts( groupI );
                pathFindingDriver.findConflicts( groupI, false );
            }
            else {
                try {
                    groupI = agentGroups.find( conflict.getAgent1().getId() );
                    groupJ = agentGroups.find( ( (AgentConflict) conflict ).getAgent2().getId() );

                    // solve conflicts and find new paths
                    if ( groupI.equals( groupJ ) ) {
                        pathFindingDriver.execute( true, groupI, -1 );
                    } else if ( pathFindingDriver.execute( false, groupI, groupJ ) ) {
                        agentGroups.union( groupI, groupJ );
                    } else if ( pathFindingDriver.execute( false, groupJ, groupI ) ) {
                        agentGroups.union( groupI, groupJ );
                    } else {
                        agentGroups.union( groupI, groupJ );
                        groupJ = agentGroups.find(groupJ);
                        pathFindingDriver.execute( true, groupJ, -1 );
                    }

                    // remove previous conflicts with rerouted agents and find new conflicts (groupI and groupJ are now same group)
                    pathFindingDriver.updateConflicts( groupI );
                    groupI = agentGroups.find(groupI);
                    pathFindingDriver.findConflicts( groupI, false );
                } catch ( NullPointerException e ) {
                    pathFindingDriver.updateConflicts( groupI );
                    System.out.println( e );
                }
            }
        }
    }
}
/*
    while (!conflicts.isEmpty()) do
            if (g1 and g2 conflicted before)
            findPaths(g1 + g2);
            else if (findPaths(g1).getCost() < preDefinedCost)
        continue;
        else if (findPaths(g2).getCost() < preDefinedCost)
        continue;
        else
        findPaths(g1 + g2);
        findAllConflicts(G);
*/