package algorithms.heuristics;

import agents.Agent;
import algorithms.Astar.ODStar.ODStarState;
import algorithms.trees.State;
import geometry.coords.PointInterface;

public class DiagonalDistanceHeuristicSOC extends DiagonalDistanceHeuristic {

    /**
     * An extension of the diagonal distance heuristic to multi agents.
     * Uses the Sum of Cost heuristic (SOC)
     * Currently this is a very naive way of computing the SOC as it recalculates it for every agent at each state
     * TODO: only update the needed agent
     */

    @Override
    public float calcHeuristic( State currentNode, State goalNode) {
        float soc = 0;
        for ( Agent agent : ( (ODStarState) currentNode).getAgents() ){
            soc += getSomeOfCosts( agent.getPath().getWaypoints().get( agent.getPath().getWaypoints().size() - 1 ), agent.getGoal());
        }
        return soc;
    }

    private double getSomeOfCosts( PointInterface start, PointInterface goal ){
        return super.computeHeuristic(start, goal);
    }
}
