package algorithms.CBS.GreedyCBS;

import agents.Agent;
import agents.path.PathLine;
import algorithms.AlgorithmName;
import algorithms.CBS.CBSSearch;
import algorithms.CBS.CBSState;
import algorithms.conflicts.AgentConflict;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import map.Map;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

public class GreedyCBSSearch extends CBSSearch {

    public GreedyCBSSearch( Map map, List<Agent> agents, Node root, HeuristicInterface heuristic, AlgorithmName singleSearch){
        super(map, agents, root, heuristic, singleSearch);
    }

    @Override
    public State getNewState( float cost, HashMap<Integer, HashSet<PathLine>> agentObstacles, PriorityQueue<AgentConflict> conflictsStart) {
        HashMap<Agent, HashSet<Agent>> pairConflicts = new HashMap<>();
        State startState = new GreedyCBSState(-1, Double.MAX_VALUE, -1, cost, agentObstacles, _singleSearch,
                conflictsStart, null, _heuristic, pairConflicts);
        return startState;
    }
}
