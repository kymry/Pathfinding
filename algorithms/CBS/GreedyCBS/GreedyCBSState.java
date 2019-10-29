package algorithms.CBS.GreedyCBS;

import agents.Agent;
import agents.path.Path;
import agents.path.PathLine;
import algorithms.AlgorithmName;
import algorithms.CBS.CBSState;
import algorithms.conflicts.AgentConflict;
import algorithms.conflicts.AlgorithmConflict;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.State;
import main.ConflictDetection;
import org.apache.commons.math3.util.Pair;
import utils.AgentUtils;
import utils.Utils;

import java.util.*;

public class GreedyCBSState extends CBSState {

    protected List<AgentConflict> allConflicts = new ArrayList<>();
    protected HashMap<Agent, HashSet<Agent>> pairConflicts;
    protected int totalConflicts;
    protected double totalPairs;

    public GreedyCBSState( double gVal, double hVal, double fVal, float totalCost, HashMap<Integer, HashSet< PathLine >> agentObstacles,
                     AlgorithmName singleSearch, PriorityQueue< AgentConflict > conflicts, Agent agent, HeuristicInterface
                            heuristic, HashMap<Agent, HashSet<Agent>> pairConflicts){
            super(gVal, hVal, fVal, totalCost, agentObstacles, singleSearch, conflicts, agent, heuristic);
            this.pairConflicts = pairConflicts;
    }

    @Override
    public GreedyCBSState getNewState(HashMap<Integer, HashSet<PathLine>> agentObstacles, PriorityQueue<AgentConflict> conflicts, Agent agent) {
        GreedyCBSState state = new GreedyCBSState(_gVal, Double.MAX_VALUE, _fVal, Float.MAX_VALUE, agentObstacles,
                _singleSearch, conflicts, agent, _heuristic, pairConflicts);
        state.setAllSingleAgentPaths( AgentUtils.clonePaths(_allSingleAgentPaths));
        return state;
    }

    @Override
    public void otherUpdateds() {

        // get total conflicts
        totalConflicts = allConflicts.size();

        // get total pairs of conflicts
        totalPairs = 0;
        for (HashSet<Agent> x : pairConflicts.values()){
            totalPairs += x.size();
        }
        totalPairs /= 2;
    }

    @Override
    public void findAgentConflict() {
        //remove conflicts involving the agent
        Utils.removeAgentFromConflicts(_conflicts, _agent.getId());
        findConflict(_agent, 0);
        //update heuristic value;
        updateHeuristicValue();
        _earliestConflict = _conflicts.poll();

    }

    @Override
    public void findConflict(Agent agentI, int index) {

        // remove conflicts from allConflicts
        Iterator<AgentConflict> iter = allConflicts.iterator();
        while (iter.hasNext()) {
            AgentConflict conflict = iter.next();
            if (conflict.getAgent1().getId() == _agent.getId() || conflict.getAgent2().getId() == _agent.getId()) {
                iter.remove();
            }
        }

        // remove agents from pairsConflicts
        if ( pairConflicts.containsKey( agentI ) ){
            for (Agent a : pairConflicts.get(agentI)){
                pairConflicts.get(a).remove(agentI);
            }
            pairConflicts.remove(agentI);
        }

        // for each pair of current agent and the other agents paths that belong to the same path time interval
        for(Agent agentJ : agentI.getAgentsInSharedTime()) {
            //avoid repetition
            if(agentJ.getIndex() >= index) {
                Path pathJ = agentJ.getPath();
                // check for conflicts
                List<AlgorithmConflict> conflicts = new ArrayList<AlgorithmConflict>();
                ConflictDetection.checkPathConflict(_allSingleAgentPaths.get(agentI.getIndex()), pathJ, null, null, false, conflicts, false);
                // if there are no conflicts, continue
                if (conflicts.isEmpty()) {
                    continue;
                }
                // add the pairs earliest conflicts to the conflicts priority queue
                _conflicts.add((AgentConflict) conflicts.get(0));

                // add all conflicts to allConflicts
                for (AlgorithmConflict c : conflicts){
                    allConflicts.add((AgentConflict)c);
                }

                // add agent I to pair
                if ( pairConflicts.containsKey( agentI ) ){
                } else {
                    pairConflicts.put(agentI, new HashSet<>());
                }
                pairConflicts.get(agentI).add(agentJ);

                // add agent J to pair
                if ( pairConflicts.containsKey( agentJ ) ){
                } else {
                    pairConflicts.put(agentJ, new HashSet<>());
                }
                pairConflicts.get(agentJ).add(agentI);
            }
        }
    }

    public double getNumberPairs(){ return totalPairs; }

    public double getNumberConflicts(){ return totalConflicts; }

    public double getNumberAgentsConflicts(){ return getConflicts().size(); }
}
