package algorithms.comparators;

import algorithms.CBS.GreedyCBS.GreedyCBSState;
import algorithms.trees.Node;

import java.util.Comparator;

public class AgentConflictComparator implements Comparator<Node> {

    @Override
    public int compare( Node o1, Node o2) {

        if (((GreedyCBSState)o1.getState()).getNumberAgentsConflicts() < ((GreedyCBSState)o2.getState()).getNumberAgentsConflicts()){
            return -1;
        }
        else if(((GreedyCBSState)o1.getState()).getNumberAgentsConflicts()  == ((GreedyCBSState)o2.getState()).getNumberAgentsConflicts() ) {
            return 0;
        }
        return 1;
    }
}
