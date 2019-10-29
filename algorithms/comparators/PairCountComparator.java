package algorithms.comparators;

import algorithms.CBS.GreedyCBS.GreedyCBSState;
import algorithms.conflicts.AgentConflict;
import algorithms.trees.Node;

import java.util.Comparator;

public class PairCountComparator implements Comparator<Node> {

    @Override
    public int compare(Node o1, Node o2) {

        if (((GreedyCBSState)o1.getState()).getNumberPairs() < ((GreedyCBSState)o2.getState()).getNumberPairs()){
            return -1;
        }
        else if(((GreedyCBSState)o1.getState()).getNumberPairs()  == ((GreedyCBSState)o2.getState()).getNumberPairs() ) {
            return 0;
        }
        return 1;
    }
}
