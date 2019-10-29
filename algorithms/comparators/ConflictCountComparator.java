package algorithms.comparators;

import algorithms.CBS.GreedyCBS.GreedyCBSState;
import algorithms.trees.Node;

import java.util.Comparator;

public class ConflictCountComparator implements Comparator<Node> {

    @Override
    public int compare(Node o1, Node o2) {

        if (((GreedyCBSState)o1.getState()).getNumberConflicts() < ((GreedyCBSState)o2.getState()).getNumberConflicts()){
            return -1;
        }
        else if(((GreedyCBSState)o1.getState()).getNumberConflicts()  == ((GreedyCBSState)o2.getState()).getNumberConflicts() ) {
            return 0;
        }
        return 1;
    }

}
