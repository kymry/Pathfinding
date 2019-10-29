package algorithms.comparators;

import java.util.Comparator;

import algorithms.trees.Node;

public class FValueComparator implements Comparator<Node> {
	
	@Override
	public int compare(Node o1, Node o2) {
		if (o1.getState().getFVal() < o2.getState().getFVal()){
			return -1;
		}
		else if(o1.getState().getFVal() == o2.getState().getFVal()) {
			if (o1.getState().getGVal() > o2.getState().getGVal()){
				return -1;
			}
			else if(o1.getState().getGVal() == o2.getState().getGVal()) {
				return 0;
			}
			return 1;
		}
		return 1;
	}

}
