package algorithms.comparators;

import java.util.Comparator;

import algorithms.trees.Node;

public class HValueComparator implements Comparator<Node> {

	@Override
	public int compare(Node o1, Node o2) {
		if (o1.getState().getHVal() < o2.getState().getHVal()){
			return -1;
		}
		else if(o1.getState().getHVal() == o2.getState().getHVal()) {
			return 0;
		}
		return 1;
	}

}
