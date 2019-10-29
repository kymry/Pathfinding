package algorithms.comparators;

import algorithms.trees.Node;

import java.util.Comparator;

public class WeightedFValueComparator implements Comparator<Node> {

    double weight;

    public WeightedFValueComparator( double weight){
        this.weight = weight;
    }

    @Override
    public int compare(Node o1, Node o2) {
        if (o1.getState().getGVal() + o1.getState().getHVal() * weight <
                o2.getState().getGVal() + o2.getState().getHVal() * weight ){
            return -1;
        }
        else if(o1.getState().getGVal() +  o1.getState().getHVal() * weight ==
                o2.getState().getGVal() + o2.getState().getHVal() * weight) {

            if (o1.getState().getHVal() * weight > o2.getState().getHVal() * weight){
                return -1;
            }
            else if(o1.getState().getHVal() * weight == o2.getState().getHVal() * weight) {
                return 0;
            }
            return 1;
        }
        return 1;
    }
}
