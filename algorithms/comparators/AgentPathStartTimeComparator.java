package algorithms.comparators;

import agents.Agent;
import java.util.Comparator;

public class AgentPathStartTimeComparator implements Comparator<Agent> {

    /* Not using 0 for equal values because left value is arbitrarily chosen */
    @Override
    public int compare(Agent a, Agent b){

        double t1;
        double t2;

        t1 = a.getPath().getStartTimestep();
        t2 = a.getPath().getStartTimestep();

        if ( t1 < t2 ){
            return 1;
        } else if ( t1 > t2 ){
            return -1;
        } else {
            return 1;
        }
    }

}
