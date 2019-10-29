package algorithms.comparators;

import java.util.Comparator;

import algorithms.conflicts.AgentConflict;

public class ConflictComparator implements Comparator<AgentConflict> {

	@Override
	public int compare(AgentConflict conflictA, AgentConflict conflictB) {
		
		double t1 = conflictA.getTimestep();
		double t2 = conflictB.getTimestep();
		
		if (t1 < t2) {
			return -1;
		} 
		if (t1 > t2) {
			return 1;
		}
		if (conflictA.getAgent1().getId() < conflictB.getAgent1().getId()) {
			return -1;
		} 
		if (conflictA.getAgent1().getId() > conflictB.getAgent1().getId()) {
			return 1;
		}
		if (conflictA.getAgent2().getId() < conflictB.getAgent2().getId()) {
			return -1;
		} 
		if (conflictA.getAgent2().getId() > conflictB.getAgent2().getId()) {
			return 1;
		}
		return 0;
	}

}
