package algorithms.conflicts;

import agents.Agent;
import agents.path.PathLine;
import operations.Operation;

public class ObstacleConflict extends AlgorithmConflict {
	
	public ObstacleConflict(Agent agent1, Operation operation1, Operation operation2, int timestep, PathLine location1, boolean solvable) {
		super(agent1, timestep, location1, ConflictType.OBSTACLE, operation1, operation2, solvable);
	}

}
