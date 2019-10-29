package algorithms.conflicts;

import agents.Agent;
import agents.path.PathLine;
import operations.Operation;

public class AgentConflict extends AlgorithmConflict {

	private Agent _agent2;
	private PathLine _location2;
	
	public AgentConflict(Agent agent1, Agent agent2, int timestep, PathLine location1, PathLine location2, Operation operation1, Operation operation2, boolean solvable) {
		super(agent1, timestep, location1, ConflictType.AGENT, operation1, operation2, solvable);
		_agent2 = agent2;
		_location2 = location2;
	}
	
	@Override
	public String toString() {
		return "Timestep: " + _timestep + "; Agent 1: " + _agent1 + " Location: " + _location1.toString() + "; Agent 2: " + _agent2 + " Location: " + _location2.toString();
	}
	
	public Agent getAgent2() {
		return _agent2;
	}

	public PathLine getLocation2() {
		return _location2;
	}

	public Operation getOperation2() {
		return _operation2;
	}

	public void setOperation2(Operation operation2) {
		this._operation2 = operation2;
	}

}
