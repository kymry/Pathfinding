package algorithms.conflicts;

import agents.Agent;
import agents.path.PathLine;
import operations.Operation;

public class AlgorithmConflict {
	protected Agent _agent1;
	protected int _timestep;
	protected PathLine _location1;
	protected ConflictType _type;	
	protected Operation _operation1;
	protected Operation _operation2;
	protected boolean _solvable;
	
	public AlgorithmConflict(Agent agent1, int timestep, PathLine location1, ConflictType type, Operation operation1, Operation operation2, boolean solvable) {
		_agent1 = agent1;
		_timestep = timestep;
		_type = type;
		_location1 = location1;
		_operation1 = operation1;
		_operation2 = operation2;
		_solvable = solvable;
	}
	
	@Override
	public String toString() {
		return "Timestep: " + _timestep + "; Agent 1: " + _agent1.toString() + " Location: " + _location1.toString();
	}

	public Agent getAgent1() {
		return _agent1;
	}

	public double getTimestep() {
		return _timestep;
	}
	
	public PathLine getLocation1() {
		return _location1;
	}

	public ConflictType getType() {
		return _type;
	}

	public Operation getOperation1() {
		return _operation1;
	}

	public void setOperation1(Operation operation) {
		this._operation1 = operation;
	}
	
	public Operation getOperation2() {
		return _operation2;
	}

	public void setOperation2(Operation operation) {
		this._operation2 = operation;
	}

	public boolean isSolvable() {
		return _solvable;
	}

	public void setSolvable(boolean solvable) {
		this._solvable = solvable;
	}

}
