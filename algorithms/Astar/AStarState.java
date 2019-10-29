package algorithms.Astar;

import java.util.ArrayList;
import java.util.List;

import agents.Agent;
import agents.path.PathLine;
import algorithms.trees.State;
import geometry.coords.AgentPosition;
import geometry.coords.PointInterface;
import geometry.coords.UTMCoord;
import map.Map;
import utils.AgentUtils;
import utils.GeometryUtils;
import utils.Utils;

public class AStarState extends State {
	protected double _timestep;
	protected boolean _waitMove;
	protected PointInterface _position;
	
	public AStarState() {
		super();
		_timestep = 0;
		_position = null;
	}
	
	public AStarState(double gVal, double hVal, double fVal, AgentPosition position) {
		super(gVal, hVal, fVal);
		_timestep = position.getEta();
		_position = position;
	}
	
	public AStarState(AgentPosition position) {
		super(-1, -1, -1);
		_timestep = position.getEta();
		_position = position;
	}
	
	public List<State> expand(Map map, Agent agent){
		List<State> neighborStates = new ArrayList<State>();
		//first get all neighbor positions
		PointInterface goalPos = Utils.convertPositionToVoxel((UTMCoord)agent.getGoal());
		List<PointInterface> neighborPositions = getPossibleNeighborPositions(map, agent.getRadius(), goalPos);
		//then check which ones are valid
		for(PointInterface neighbor : neighborPositions) {
			AgentUtils.updateTimestep((AgentPosition)neighbor, (AgentPosition)_position, agent.getSpeed());
			PathLine line = new PathLine(_position, neighbor, agent);
			if(map.isMoveValid(line, goalPos)) {
				//create and add the new state
				AStarState newState = new AStarState((AgentPosition)neighbor);
				newState._waitMove = ((AgentPosition)neighbor).isWait();
				neighborStates.add(newState);
			}
		}
		return neighborStates;
	}
	
	@Override
	public double calculateDistance(State state) {
		return GeometryUtils.getDistanceBetweenPoints3D(_position, ((AStarState)state).getPosition());
	}
	
	@Override
	public boolean  equals(Object o) {
		AStarState other = (AStarState)o;
		AgentPosition otherPosition = (AgentPosition)other.getPosition();
		boolean samePosition = _position.equals(new UTMCoord((int)otherPosition.getX(), (int)otherPosition.getY(), (int)otherPosition.getZ(), otherPosition.getZone(), otherPosition.getLatitudeBand()));
//		boolean sameTimeStep = false;
//		if(samePosition) {
//			sameTimeStep = Utils.valueInBetween(((AgentPosition)_position).getEta(), otherPosition.getEta() - 1, otherPosition.getEta() + 1);
//		}
		return samePosition /*&& sameTimeStep*/;
	}
	
	@Override
	public int hashCode() {
		return _position.hashCode();
	}
	
	@Override
	public State clone() {
		return new AStarState(_gVal, _hVal, _fVal, (AgentPosition)_position.clonePoint());
	}
	
	@Override
	public int compareTo(State o) {
		AStarState other = (AStarState)o;
		if(_fVal < other._fVal) {
			return -1;
		}
		if(_fVal > other._fVal) {
			return 1;
		}
		return 0;
	}
	
	@Override
	public String toString() {
		return _position.toString() + "\n" + super.toString();
	}
	
	@Override
	public double getLowerBound() {
		return _fVal;
	}

	
	/**
	 * @return all the voxels around the state position that are inside the map. Does not check for any kind of obstacles
	 */
	public List<PointInterface> getPossibleNeighborPositions(Map map, double agentRadius, PointInterface goal){
		List<PointInterface> positions = new ArrayList<PointInterface>();
		for(int x = -1; x <= 1; x++) {
			for(int y = -1; y <= 1; y++) {
				for(int z = -1; z <= 1; z++) {
					AgentPosition position = new AgentPosition((int)_position.getX() + x, (int)_position.getY() + y, (int)_position.getZ() + z, -1, -1, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());
					if(map.inMap(position, goal, agentRadius)) {
						positions.add(position);
					}
				}
			}
		}
		return positions;
	}
	
	public double getTimestep() {
		return _timestep;
	}

	public void setTimestep(double _timestep) {
		this._timestep = _timestep;
	}

	public PointInterface getPosition() {
		return _position;
	}
	
	public void setPosition(PointInterface position) {
		_position = position;
	}

}
