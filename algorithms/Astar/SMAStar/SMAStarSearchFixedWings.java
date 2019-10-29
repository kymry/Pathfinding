package algorithms.Astar.SMAStar;

import agents.Agent;
import algorithms.heuristics.HeuristicInterface;
import algorithms.trees.Node;
import algorithms.trees.State;
import geometry.coords.AgentPositionFW;
import map.Map;

public class SMAStarSearchFixedWings extends SMAStarSearch {

	public SMAStarSearchFixedWings(Map map, Agent agent, Node root, HeuristicInterface heuristic) {
		super(map, agent, root, heuristic);
	}
	
	@Override
	public State createStartStatePosition() {
		AgentPositionFW agentStart = (AgentPositionFW) _agent.getStart();
		return new SMAStarStateFixedWings(0, -1, -1, agentStart, agentStart.getCoordinatesReal());
	}
	
	@Override	
	public void createGoalState() {
		AgentPositionFW goal = (AgentPositionFW)_agent.getGoal();
		AgentPositionFW agentGoal = new AgentPositionFW(goal, goal.getZone(), goal.getLatitudeBand(), goal.getPhi(), goal.getCoordinatesReal());
		switch(_agent.getType()) {
		case PARROTDISCO:
			_goal = new SMAStarStateParrotDisco(agentGoal, goal.getCoordinatesReal());
		default:
			_goal = new SMAStarStateFixedWings(agentGoal, goal.getCoordinatesReal());
		}
		 
	}

}
