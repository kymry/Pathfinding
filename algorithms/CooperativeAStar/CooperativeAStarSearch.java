package algorithms.CooperativeAStar;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import agents.Agent;
import agents.path.Path;
import algorithms.AlgorithmName;
import algorithms.Search;
import algorithms.conflicts.AgentConflict;
import algorithms.conflicts.AlgorithmConflict;
import algorithms.conflicts.ConflictType;
import algorithms.trees.Node;
import exceptions.SolutionNotFoundException;
import exceptions.TimeoutException;
import main.ConflictDetection;
import map.Map;
import utils.AlgorithmUtils;
import utils.Debug;
import utils.DebugEnums;
import utils.OperationUtils;
import utils.TestUtils;
import utils.Utils;

public class CooperativeAStarSearch extends Search{
	
	private Map _map;	
	private AlgorithmName _singleSearch;
	private List<Agent> _agents;
	private List<AgentConflict> _conflicts;
	private List<Path> _acceptedPaths;

	/**
	 * @param map : safer if a clone
	 * @param agents
	 * @param singleSearch : Focal A* can also be used
	 * @param conflicts that were detected before the search started
	 */
	public CooperativeAStarSearch(Map map, List<Agent> agents,  AlgorithmName singleSearch, List<AlgorithmConflict> conflicts) {
		_map = map;
		_agents = agents;
		_singleSearch = singleSearch;
		_noSolution = false;
		_acceptedPaths = new ArrayList<Path>();
		_conflicts = new ArrayList<AgentConflict>();
		for(AlgorithmConflict conflict : conflicts) {
			if(conflict.getType() == ConflictType.AGENT) {
				_conflicts.add((AgentConflict)conflict);
			}
		}
	}
	
	@Override
	public boolean findPath(long timeLimit, long initialTime){
		final ExecutorService service = Executors.newSingleThreadExecutor();
		try {
			service.submit(() -> {
				try {
					search(timeLimit);
				}
				catch(Exception e) {
					TestUtils.printExceptionMessage(e.getMessage(), "");
					_noSolution = true;
				}	
				service.shutdownNow();
			});  
			service.awaitTermination(timeLimit, TimeUnit.MILLISECONDS);
		}
		catch (InterruptedException e) {
			Debug.print(DebugEnums.ERROR, "TIMEOUT " + e.getMessage());
			return true;
		}
		finally {
            service.shutdown();
        }
		return false;
	}
	
	public void search(long timeLimit) {
		for(Agent agent : _agents) {
			//check if the agent has conflicts
			if(conflictFound(agent.getId())) {
				//if so,
				//create root node
				Node root = AlgorithmUtils.getRootNode(_singleSearch);
				List<Agent> agents = new ArrayList<Agent>();
				agents.add(agent);
				try {
					//find new path
					AlgorithmUtils.runAlgorithm(_map, agents, root, _singleSearch, _singleSearch, timeLimit, _initialTime, 0, null);
					agent.getPath().setFreeOfConflicts(true);
				} catch (SolutionNotFoundException e) {
					//inform that one of the agents does not have a solution but continue the search
					Debug.print(DebugEnums.DEBUG, "Cooperative A*: One of the agents could not find a path");
					agent.getPath().setFreeOfConflicts(false);
					continue;
				} catch (TimeoutException e) {
					// Not expecting timeouts here
					e.printStackTrace();
				}
			}
			
			//add path to the map and list of accepted paths
			OperationUtils.addAgentObstacleFromPart(_map.getAgentObstacles(), agent.getPath());
			_acceptedPaths.add(agent.getPath());
		}
	}
	
	public boolean conflictFound(int agentId) {
		//check if conflict detection had previously found a conflict
		if(Utils.containsAgentConflicts(_conflicts, agentId)) {
			return true;
		}
		
		//if not, check if it has a new conflict with an agent already fixed by CA*
		for(Path pathJ : _acceptedPaths) {
			List<AlgorithmConflict> conflicts = new ArrayList<AlgorithmConflict>();
			ConflictDetection.checkPathConflict(Utils.getAgentFromID(_agents, agentId).getPath(), pathJ, null, null, false, conflicts, true);
			// if there are conflicts, return true
			if (!conflicts.isEmpty()) {
				return true;
			}
		}		
		return false;
	}
}
