package algorithms.heuristics;

import algorithms.Astar.AStarState;
import algorithms.trees.State;
import geometry.coords.PointInterface;
import geometry.coords.UTMCoord;
import utils.GeometryUtils;
import utils.GeneralUtils;

public class DiagonalDistanceHeuristic implements HeuristicInterface{

	@Override
	public float calcHeuristic(State currentNode, State goalNode) {
		PointInterface start = ((AStarState)currentNode).getPosition();
		PointInterface goal = ((AStarState)goalNode).getPosition();
		
		return computeHeuristic(start, goal);
			
	}
	
	public float computeHeuristic(PointInterface start, PointInterface goal) {
		float weight = 1.01f;

		//if current node is the root and the position of the start is not centered in the voxel, the computation must be different. The same happens if the goal position is not centered in the voxel
		//to keep the heuristic admissible, select the shortest distance between the goal and the current voxel the agent is and the neighbor voxels
		boolean integerStart = GeneralUtils.isPositionInteger(start);
		boolean integerGoal = GeneralUtils.isPositionInteger(goal);
		if(integerStart && integerGoal) {
			return weight * GeometryUtils.getDiagonalDistance(start, goal);
		}
		
		//TODO check if there is a more efficient way to return an admissible heuristic value
		float minDistance = Float.MAX_VALUE;
		
		if(!integerStart && integerGoal) {
			for(int i = -1; i <= 1; i++) {
				for(int j =-1; j <= 1; j++) {
					for(int k = -1; k <= 1; k++) {
						PointInterface voxel = new UTMCoord((int)start.getX() + i, (int)start.getY() + j, (int)start.getZ() + k, ((UTMCoord)start).getZone(), ((UTMCoord)start).getLatitudeBand());
						minDistance = Math.min(minDistance, GeometryUtils.getDiagonalDistance(voxel, goal) + (float)GeometryUtils.getDistanceBetweenPoints3D(start, voxel));
					}				
				}
			}
			return weight * minDistance;
		}
		
		//the goal voxel can only be one. the objective is always reaching that voxel.
		if(integerStart && !integerGoal) {
			PointInterface voxel = new UTMCoord((int)goal.getX(), (int)goal.getY(), (int)goal.getZ(), ((UTMCoord)goal).getZone(), ((UTMCoord)goal).getLatitudeBand());
			return weight * GeometryUtils.getDiagonalDistance(start, voxel);
		}
		
		for(int i = -1; i <= 1; i++) {
			for(int j =-1; j <= 1; j++) {
				for(int k = -1; k <= 1; k++) {
					PointInterface startVoxel = new UTMCoord((int)start.getX() + i, (int)start.getY() + j, (int)start.getZ() + k, ((UTMCoord)start).getZone(), ((UTMCoord)start).getLatitudeBand());
					PointInterface goalVoxel = new UTMCoord((int)goal.getX(), (int)goal.getY(), (int)goal.getZ(), ((UTMCoord)goal).getZone(), ((UTMCoord)goal).getLatitudeBand());
					minDistance = Math.min(minDistance, GeometryUtils.getDiagonalDistance(startVoxel, goalVoxel) + (float)GeometryUtils.getDistanceBetweenPoints3D(start, startVoxel));
				}				
			}
		}
		
		return weight * minDistance;
	}

}
