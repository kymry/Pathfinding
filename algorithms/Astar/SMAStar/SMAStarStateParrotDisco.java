package algorithms.Astar.SMAStar;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.tan;

import java.util.ArrayList;
import java.util.List;

import agents.Agent;
import agents.FixedWings;
import agents.ParrotDisco;
import algorithms.trees.State;
import geometry.coords.AgentPosition;
import geometry.coords.AgentPositionFW;
import geometry.coords.PointInterface;
import geometry.coords.UTMCoord;
import map.Map;
import utils.GeometryUtils;
import utils.MapUtils;

public class SMAStarStateParrotDisco extends SMAStarStateFixedWings{
	
	public SMAStarStateParrotDisco(AgentPosition position, PointInterface coordinatesReal) {
		super(position, coordinatesReal);
	}
	
	public SMAStarStateParrotDisco(double gVal, double hVal, double fVal, AgentPosition position, PointInterface coordinatesReal) {
		super(gVal, hVal, fVal, position, coordinatesReal);
	}
	
	/* (non-Javadoc)
	 * @see algorithms.Astar.AStarState#expand(map.Map, agents.Agent)
	 * This method is implemented for FixedWings (ParrotDisco). If using different agents, create the necessary sub classes that extend this one, and implement their respective expand methods
	 */
	@Override
	public List<State> expand(Map map, Agent agent){
		int nbMotions = 5;
		List<State> result = new ArrayList<State>();
		PointInterface currentPosition = _coordinatesReal.clonePoint();
		double currentPhi = ((AgentPositionFW)_position).getPhi();
		List<AgentPositionFW> allMoves = new ArrayList<AgentPositionFW>();
		float metersPerVoxel = map.getMetersPerVoxel();
		
		/*
		 * Generation of motions for the fixed-wing -> 1st = straightforward on
		 * the same plane -> generation of n motions straightforward on upper
		 * planes -> generation of n motions straightforward on lower planes
		 * 
		 * -> generation of n motions on different right circles, same plane ->
		 * generation of n motions on different left circles, same plane
		 * 
		 * -> generation of n motions on different right circles, upper plane ->
		 * generation of n motions on different left circles, upper plane
		 * 
		 * -> generation of n motions on different right circles, lower plane ->
		 * generation of n motions on different left circles, lower plane
		 */
		
		// Straightforward on the same plane
		double nX = currentPosition.getX() + agent.getSpeed() * cos(currentPhi + PI / 2);
		double nY = currentPosition.getY() + agent.getSpeed() * sin(currentPhi + PI / 2);
		double nZ = currentPosition.getZ();
		double nPhi = currentPhi;
		
		UTMCoord newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());
		allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));
		// n Motions Straightforward on upper/lower planes
		double step_z = agent.getSpeed() * tan(((ParrotDisco)agent).getMuMax()) / nbMotions;
		
		for (int k = 1; k <= nbMotions; k++) {
			double coeff = step_z * k;
			nX = currentPosition.getX() + sqrt(pow(agent.getSpeed(), 2) - pow(coeff, 2)) * cos(currentPhi + PI / 2);
			nY = currentPosition.getY() + sqrt(pow(agent.getSpeed(), 2) - pow(coeff, 2)) * sin(currentPhi + PI / 2);
			nZ = currentPosition.getZ() + coeff;
			nPhi = currentPhi;
			nPhi = GeometryUtils.correctAngle(nPhi);
			newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

			allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

			nX = currentPosition.getX() + sqrt(pow(agent.getSpeed(), 2) - pow(coeff, 2)) * cos(currentPhi + PI / 2);
			nY = currentPosition.getY() + sqrt(pow(agent.getSpeed(), 2) - pow(coeff, 2)) * sin(currentPhi + PI / 2);
			nZ = currentPosition.getZ() - coeff;
			newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());
			allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

		}
		
		// n Motions right/left circles, same/upper/lower planes
		for (int k = 0; k < nbMotions; k++) {
			double radius = ((FixedWings)agent).getRho() + k * 10;

			// Same plane
			// CRi
			double x_Ri = currentPosition.getX() + radius * cos(currentPhi);
			double y_Ri = currentPosition.getY() + radius * sin(currentPhi);

			double theta = agent.getSpeed() / radius;
			nX = x_Ri - radius * sin(currentPhi - theta + PI / 2);
			nY = y_Ri + radius * cos(currentPhi - theta + PI / 2);
			nZ = currentPosition.getZ();
			nPhi = currentPhi - theta;
			nPhi = GeometryUtils.correctAngle(nPhi);
			newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

			allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

			// CLi
			double x_Li = currentPosition.getX() - radius * cos(currentPhi);
			double y_Li = currentPosition.getY() - radius * sin(currentPhi);

			nX = x_Li + radius * sin(currentPhi + theta + PI / 2);
			nY = y_Li - radius * cos(currentPhi + theta + PI / 2);
			nZ = currentPosition.getZ();
			nPhi = currentPhi + theta;
			nPhi = GeometryUtils.correctAngle(nPhi);
			newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

			allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

		}
		
		for (int i = 1; i <= nbMotions; i++) {
			for (int k = 0; k < nbMotions; k++) {

				double angle = i * ((ParrotDisco)agent).getMuMax() / nbMotions;
				double radius = ((FixedWings)agent).getRho() + k * 10;

				double x_Ri = currentPosition.getX() + radius * cos(currentPhi);
				double y_Ri = currentPosition.getY() + radius * sin(currentPhi);

				double x_Li = currentPosition.getX() - radius * cos(currentPhi);
				double y_Li = currentPosition.getY() - radius * sin(currentPhi);

				// Upper/Lower planes
				// CRi
				// Upper plane

				double b = tan(angle) * radius;
				double c = sqrt(pow(radius, 2) + pow(b, 2));
				double theta = agent.getSpeed() / c;

				nX = x_Ri - radius * sin(currentPhi - theta + PI / 2);
				nY = y_Ri + radius * cos(currentPhi - theta + PI / 2);
				nZ = currentPosition.getZ() + theta * b / c;
				nPhi = currentPhi - theta;
				nPhi = GeometryUtils.correctAngle(nPhi);
				newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

				allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

				// Lower plane
				nZ = currentPosition.getZ() - theta * b / c;
				newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

				allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

				// CLi
				// Upper plane
				nX = x_Li + radius * sin(currentPhi + theta + PI / 2);
				nY = y_Li - radius * cos(currentPhi + theta + PI / 2);
				nZ = currentPosition.getZ() + theta * b / c;
				nPhi = currentPhi + theta;
				nPhi = GeometryUtils.correctAngle(nPhi);
				newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

				allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));

				// Lower plane
				nZ = currentPosition.getZ() - theta * b / c;
				newRealCoords = new UTMCoord(nX, nY, nZ, ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());

				allMoves.add(new AgentPositionFW(newRealCoords, nPhi, newRealCoords));
			}
		}
		
		for (AgentPositionFW move : allMoves) {
			int Max_x = (int)map.getWidth();
			int Max_y = (int)map.getHeight();
			int Max_z = (int)map.getAltitude();

			if (move.getX() >= 0 && move.getX() <= Max_x * metersPerVoxel && move.getY() >= 0 && move.getY() < Max_y * metersPerVoxel
					&& move.getZ() >= 0 && move.getZ() < Max_z * metersPerVoxel) {
				boolean free = false;
				//TODO check conflicts
				if(free) {
					UTMCoord voxelCoords = (UTMCoord) MapUtils.convertRealCoordinatesToVoxel(_coordinatesReal, map.getMetersPerVoxel());
					
					UTMCoord realCoords = new UTMCoord(_coordinatesReal.getX(), _coordinatesReal.getY(), _coordinatesReal.getZ(), ((UTMCoord)_position).getZone(), ((UTMCoord)_position).getLatitudeBand());	
					AgentPositionFW index = new AgentPositionFW(voxelCoords, move.getPhi(), realCoords);
					State tState = new SMAStarStateParrotDisco(-1, -1, -1, index, realCoords);
					result.add(tState);
				}				
			}
		}
		return result;
	}

}
