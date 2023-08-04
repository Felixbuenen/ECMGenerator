#include "ECMPathPlanner.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECM.h"

namespace ECM {
	namespace PathPlanning {

		Path ECMPathPlanner::GetPath(const Environment& environment, Point start, Point goal, float clearance)
		{
			// TODO:
			// First check if start and goal are valid positions.
			Path result;

			// an ECM path is planned as follows:
			// 1. query the cell location of the start / goal position.
			auto ecmStart = environment.QueryECM(start);
			auto ecmGoal = environment.QueryECM(goal);
			
			// for now we require that we only have 1 ECM in our environment
			if (ecmStart != ecmGoal)
			{
				return result;
			}

			auto ecm = ecmStart;
			auto startCell = ecm->GetECMGraph().GetCell(start.x, start.y);
			auto goalCell = ecm->GetECMGraph().GetCell(goal.x, goal.y);

			if (!startCell || !goalCell)
			{
				printf("PathPlanning error: Could not find the cell of start and/or goal position.\n");
				return result;
			}
			
			// 2. retract the start / goal position on the ECM graph
			int startEdgeIdx = startCell->ecmEdge;
			int goalEdgeIdx = goalCell->ecmEdge;

			// if start and goal in the same corridor, simply return a straight line path
			if (startEdgeIdx == goalEdgeIdx)
			{
				result.push_back(start);
				result.push_back(goal);

				return result;
			}

			Point retrStart, retrGoal;
			ECMEdge startEdge, goalEdge;
			if (!ecm->RetractPoint(start, *startCell, retrStart, startEdge))
			{
				printf("retraction failed");
			}
			if (!ecm->RetractPoint(goal, *goalCell, retrGoal, goalEdge))
			{
				printf("retraction failed");
			}

			// DEBUG
			result.push_back(start);
			result.push_back(retrStart);
			result.push_back(retrGoal);
			result.push_back(goal);

			// 3. Plan a path on the medial axis using the clearance and A*.
			//    > now we have a path from start to goal over the medial axis
			// 4. Triangulate the ECM cells through which the path goes. Shrink these triangles using "clearance"
			// 5. Use the funnel algorithm to generate a path over these triangles
			// 6. smooth path.
			// -- done.

			return result;
		}

	}
}
