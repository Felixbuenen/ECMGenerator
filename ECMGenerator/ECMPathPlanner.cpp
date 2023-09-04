#include "ECMPathPlanner.h"

#include "ECMDataTypes.h"
#include "Environment.h"
#include "ECM.h"
#include "AStar.h"

namespace ECM {
	namespace PathPlanning {

		ECMPathPlanner::ECMPathPlanner() { }

		ECMPathPlanner::~ECMPathPlanner()
		{
			delete m_AStar;
		}

		bool ECMPathPlanner::Initialize(ECMGraph& graph)
		{
			m_AStar = new AStar(graph);
			m_AStar->Initialize();

			return true;
		}


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
			auto startCell = ecm->GetECMGraph().FindCell(start.x, start.y);
			auto goalCell = ecm->GetECMGraph().FindCell(goal.x, goal.y);

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
			ECMEdge startEdge;
			ECMEdge goalEdge;
			if (!ecm->RetractPoint(start, *startCell, retrStart, startEdge))
			{
				printf("retraction for start point failed");
			}
			if (!ecm->RetractPoint(goal, *goalCell, retrGoal, goalEdge))
			{
				printf("retraction for end point failed");
			}

			// DEBUG

			// TODO: currently de astar path returns a list of ECM vertices. However, this does not make sense. We want to return ECM edges, because only then
			// do we have the right left/right closest obstacle information. This requires us to adapt the astar pathfinding algorithm.
			std::vector<int> astarPath;
			if(!m_AStar->FindPath(retrStart, retrGoal, &startEdge, &goalEdge, 1.0f, astarPath))
			{
				printf("couldn't find path!\n");
			}

			// DEBUG
			result.push_back(start);
			result.push_back(retrStart);
			for (int i = 0; i < astarPath.size(); i++)
			{
				const auto& vert = ecm->GetECMGraph().GetVertex(astarPath[i]);
				result.push_back(vert->position);
			}
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

		void ECMPathPlanner::CreateCorridor(const std::vector<int>& maPath, Corridor& outCorridor, std::shared_ptr<ECM> ecm)
		{
			for (int i = 0; i < maPath.size(); i++)
			{
				const auto& vert = ecm->GetECMGraph().GetVertex(maPath[i]);

				outCorridor.diskCenters.push_back(vert->position);
				outCorridor.diskRadii.push_back(vert->clearance);
			}
		}

	}
}
