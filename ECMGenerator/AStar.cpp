#include "AStar.h"

#include "ECM.h"
#include "UtilityFunctions.h"
#include "Timer.h"

namespace ECM {

	bool AStar::Initialize()
	{
		// initialize astar node array
		const auto& ecmVerts = m_Graph.GetVertices();
		int numVerts = ecmVerts.size();
		m_Nodes.resize(numVerts);
		m_Visited.resize(numVerts);
		m_NodeClearance.resize(numVerts);

		INVALID_NODE_INDEX = numVerts;
		
		for (int i = 0; i < numVerts; i++)
		{
			int index = ecmVerts[i].idx;
			m_Nodes[index].index = index;
			m_Nodes[index].gCost = Utility::MAX_FLOAT;
			m_Nodes[index].fCost = Utility::MAX_FLOAT;
			m_Nodes[index].parentIndex = INVALID_NODE_INDEX;

			m_NodeClearance[index] = ecmVerts[i].clearance;
		}
		
		return true;
	}

	bool AStar::FindPath(const Point& startLocation, const Point& goalLocation, const ECMEdge* startEdge, const ECMEdge* goalEdge, float clearance, std::vector<int>& outPath)
	{
		//Timer timer("AStar::FindPath");

		// create the list of open nodes as a priority queue
		std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarCompare> openList;
		
		// init first nodes
		const ECMVertex* startVertA = m_Graph.GetVertex(startEdge->half_edges[0].v_target_idx);
		const ECMVertex* startVertB = m_Graph.GetVertex(startEdge->half_edges[1].v_target_idx);

		int start_idx_a = startVertA->idx;
		int start_idx_b = startVertB->idx;
		m_Nodes[start_idx_a].index = start_idx_a;
		m_Nodes[start_idx_a].gCost = Utility::MathUtility::Distance(startLocation, startVertA->position);
		m_Nodes[start_idx_a].fCost = m_Nodes[start_idx_a].gCost + Heuristic(startVertA->position, goalLocation);
		m_Nodes[start_idx_b].index = start_idx_b;
		m_Nodes[start_idx_b].gCost = Utility::MathUtility::Distance(startLocation, startVertB->position);
		m_Nodes[start_idx_b].fCost = m_Nodes[start_idx_b].gCost + Heuristic(startVertB->position, goalLocation);

		openList.push(&m_Nodes[start_idx_a]);
		openList.push(&m_Nodes[start_idx_b]);

		// if we reach any two of the nodes of the goal edge, we found the path
		// this is because once we have reached A or B, our heuristic perfectly describes the shortest distance
		// so the premise that up to this point, this is the shortest path, still holds true.
		int goal_idx_a = goalEdge->half_edges[0].v_target_idx;
		int goal_idx_b = goalEdge->half_edges[1].v_target_idx;


		// then start AStar search loop
		while (!openList.empty())
		{

			// if we already visited a node, remove it from the list
			while (!openList.empty() && IsVisited(openList.top()->index))
			{
				openList.pop();
			}

			if (openList.empty())
			{
				CleanRequestData();

				return false;
			}

			// front node has lowest distance to end goal, check that one first
			AStarNode& current = *openList.top();
			openList.pop();
			SetVisited(current.index);

			if (m_NodeClearance[current.index] < clearance) continue;

			// if we're at the final node, this means we have found the optimal path: construct path and return
			if (current.index == goal_idx_a || current.index == goal_idx_b)
			{
				ConstructPath(start_idx_a, start_idx_b, goal_idx_a, goal_idx_b, current, outPath);
				CleanRequestData();

				return true;
			}


			int halfEdgeIdx = m_Graph.GetVertex(current.index)->half_edge_idx;
			const ECMHalfEdge* incidentEdge = m_Graph.GetHalfEdge(halfEdgeIdx);
			int startNeighborV = incidentEdge->v_target_idx;
			int nextNeighborV = startNeighborV;

			std::vector<int> neighbors;
			do
			{
				neighbors.push_back(nextNeighborV);
				
				int nextEdgeIdx = incidentEdge->next_idx;
				incidentEdge = m_Graph.GetHalfEdge(nextEdgeIdx);
				nextNeighborV = incidentEdge->v_target_idx;

			} while (startNeighborV != nextNeighborV);


			do {
				if (IsVisited(nextNeighborV)) {
					incidentEdge = m_Graph.GetHalfEdge(incidentEdge->next_idx);
					nextNeighborV = incidentEdge->v_target_idx;
					continue;
				}

				openList.push(&m_Nodes[nextNeighborV]);

				Point currentPosition = m_Graph.GetVertex(current.index)->position;
				Point position = m_Graph.GetVertex(nextNeighborV)->position;

				// update G cost (actual cost to this node) and F cost (G cost + heuristic)
				//float newG = current.gCost + nEdge->Cost();
				float newG = current.gCost + Utility::MathUtility::Distance(currentPosition, position);

				if (newG < m_Nodes[nextNeighborV].gCost)
				{
					float newF = newG + Heuristic(position, goalLocation);

					m_Nodes[nextNeighborV].parentIndex = current.index;
					m_Nodes[nextNeighborV].fCost = newF;
					m_Nodes[nextNeighborV].gCost = newG;
				}

				incidentEdge = m_Graph.GetHalfEdge(incidentEdge->next_idx);
				nextNeighborV = incidentEdge->v_target_idx;
			} while (nextNeighborV != startNeighborV);

		}

		CleanRequestData();
		
		// no path found
		return false;
	}

	void AStar::CleanRequestData()
	{
		for (int i = 0; i < m_Nodes.size(); i++)
		{
			m_Nodes[i].fCost = Utility::MAX_FLOAT;
			m_Nodes[i].gCost = Utility::MAX_FLOAT;
			m_Nodes[i].parentIndex = INVALID_NODE_INDEX;
		}

		for (int i = 0; i < m_Visited.size(); i++)
		{
			m_Visited[i] = false;
		}
	}



	float AStar::Heuristic(Point start, Point goal) const
	{
		return Utility::MathUtility::Distance(start, goal);
	}

	void AStar::ConstructPath(int startV1, int startV2, int goalV1, int goalV2, AStarNode& lastNode, std::vector<int>& outPath)
	{
		// ignore start and goal nodes, they do not exist in the ECM

		std::vector<int> reversedPath;

		if (lastNode.index == goalV1)
		{
			reversedPath.push_back(goalV2);
			reversedPath.push_back(goalV1);
		}
		else
		{
			reversedPath.push_back(goalV1);
			reversedPath.push_back(goalV2);
		}

		int nextIndex = lastNode.parentIndex;
		while (nextIndex < INVALID_NODE_INDEX)
		{
			reversedPath.push_back(nextIndex);
			nextIndex = m_Nodes[nextIndex].parentIndex;
		}

		for (int i = reversedPath.size() - 1; i >= 0; i--)
		{
			outPath.push_back(reversedPath[i]);
		}
	}

	bool AStar::IsVisited(int index) const
	{
		return m_Visited[index];
		//int idx = index >> 3;
		//int remainder = index - idx << 3;
		//
		//return (m_Visited[idx] & (1 << (8 - remainder))) > 0;
	}

	void AStar::SetVisited(int index)
	{
		m_Visited[index] = true;
		//int idx = index >> 3;
		//int remainder = index - idx << 3;
		//
		//m_Visited[idx] = m_Visited[idx] | (1 << (8 - remainder));
	}


}