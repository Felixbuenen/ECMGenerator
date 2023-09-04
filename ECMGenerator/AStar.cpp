#include "AStar.h"

#include "ECM.h"
#include "UtilityFunctions.h"

namespace ECM {

	bool AStar::Initialize()
	{
		// initialize astar node array
		const auto& ecmVerts = m_Graph.GetVertices();
		int numVerts = ecmVerts.size();
		m_Nodes.resize(numVerts);
		m_Visited.resize(numVerts);
		
		for (int i = 0; i < numVerts; i++)
		{
			int index = ecmVerts[i].idx;
			m_Nodes[index].index = index;
			m_Nodes[index].gCost = Utility::MAX_FLOAT;
			m_Nodes[index].fCost = Utility::MAX_FLOAT;
		}
		
		INVALID_NODE_INDEX = numVerts;
		
		return true;
	}

	bool AStar::Update()
	{
		/*
		* TODO:
		* This method must be implemented if we implement dynamic changes in the ECM.
		* We should:
		* 1) resize m_Nodes and add the new ECM vertex nodes.
		* 2) update the INVALID_NODE_INDEX value.
		*/
		return false;
	}


	bool AStar::FindPath(const Point& startLocation, const Point& goalLocation, const ECMEdge* startEdge, const ECMEdge* goalEdge, float clearance, std::vector<int>& outPath)
	{
		/*
		*	TODO: nu worden node pointers niet opgeslagen op een centrale plek. Denk na hoe we dit efficient kunnen doen.
		*	Optie 1: maak een array van a star nodes, de size van alle ECM vertices. de index van een ECM vertex representeert de index van de astar node in de array.
		*            je hebt dus constante tijd lookup. dit is de snelste methode, echter bij grote navigation meshes met veel ecm vertices kost het veel geheugen (echter is
		*            geheugen vaak niet het probleem). je hoeft dan ook geen closed list lookup te doen. dit is op dit moment een gigantische kostenpost in grote omgevingen. 
		*			 in plaats daarvan zou je een byte array kunnen opslaan waarbij de individuele bits de 'visited' flag representeren (bespaar je geheugen mee).
		*/

		// create the list of open nodes as a priority queue
		std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarCompare> openList;
		
		// init first nodes
		const ECMVertex* startVertA = m_Graph.GetVertex(startEdge->half_edges[0].v_target_idx);
		const ECMVertex* startVertB = m_Graph.GetVertex(startEdge->half_edges[1].v_target_idx);

		int idxA = startVertA->idx;
		int idxB = startVertB->idx;
		m_Nodes[idxA].index = idxA;
		m_Nodes[idxA].gCost = Utility::MathUtility::Distance(startLocation, startVertA->position);
		m_Nodes[idxA].fCost = m_Nodes[idxA].gCost + Heuristic(startVertA->position, goalLocation);
		m_Nodes[idxB].index = idxB;
		m_Nodes[idxB].gCost = Utility::MathUtility::Distance(startLocation, startVertB->position);
		m_Nodes[idxB].fCost = m_Nodes[idxB].gCost + Heuristic(startVertB->position, goalLocation);

		openList.push(&m_Nodes[idxA]);
		openList.push(&m_Nodes[idxB]);

		// if we reach any two of the nodes of the goal edge, we found the path
		// this is because once we have reached A or B, our heuristic perfectly describes the shortest distance
		// so the premise that up to this point, this is the shortest path, still holds true.
		int goal_idx_a = goalEdge->half_edges[0].v_target_idx;
		int goal_idx_b = goalEdge->half_edges[1].v_target_idx;

		// TODO:
		// we don't need to be difficult about reaching the goal node..
		// once we reach either goal node A or B, we have found the shortest path
		// this is because once we have reached A or B, our heuristic perfectly describes the shortest distance
		// so the premise that up to this point, this is the shortest path, still holds true.


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

			// if we're at the final node, this means we have found the optimal path: construct path and return
			if (current.index == goal_idx_a || current.index == goal_idx_b)
			{
				ConstructPath(current, outPath);
				CleanRequestData();

				return true;
			}


			int halfEdgeIdx = m_Graph.GetVertex(current.index)->half_edge_idx;
			ECMHalfEdge* incidentEdge = m_Graph.GetHalfEdge(halfEdgeIdx);
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

	void AStar::ConstructPath(AStarNode& goal, std::vector<int>& outPath)
	{
		// ignore start and goal nodes, they do not exist in the ECM

		std::vector<int> reversedPath;
		reversedPath.push_back(goal.index);

		int nextIndex = goal.parentIndex;
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