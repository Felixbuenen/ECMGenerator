#include "AStar.h"

#include "ECM.h"
#include "UtilityFunctions.h"

namespace ECM {

	bool AStar::Initialize()
	{
		// initialize astar node array
		const auto& ecmVerts = m_Graph.GetVertices();
		int numVerts = ecmVerts.size();
		m_Nodes.resize(numVerts + 2); // add two for extra start and goal nodes
		m_Visited.resize(numVerts + 2);
		
		int noNeighCounter = 0;
		for (int i = 0; i < numVerts; i++)
		{
			int index = ecmVerts[i].Index();
			m_Nodes[index].index = index;
			m_Nodes[index].neighbors = m_Graph.GetNeighboringVertices(index);
			m_Nodes[index].gCost = Utility::MAX_FLOAT;
			m_Nodes[index].fCost = Utility::MAX_FLOAT;

			if (m_Nodes[index].neighbors.size() == 0) noNeighCounter++;
		}

		if (true)
		{
			int i = noNeighCounter;
		}
		
		START_NODE_INDEX = numVerts;
		GOAL_NODE_INDEX = numVerts + 1;
		INVALID_NODE_INDEX = numVerts + 2;
		
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


	bool AStar::FindPath(const Point& startLocation, const Point& goalLocation, const ECMEdge& startEdge, const ECMEdge& goalEdge, float clearance, std::vector<int>& outPath)
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
		
		// initialize the first nodes
		const ECMVertex& nodeA = m_Graph.GetVertex(startEdge.V0());
		const ECMVertex& nodeB = m_Graph.GetVertex(startEdge.V1());

		int idxA = nodeA.Index();
		int idxB = nodeB.Index();

		m_Nodes[START_NODE_INDEX].index = START_NODE_INDEX;
		m_Nodes[START_NODE_INDEX].neighbors.push_back(idxA);
		m_Nodes[START_NODE_INDEX].neighbors.push_back(idxB);
		m_Nodes[START_NODE_INDEX].gCost = 0.0f;
		m_Nodes[START_NODE_INDEX].fCost = Heuristic(startLocation, goalLocation);

		openList.push(&m_Nodes[START_NODE_INDEX]);

		// check from which two ECM vertices the goal location can be reached
		const ECMVertex& goalNodeA = m_Graph.GetVertex(goalEdge.V0());
		const ECMVertex& goalNodeB = m_Graph.GetVertex(goalEdge.V1());
		int goalIdxA = goalNodeA.Index();
		int goalIdxB = goalNodeB.Index();

		m_Nodes[GOAL_NODE_INDEX].index = GOAL_NODE_INDEX;
		m_Nodes[GOAL_NODE_INDEX].gCost = Utility::MAX_FLOAT;
		m_Nodes[GOAL_NODE_INDEX].fCost = Utility::MAX_FLOAT;
		m_Nodes[GOAL_NODE_INDEX].neighbors.push_back(goalIdxA);
		m_Nodes[GOAL_NODE_INDEX].neighbors.push_back(goalIdxB);
		m_Nodes[goalIdxA].neighbors.push_back(GOAL_NODE_INDEX);
		m_Nodes[goalIdxB].neighbors.push_back(GOAL_NODE_INDEX);

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
				CleanRequestData(goalIdxA, goalIdxB);

				return false;
			}

			// front node has lowest distance to end goal, check that one first
			AStarNode& current = *openList.top();
			openList.pop();
			SetVisited(current.index);

			// if we're at the final node, this means we have found the optimal path: construct path and return
			if (current.index == GOAL_NODE_INDEX)
			{
				ConstructPath(current, outPath);
				CleanRequestData(goalIdxA, goalIdxB);

				return true;
			}

			// loop through neighbors 
			for (int n : current.neighbors)
			{
				//AStarNode& neighbor = m_Nodes[n];

				if (IsVisited(n)) continue;

				openList.push(&m_Nodes[n]);

				if (n == GOAL_NODE_INDEX)
				{
					int i = 0;
				}

				Point currentPosition;
				if (current.index == GOAL_NODE_INDEX) currentPosition = goalLocation;
				else if (current.index == START_NODE_INDEX) currentPosition = startLocation;
				else currentPosition = m_Graph.GetVertex(current.index).Position();

				Point position;
				if (n == GOAL_NODE_INDEX) position = goalLocation;
				else if (n == START_NODE_INDEX) position = startLocation;
				else position = m_Graph.GetVertex(n).Position();

				// update G cost (actual cost to this node) and F cost (G cost + heuristic)
				//float newG = current.gCost + nEdge->Cost();
				float newG = current.gCost + Utility::MathUtility::Distance(currentPosition, position);

				if (newG < m_Nodes[n].gCost)
				{
					float newF = newG + Heuristic(position, goalLocation);

					m_Nodes[n].parentIndex = current.index;
					m_Nodes[n].fCost = newF;
					m_Nodes[n].gCost = newG;
				}


			}
		}

		CleanRequestData(goalIdxA, goalIdxB);
		
		// no path found
		return false;
	}

	void AStar::CleanRequestData(int goalNeighborA, int goalNeighborB)
	{
		m_Nodes[START_NODE_INDEX].neighbors.clear();
		m_Nodes[GOAL_NODE_INDEX].neighbors.clear();

		m_Nodes[goalNeighborA].neighbors.pop_back();
		m_Nodes[goalNeighborB].neighbors.pop_back();

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

		int nextIndex = goal.parentIndex;
		while (nextIndex < INVALID_NODE_INDEX)
		{
			reversedPath.push_back(nextIndex);
			nextIndex = m_Nodes[nextIndex].parentIndex;
		}

		reversedPath.pop_back();

		for (int i = reversedPath.size() - 1; i >= 0; i--)
		{
			outPath.push_back(reversedPath[i]);
		}
	}

	bool AStar::IsVisited(int index) const
	{
		if (index >= m_Visited.size() || index < 0)
		{
			int i = 3;
		}
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