#pragma once

#include <vector>
#include <queue>

#include "ECM.h"

namespace ECM {

	class ECMGraph;
	class ECMEdge;
	class ECMVertex;

	struct Point;

	// OPTIMIZATION: maybe it is faster if we break the struct in different arrays
	struct AStarNode {
		AStarNode(uint32_t _index, uint32_t _parentIndex, float _gCost, float _fCost) : 
			index(_index), parentIndex(_parentIndex), gCost(_gCost), fCost(_fCost) { }
		AStarNode() { }

		uint32_t index;
		uint32_t parentIndex;
		float gCost;
		float fCost;

		std::vector<int> neighbors;
	};

	class AStarCompare
	{
	public:
		bool operator() (const AStarNode* a, const AStarNode* b)
		{
			return a->fCost > b->fCost;
		}
	};

	class AStar {

	public:
		AStar(ECMGraph& graph) : m_Graph(graph) { }

		bool Initialize(); // set ECM and other variables
		bool Update();
		bool FindPath(const Point& startLocation, const Point& goalLocation, const ECMEdge& startEdge, const ECMEdge& goalEdge, float clearance, std::vector<int>& outPath); // params: startEdge, goalEdge, clearance, vec<ECMEdge>& outPath

		void SetHeuristic();
		float Heuristic(Point start, Point goal) const;

	private:
		void ConstructPath(AStarNode& goal, std::vector<int>& outPath);
		void CleanData();
		void CleanRequestData(int goalNeighborA, int goalNeighborB);
		inline bool IsVisited(int index) const;
		inline void SetVisited(int index);

		ECMGraph& m_Graph;
		std::vector<AStarNode> m_Nodes;
		std::vector<bool> m_Visited;

		uint32_t INVALID_NODE_INDEX;
		uint32_t START_NODE_INDEX;
		uint32_t GOAL_NODE_INDEX;
	};

}