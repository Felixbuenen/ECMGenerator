#pragma once

#include <vector>

#include "Simulator.h"

#define KDTREE_NULL_NODE -1

namespace ECM {

	struct Point;

	namespace Simulation {

		class Simulator;
		struct PositionComponent;

		struct KDTreeNode
		{
			Point* pos;
		};

		struct KDTreeCompareX
		{
		public:
			KDTreeCompareX(PositionComponent* positions) : m_Positions(positions) { }

			bool operator() (const int a, const int b)
			{
				return m_Positions[a].x < m_Positions[b].x;
			}

		private:
			PositionComponent* m_Positions;
		};

		struct KDTreeCompareY
		{
		public:
			KDTreeCompareY(PositionComponent* positions) : m_Positions(positions) { }

			bool operator() (const int a, const int b)
			{
				return m_Positions[a].y < m_Positions[b].y;
			}

		private:
			PositionComponent* m_Positions;
		};

		// TODO:
		// Have 2 KD-trees used by the simulation:
		// 1. Active tree. This is the tree that is used by KNN queries. By the simulation. It is read-only.
		// 2. Build tree. This is the tree object that gets updated by a separate thread. The simulation object does not use this tree. When the build tree is completed, it
		//    it will be the new 'active' tree. This way, we relax the requirement that a KD-tree must be finished before the next simulation update. Why is this okay? It depends of course.
		//    But the simulation is a flowing motion. Let's say it takes 200ms to update a KD-tree. Then, on average, the simulation will use an outdated tree every other update (if sim step = 100ms).
		//    After one simulation step, we make the assumption/relaxation that the neighbors have not changed drastically. Any KNN errors will therefor be minimal. We exploit this fact. 
		// In fact, we could say: rebuild tree every 500ms to make it more explicit.
		class KDTree
		{
		public:
			void Construct(Simulator* simulation);
			void TestConstruct(PositionComponent* positions, int size);
			void Clear();

			// Reference: https://gopalcdas.com/2017/05/24/construction-of-k-d-tree-and-using-it-for-nearest-neighbour-search/
			void KNearestAgents(Simulator* simulation, int agent, int k, int* outAgents);
			void KNearestAgentsTest(PositionComponent* positions, int agent, int k, int* outAgents);

			void AgentsInRange(Simulator* simulation, int agent, float radius, int* outAgents);
			void AgentsInRangeTest(PositionComponent* positions, int agent, float radius, int maxAgents, std::vector<int>& outAgents);

		private:
			void ConstructRecursive(PositionComponent* positions, int index, KDTreeCompareY& compY, KDTreeCompareX compX, int leftBound, int rightBound, int depth, int* outIndices);

			void KNearestAgents_R(const Vec2& target, int currentIndex, int k, int& kFound, int depth, std::vector<int>& nearestIndices, std::vector<float>& nearestSqDistances, PositionComponent* positions, int* outAgents);
			void AgentsInRange_R(const Vec2& target, float sqRadius, int currentIndex, int maxAgents, int& nFound, int depth, PositionComponent* positions, std::vector<int>& outAgents);

			int LeftTree(int root) const;
			int RightTree(int root) const;
			
			std::vector<int> m_Tree;
			int m_MaxDepth;
		};

	}

}