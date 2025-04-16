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
		//    it will be the new 'active' tree. This way, we relax the requirement that a KD-tree must be finished before the next simulation update. We could explicitly
		//	  define a KD-tree update interval.
		class KDTree
		{
		public:
			void Construct(Simulator* simulation);
			void TestConstruct(PositionComponent* positions, int size);
			void Clear();

			void KNearestAgents(Simulator* simulation, int agent, int k, std::vector<Entity>& outAgents, int& outNumNeighbors);
			void AgentsInRange(Simulator* simulation, int agent, float radius, std::vector<Entity>& outAgents, int& outNumNeighbors);

			void AgentsInRangeTest(PositionComponent* positions, int agent, float radius, int maxAgents, std::vector<int>& outAgents);

		private:
			void ConstructRecursive(PositionComponent* positions, int index, KDTreeCompareY& compY, KDTreeCompareX& compX, int leftBound, int rightBound, int depth, int* outIndices);

			void KNearestAgents_R(const Vec2& target, int currentIndex, int k, int& kFound, int depth, std::vector<Entity>& nearestIndices, std::vector<float>& nearestSqDistances, PositionComponent* positions);
			void AgentsInRange_R(const Vec2& target, float sqRadius, int currentIndex, int maxAgents, int& nFound, int depth, PositionComponent* positions, std::vector<int>& outAgents);

			int LeftTree(int root) const;
			int RightTree(int root) const;

			std::vector<int> m_Tree;
			int m_MaxDepth;
		};

	}

}