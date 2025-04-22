#include "KDTree.h"

#include "Simulator.h"
#include "ECMDataTypes.h"
#include "UtilityFunctions.h"
#include "Timer.h"

namespace ECM {

	namespace Simulation {

		int KDTree::LeftTree(int root) const
		{
			return root * 2 + 1;

		}
		int KDTree::RightTree(int root) const
		{
			return root * 2 + 2;
		}

		void KDTree::Construct(Simulator* simulation)
		{
			volatile MultipassTimer orcaTimer("KDTree::Construct()");
			PositionComponent* positions = simulation->GetPositionData();

			int size = simulation->GetNumAgents();
			if (size == 0) return;

			// create buffer with indices of active agents
			int* indices = new int[size];
			bool* activeFlags = simulation->GetActiveFlags();
			int currentIndex = 0;
			for (int i = 0; i <= simulation->GetLastIndex(); i++)
			{
				if (!activeFlags[i]) 
					continue;

				indices[currentIndex] = i;
				currentIndex++;
			}

			// compare objects used to sort on X- and Y-axis
			KDTreeCompareX compX(positions);
			KDTreeCompareY compY(positions);

			// create the KD tree buffer. The tree is represented in memory as an array.
			m_MaxDepth = std::ceil(std::log2(size + 1) - 1); // formula to calculate the height of a balanced binary tree
			int treeSize = std::pow(2, (m_MaxDepth + 1)) - 1;
			m_Tree.clear();
			m_Tree.resize(treeSize, KDTREE_NULL_NODE);

			int numIndices = currentIndex;
			ConstructRecursive(positions, 0, compY, compX, 0, size, 0, indices);

			delete[] indices;
		}

		void KDTree::ConstructRecursive(PositionComponent* positions, int index, KDTreeCompareY& compY, KDTreeCompareX& compX, int leftBound, int rightBound, int depth, int* indices)
		{
			// base case: stop recursion
			if (leftBound >= rightBound) return;

			// sort on X or Y-axis, based on the current subtree level
			if (depth % 2 == 0)
			{
				std::sort(indices + leftBound, indices + rightBound, compX);
			}
			else
			{
				std::sort(indices + leftBound, indices + rightBound, compY);
			}

			// the current tree node gets the middle value of the sorted subtree values
			int mid = leftBound + (rightBound - 1 - leftBound) / 2;
			m_Tree[index] = indices[mid];

			// build left child tree
			ConstructRecursive(positions, LeftTree(index), compY, compX, leftBound, mid, depth + 1, indices);

			// build right child tree
			ConstructRecursive(positions, RightTree(index), compY, compX, mid + 1, rightBound, depth + 1, indices);
		}

		void KDTree::KNearestAgents(Simulator* simulation, int agent, int k, std::vector<Entity>& outAgents, int& outNumNeighbors)
		{
			// TODO: cache?
			std::vector<float> nearestSqDistances;
			nearestSqDistances.resize(k, Utility::MAX_FLOAT);

			PositionComponent* positions = simulation->GetPositionData();
			Vec2 target(positions[agent].x, positions[agent].y);

			outNumNeighbors = 0;
			KNearestAgents_R(target, 0, k, outNumNeighbors, 0, outAgents, nearestSqDistances, positions);
		}

		void KDTree::KNearestAgents_R(const Vec2& target, int currentIndex, int k, int& kFound, int depth, std::vector<Entity>& nearestIndices, std::vector<float>& nearestSqDistances, PositionComponent* positions)
		{
			// stop recursion if we reached end of tree
			if (depth > m_MaxDepth) return;
			if (m_Tree[currentIndex] == KDTREE_NULL_NODE) return;

			// - Updating the search results -
			// first: check the distance to the position of the current agent
			const PositionComponent& currentPosition = positions[m_Tree[currentIndex]];
			Vec2 diffToNode(currentPosition.x - target.x, currentPosition.y - target.y);
			float sqDistToCurrentNode = diffToNode.x * diffToNode.x + diffToNode.y * diffToNode.y;

			// second: is the current agent closer than 1 of the closest k nearest agents?
			// first fill up the k nearest data
			if (kFound < k && sqDistToCurrentNode > Utility::EPSILON)
			{
				nearestIndices[kFound] = m_Tree[currentIndex];
				nearestSqDistances[kFound] = sqDistToCurrentNode;
				kFound++;

				if (kFound == k)
				{
					// k nearest filled, make sure the last element contains the largest element

					// TODO: instead of last index, make first index the largest distance
					float largestDistance = sqDistToCurrentNode;
					int largetIndexTree = k - 1;
					for (int i = 0; i < (k - 1); i++)
					{
						if (nearestSqDistances[i] > largestDistance)
						{
							largestDistance = nearestSqDistances[i];
							largetIndexTree = i;
						}
					}

					// update list such that the first element contains the neighbor with the largest distance
					nearestSqDistances[0] = largestDistance;
					nearestIndices[0] = nearestIndices[largetIndexTree];
					nearestSqDistances[largetIndexTree] = sqDistToCurrentNode;
					nearestIndices[largetIndexTree] = m_Tree[currentIndex];
				}
			}
			else
			{
				kFound = k;

				// only check the first element. the first element contains the furthest distance.
				if (sqDistToCurrentNode < nearestSqDistances[0] && sqDistToCurrentNode > Utility::EPSILON)
				{
					nearestSqDistances[0] = sqDistToCurrentNode;
					nearestIndices[0] = m_Tree[currentIndex];

					// go through list of current k nearest to find the new largest distance and update the k nearest list accordingly.
					float largestDistance = sqDistToCurrentNode;
					int largetIndexTree = 0;
					for (int i = 1; i < k; i++)
					{
						if (nearestSqDistances[i] > largestDistance)
						{
							largestDistance = nearestSqDistances[i];
							largetIndexTree = i;
						}
					}

					// update list to swap the highest distance with the last element
					nearestSqDistances[0] = largestDistance;
					nearestIndices[0] = nearestIndices[largetIndexTree];
					nearestSqDistances[largetIndexTree] = sqDistToCurrentNode;
					nearestIndices[largetIndexTree] = m_Tree[currentIndex];
				}
			}

			// finally decide which sub-tree we will search first. We first search the sub-tree that contains the target.
			float currentVal = depth % 2 == 0 ? currentPosition.x : currentPosition.y;
			float targetValToCheck = depth % 2 == 0 ? target.x : target.y;
			
			// Update the threshold for the right subtree based on the current node's position.
			if (targetValToCheck < currentVal) {
				// recurse left tree first
				KNearestAgents_R(target, LeftTree(currentIndex), k, kFound, depth + 1, nearestIndices, nearestSqDistances, positions);
				
				float distToBBOX = (targetValToCheck - currentVal);
				float sqDistanceToBBOX = distToBBOX * distToBBOX;
			
				// Check if you need to search the right subtree based on the updated threshold.
				// I.e.: if the furthest nearest neighbor is closer than the splitting node, than there cannot be a neighbor in the other half that is closer (-> prune).
				if (sqDistanceToBBOX < nearestSqDistances[k - 1]) {
					KNearestAgents_R(target, RightTree(currentIndex), k, kFound, depth + 1, nearestIndices, nearestSqDistances, positions);
				}
			}
			else {
				// recurse right tree first
				KNearestAgents_R(target, RightTree(currentIndex), k, kFound, depth + 1, nearestIndices, nearestSqDistances, positions);
			
				float distToBBOX = (targetValToCheck - currentVal);
				float sqDistanceToBBOX = distToBBOX * distToBBOX;

				// Check if you need to search the left subtree based on the updated threshold.
				if (sqDistanceToBBOX < nearestSqDistances[k - 1]) {
					KNearestAgents_R(target, LeftTree(currentIndex), k, kFound, depth + 1, nearestIndices, nearestSqDistances, positions);
				}
			}

		}

		// Very similar implementation to KNearestAgents_R(). We do not sort because we do not require the nearest: we simply require any agent that is within the specified range.
		// We cap the search when the maximum number of agents is reached. This means it is theoretically possible that we miss certain agents within our vicinity. 
		void KDTree::AgentsInRange_R(const Vec2& target, float sqRadius, int currentIndex, int maxAgents, int& nFound, int depth, PositionComponent* positions, std::vector<int>& outAgents)
		{
			// stop recursion if we reached end of tree or if we reached maximum number of agents: stop recursion.
			if (depth > m_MaxDepth) return;
			if (m_Tree[currentIndex] == KDTREE_NULL_NODE) return;
			if (nFound == maxAgents) return;

			// - Updating the search results -
			// first: check the distance to the position of the current agent
			const PositionComponent& currentPosition = positions[m_Tree[currentIndex]];
			Vec2 diffToNode(currentPosition.x - target.x, currentPosition.y - target.y);
			float sqDistToCurrentNode = diffToNode.x * diffToNode.x + diffToNode.y * diffToNode.y;

			// second: is the current agent within the specified range? Then add it to our results.
			if (sqDistToCurrentNode < sqRadius)
			{
				outAgents[nFound] = m_Tree[currentIndex];
				nFound++;
			}

			// If we reached maximum number of agents: stop recursion to prevent unnecessary recursive calls.
			// Note we have this condition also at the start of the method: it could be that a recursive call was done at a moment
			//  when nFound != maxAgents, but now it is. 
			if (nFound == maxAgents) return;

			// finally decide which sub-tree we will search first. We first search the sub-tree that contains the target.
			float currentVal = depth % 2 == 0 ? currentPosition.x : currentPosition.y;
			float targetValToCheck = depth % 2 == 0 ? target.x : target.y;

			if (targetValToCheck < currentVal) {
				// recurse left tree first
				AgentsInRange_R(target, sqRadius, LeftTree(currentIndex), maxAgents, nFound, depth + 1, positions, outAgents);

				float sqDistanceToBBOX = 0.0f;

				// Update the threshold for the right subtree based on the current node's position.
				if (depth % 2 == 0) {
					sqDistanceToBBOX = std::pow(target.x - currentVal, 2.0f);
				}
				else {
					sqDistanceToBBOX = std::pow(target.y - currentVal, 2.0f);
				}

				// Check if you need to search the right subtree based on the updated threshold.
				if (sqDistanceToBBOX < sqRadius) {
					AgentsInRange_R(target, sqRadius, RightTree(currentIndex), maxAgents, nFound, depth + 1, positions, outAgents);
				}
			}
			else {
				// recurse right tree first
				AgentsInRange_R(target, sqRadius, RightTree(currentIndex), maxAgents, nFound, depth + 1, positions, outAgents);

				float sqDistanceToBBOX = 0.0f;

				// Update the threshold for the left subtree based on the current node's position.
				if (depth % 2 == 0) {
					sqDistanceToBBOX = std::pow(currentVal - target.x, 2.0f);
				}
				else {
					sqDistanceToBBOX = std::pow(currentVal - target.y, 2.0f);
				}

				// Check if you need to search the left subtree based on the updated threshold.
				if (sqDistanceToBBOX < sqRadius) {
					AgentsInRange_R(target, sqRadius, LeftTree(currentIndex), maxAgents, nFound, depth + 1, positions, outAgents);
				}
			}

		}


		void KDTree::Clear()
		{
			m_Tree.clear();
		}


		// DEBUG METHODS

		void KDTree::TestConstruct(PositionComponent* positions, int size)
		{
			int* indices = new int[size];
			for (int i = 0; i < size; i++)
			{
				indices[i] = i;
			}
			int currentIndex = 0;

			KDTreeCompareX compX(positions);
			KDTreeCompareY compY(positions);

			m_MaxDepth = std::ceil(std::log2(size + 1) - 1); // this is the formula to calculate the height of a balanced binary tree
			int treeSize = std::pow(2, (m_MaxDepth + 1)) - 1;
			m_Tree.clear();
			m_Tree.resize(treeSize, KDTREE_NULL_NODE);

			ConstructRecursive(positions, 0, compY, compX, 0, size - 1, 0, indices);

			delete[] indices;
		}


		void KDTree::AgentsInRangeTest(PositionComponent* positions, int agent, float radius, int maxAgents, std::vector<int>& outAgents)
		{
			outAgents.resize(maxAgents, -1);

			float sqRadius = radius * radius;
			Vec2 target(positions[agent].x, positions[agent].y);

			int numFound = 0;
			AgentsInRange_R(target, sqRadius, 0, maxAgents, numFound, 0, positions, outAgents);
		}

	}

}