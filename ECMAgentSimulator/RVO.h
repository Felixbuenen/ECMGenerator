#pragma once

#include "ECMDataTypes.h"

#include <vector>

namespace ECM {

	namespace Simulation {

		class Simulator;

		struct PositionComponent;
		struct VelocityComponent;
		struct ClearanceComponent;

		// ORCA: https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf
		// The white paper contains a clear description of the model, with conservative collision time as the selector condition, as well as the adaptation to a
		// shape-agnostic implementation. I also found it interesting to implement the 3 described avoid-exceptions to create more dynamic / realistic behavior for
		// game scenarios.

		// A constraint in ORCA is defined by a half-plane. We represent the half-plane line by a point on this line. The direction of the half-plane is indicated by the normal N.
		struct Constraint
		{
			Point Pos;
			Vec2 N;
		};

		// TODO:
		// change class name/script name to ORCA
		class RVO 
		{
		public:

			// get the RVO velocities based on the given simulator object
			void GetRVOVelocities(Simulator* simulator, VelocityComponent* outVelocities);

			void GenerateConstraints(float dt, int nNeighbors,
				const PositionComponent& position, const VelocityComponent& preferredVelocity, const ClearanceComponent& clearance,
				const PositionComponent* nPositions, const VelocityComponent* nPreferredVelocities, const ClearanceComponent* nClearances,
				std::vector<Constraint>& outConstraints);

		private:
			// settings, e.g. how many agents to consider
			int m_MaxNeighbors;

			// TODO:

			// RandomizedLP(halfPlanes, prefVel); // given a set of half planes and the preferredVelocity, solve the LP
		};

	}
}