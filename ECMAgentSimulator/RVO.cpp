#include "RVO.h"

#include "Simulator.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"

#include <cmath>

namespace ECM {

	namespace Simulation {

		void RVO::GetRVOVelocities(Simulator* simulator, VelocityComponent* outVelocities)
		{
			int numAgents = simulator->GetNumAgents();
			int lastIndex = simulator->GetLastIndex();
			bool* activeAgents = simulator->GetActiveFlags();
			PositionComponent* positions = simulator->GetPositionData();

			// loop through all agents
			for (int i = 0; i <= lastIndex; i++)
			{

			}
		}

		void RVO::GenerateConstraints(float dt, int nNeighbors,
			const PositionComponent& position, const VelocityComponent& preferredVelocity, const ClearanceComponent& clearance,
			const PositionComponent* nPositions, const VelocityComponent* nPreferredVelocities, const ClearanceComponent* nClearances,
			std::vector<Constraint>& outConstraints)
		{

			float dtRecip = 1.0f / dt;
			int counter = 0;

			// for each neighboring agent, generate the VO
			for (int i = 0; i < nNeighbors; i++)
			{
				Point VOPos(nPositions[i].x - position.x, nPositions[i].y - position.y);
				float VORadius = (nClearances[i].clearance + clearance.clearance) * dtRecip;
				float VORadiusSqr = VORadius * VORadius;

				Point relVel(nPreferredVelocities[i].dx - preferredVelocity.dx, nPreferredVelocities[i].dy - preferredVelocity.dy);
				float relVelDist = Utility::MathUtility::SquaredLength(relVel.x, relVel.y);

				// the relative velocity lies within the circle of the VO
				float sqDistFromCircleCentre = Utility::MathUtility::SquareDistance(VOPos, relVel);

				// TODO: dit klopt niet: alleen voor het onderste deel van de cirkel klopt dit.
				if (sqDistFromCircleCentre < VORadiusSqr)
				{
					// the constraint is the half-plane at the edge of the circle of the VO
					float distToEdge = sqrtf(VORadiusSqr - sqDistFromCircleCentre);

					outConstraints[counter].N = (relVel - VOPos);
					outConstraints[counter].N.Normalize();

					// TODO: dit klopt niet. Positie is iets met u * 0.5, ik ben te gaar hiervoor nu. zie paper.
					outConstraints[counter].Pos = relVel + outConstraints[counter].N * distToEdge;

					counter++;
				}

				// the relative velocity lies outside the circle of the VO
				else
				{
					// calculate VO "legs"
					bool isLeft = Utility::MathUtility::IsLeftOfSegment(Segment(Point(0, 0), VOPos), relVel); // TODO: make IsLeftOfVector()
					if (isLeft)
					{
						Vec2 leftLeg = Utility::MathUtility::Left(VOPos);
						leftLeg.Normalize();
						leftLeg = leftLeg * VORadius;
						leftLeg = leftLeg + VOPos;

						// todo: 
						// is the relvel left of the left leg? then it is not on a collision path
						if (Utility::MathUtility::IsLeftOfSegment(Segment(Point(0, 0), Point(leftLeg.x, leftLeg.y)), relVel)) continue;

						// if it is not: is the distance to the origin smaller then dt? then it is not (yet) on a collision path.
						if (Utility::MathUtility::SquaredLength(relVel) < (dt * dt)) continue;
						
						// otherwise calculate the closest position on the left leg and calculate the constraint
						leftLeg.Normalize();
						outConstraints[counter].Pos = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(leftLeg, relVel);
						outConstraints[counter].N = Utility::MathUtility::Left(leftLeg);
					}
					else
					{
						Vec2 rightLeg = Utility::MathUtility::Right(VOPos);
						rightLeg.Normalize();
						rightLeg = rightLeg * VORadius;
						rightLeg = rightLeg + VOPos;

						// todo: 
						// is the relvel right of the right leg? then it is not on a collision path
						if (!Utility::MathUtility::IsLeftOfSegment(Segment(Point(0, 0), Point(rightLeg.x, rightLeg.y)), relVel)) continue;

						// * if it is: is the distance to the origin smaller then dt? then it is not (yet) on a collision path.
						if (Utility::MathUtility::SquaredLength(relVel) < (dt * dt)) continue;

						// * otherwise calculate the closest position on the right leg and calculate the constraint
						rightLeg.Normalize();
						outConstraints[counter].Pos = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(rightLeg, relVel);
						outConstraints[counter].N = Utility::MathUtility::Right(rightLeg);
					}
				}
			}
		}

	}

}