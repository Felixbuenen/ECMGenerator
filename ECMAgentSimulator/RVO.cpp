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


			// ----------------
			// TODO 2024-01-08:
			// - first version of the constraint generation is written. Test if it works (dummy input).
			// - write algorithm that solves the linear program based on the generated constraints
			// ----------------

			// loop through all agents
			for (int i = 0; i <= lastIndex; i++)
			{
				// generate constraints

				// solve linear program

				// update velocity
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
				Point VOPos((nPositions[i].x - position.x) * dtRecip, (nPositions[i].y - position.y) * dtRecip);
				float VORadius = (nClearances[i].clearance + clearance.clearance) * dtRecip;
				float VORadiusSqr = VORadius * VORadius;

				Vec2 relVel(preferredVelocity.dx - nPreferredVelocities[i].dx, preferredVelocity.dy - nPreferredVelocities[i].dy);

				const Vec2& leftPerpendicular = Utility::MathUtility::Left(VOPos).Normalized();
				const Vec2& VOLeftLeg = VOPos + leftPerpendicular * VORadiusSqr;
				const Vec2& VORightLeg = VOPos - leftPerpendicular * VORadiusSqr;

				// Rel velocity (RelVel) ligt in VO wanneer:
				// 1. RelVel tussen de twee legs ligt
				// 2. De afstand van de cirkel tot RelVel kleiner is dan de cirkel radius
				//    OF RelVel ligt boven de relatieve middellijn van de cirkel
				
				// relative velocity vector lies within cone if LeftLeg X RelVel and RelVel X RightLeg have the same sign (i.e. >= 0)
				// if relative velocity lies outside of this cone, the relative velocity lies outside the VO
				bool withinConeAngle = (Utility::MathUtility::Cross(VOLeftLeg, relVel) * Utility::MathUtility::Cross(relVel, VORightLeg)) >= 0;
				if (!withinConeAngle) continue;

				// if the distance of the relative velocity to the VO circle centre is smaller than its radius, the relative velocity lies inside the VO.
				// if not, than the rel velocity only lies in the VO if it lies above the VO circle position
				float sqDistFromCircleCentre = Utility::MathUtility::SquareDistance(VOPos, relVel);
				bool liesWithinCircle = sqDistFromCircleCentre < VORadiusSqr;

				if (!liesWithinCircle && Utility::MathUtility::IsLeftOfVector(VOLeftLeg - VOPos, relVel - VOPos)) continue;

				// --------
				// At this point we know the point lies within the VO.
				// Now we calculate the constraint that is imposed by this VO.
				// --------

				//VERIFIED WITH TEST
				if (liesWithinCircle)
				{
					// the constraint is the half-plane at the edge of the circle of the VO
					float distToEdge = sqrtf(VORadiusSqr) - sqrtf(sqDistFromCircleCentre);

					outConstraints[counter].N = (relVel - VOPos);
					outConstraints[counter].N.Normalize();

					outConstraints[counter].Pos = Vec2(preferredVelocity.dx, preferredVelocity.dy) + outConstraints[counter].N * distToEdge * 0.5f;

					counter++;
				}
				// NOT VERIFIED WITH TEST
				else
				{
					// find closest point to the VO edges. First determine whether RelVel lies closer to the left or right VO leg.
					bool closerToLeftLeg = (relVel - VOPos).x < 0.0f;
					if (closerToLeftLeg)
					{
						const Vec2& leftLegNormalized = VOLeftLeg.Normalized();
						outConstraints[counter].N = Utility::MathUtility::Left(leftLegNormalized);
						const Vec2& U = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(leftLegNormalized, relVel) - relVel;

						outConstraints[counter].Pos = Vec2(preferredVelocity.dx, preferredVelocity.dy) + U * 0.5f;

						counter++;
					}
					else
					{
						const Vec2& rightLegNormalized = VORightLeg.Normalized();
						outConstraints[counter].N = Utility::MathUtility::Right(rightLegNormalized);
						const Vec2& U = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(rightLegNormalized, relVel) - relVel;

						outConstraints[counter].Pos = Vec2(preferredVelocity.dx, preferredVelocity.dy) + U * 0.5f;

						counter++;
					}
				}
			}
		}

	}

}