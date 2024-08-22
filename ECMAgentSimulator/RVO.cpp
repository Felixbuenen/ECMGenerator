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
			// TODO 2024-08-21:
			// - constraint generation tested. seems to be working based on dummy data.
			// - write algorithm that solves the linear program based on the generated constraints
			// ----------------

			// create memory buffer for neighbor information
			// nPositions, nPrefVel, nClearance

			// loop through all agents
			for (int i = 0; i <= lastIndex; i++)
			{
				// find N closest neighbors

				// generate constraints

				// solve linear program

				// update velocity
			}
		}


		// TODO: RVO should have access to the datastructure that holds agent positions and query from there. Does not make sense to write the query logic here.
		void RVO::GetNNearestAgents(const PositionComponent& positions, int N, std::vector<int>& outIndices) const
		{
			// TODO:
			// - for now just loop through all neighbors and keep track of the closest ones.
			// - improve with grid or KD method
		}

		// VERIFIED: using dummy agents, basic calculation of constraints seems to be working now (2024-08-21)
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
				float VOPosLength = Utility::MathUtility::Length(VOPos);
				float VORadius = (nClearances[i].clearance + clearance.clearance) * dtRecip;
				float VORadiusSqr = VORadius * VORadius;

				Vec2 relVel(preferredVelocity.dx - nPreferredVelocities[i].dx, preferredVelocity.dy - nPreferredVelocities[i].dy);

				const Vec2& leftPerpendicular = Utility::MathUtility::Left(VOPos) / VOPosLength;

				// calculate left and right leg of the VO. These legs are tangent to the VO circle. We can calculate the angle (relative to VOPos) using basic trigonometry.
				float tanAngle = asinf(VORadius / VOPosLength);
				Vec2 VOLeftLeg = Utility::MathUtility::RotateVector(VOPos, tanAngle); // note: positive angle is anti-clockwise rotation
				Vec2 VORightLeg = Utility::MathUtility::RotateVector(VOPos, -tanAngle);

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
				bool liesBelowCircleMiddle = Utility::MathUtility::IsLeftOfVector(VOLeftLeg - VOPos, relVel - VOPos);

				if (!liesWithinCircle && liesBelowCircleMiddle) continue;

				// --------
				// At this point we know the point lies within the VO.
				// Now we calculate the constraint that is imposed by this VO.
				// --------

				if (liesWithinCircle && liesBelowCircleMiddle)
				{
					// the constraint is the half-plane at the edge of the circle of the VO
					float distToEdge = VORadius - sqrtf(sqDistFromCircleCentre);

					outConstraints[counter].N = (relVel - VOPos);
					outConstraints[counter].N.Normalize();

					outConstraints[counter].Pos = Vec2(preferredVelocity.dx, preferredVelocity.dy) + outConstraints[counter].N * distToEdge * 0.5f;

					counter++;
				}
				else
				{
					// find closest point to the VO edges. First determine whether RelVel lies closer to the left or right VO leg.
					bool closerToLeftLeg = Utility::MathUtility::Dot(leftPerpendicular, relVel) >= 0;
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

		void RVO::RandomizedLP(const std::vector<Constraint>& constraints, const PositionComponent& position, const float maxSpeed, Vec2& outVelocity) const
		{
			// todo: we want a fixed array so this doesn't work. instead check if first element is an 'empty' vec2.
			if (constraints.empty()) return;

			// 1. maak een permutatie van het aantal constraints, zodat je de constraints random afgaat.
			// >> is this necessary? could be slow...
			// 
			// For now: loop through constraints from 0..N.
			// 
			
			// note we add the speed constraint using a circle of radius "Speed" around the agent's position
			int numConstraints = constraints.size();
			for (int i = 0; i < numConstraints; i++)
			{
				// find intersections and generate range [Xleft..Xright]
				

				// solve 1D constraint
			}


			// 2. Nu ga je iteratief constraints toevoegen.
			//		- als Vn in de area van de nieuwe half-plane constraint ligt, dan blijft Vn onveranderd (nog steeds optimaal). Zie Lemma 4.5, een nieuwe constraint waar Vn
			//			invalt kan geen nieuwe optimale verlocity creeeren.
			//		- als Vn niet in de area van de nieuwe half-plane constraint ligt, dan moet er een nieuwe Vn gevonden worden. Deze ligt op de lijn L van de nieuwe constraint:
			//			> bereken alle intersecties met de nieuwe constraint
			//			> nu heb je een aantal intersecties, zie je lijn L als een horizontale lijn voor je
			//			> je hebt nu intersecties die "left-bounded" zijn (d.w.z.: je mag niet naar links op L maar wel naar rechts) en je hebt "right-bounded" intersecties.
			//			> van de left-bounded intersecties, pak degene met de hoogste X waarde: Xleft.
			//			> van de right-bounded intersecties, pak degene met de laagste X waarde: Xright.
			//			> nu heb je het segment (Xleft, Xright). hier ligt de nieuwe Vn op (bereken door closest point on line).
			//		- herhaal tot je alle constraints hebt gehad.
		}

	}

}