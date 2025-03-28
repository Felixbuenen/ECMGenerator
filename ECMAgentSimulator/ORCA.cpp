#include "ORCA.h"

#include "Simulator.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"

#include <cmath>

namespace ECM {

	namespace Simulation {

		void ORCA::GetVelocity(Simulator* simulator, const Entity& entity, float stepSize, float maxSpeed, int nNeighbors, Vec2& outVelocity)
		{
			// agent neighbors
			std::vector<Entity> agentNeighbors;
			simulator->FindNNearestNeighbors(entity, nNeighbors, agentNeighbors);

			// static obstacle neighbors
			std::vector<const ObstacleVertex*> obstNeighbors;
			float range = m_lookAheadObstacle * maxSpeed + simulator->GetClearanceData()[entity].clearance;
			simulator->FindNearestObstacles(entity, range * range, obstNeighbors);

			// generate constraints
			std::vector<Constraint> constraints;
			int nObstConstraints;
			GenerateConstraints(simulator, entity, agentNeighbors, obstNeighbors, stepSize, nObstConstraints, constraints);

			const VelocityComponent& prefVelComp = simulator->GetPreferredVelocityData()[entity];
			Vec2 prefVel(prefVelComp.dx, prefVelComp.dy);

			// use constraints and LP to calculate new velocity
			int failedIndex = RandomizedLP(constraints, prefVel, maxSpeed, false, outVelocity);
			if (failedIndex < constraints.size())
			{
				RandomizedLP3D(nObstConstraints, constraints, maxSpeed, failedIndex, outVelocity);
			}
		}

		// generates the ORCA constraints
		void ORCA::GenerateConstraints(Simulator* simulator, const Entity& entity, const std::vector<Entity>& agentNeighbors, const std::vector<const ObstacleVertex*>& obstNeighbors, float stepSize, int& outNObstacleConstraints, std::vector<Constraint>& outConstraints)
		{
			const PositionComponent& positionComp = simulator->GetPositionData()[entity];
			const ClearanceComponent& clearance = simulator->GetClearanceData()[entity];
			const VelocityComponent& velocityComp = simulator->GetVelocityData()[entity];

			Point position(positionComp.x, positionComp.y);
			Vec2 velocity(velocityComp.dx, velocityComp.dy);

			// Calculate obstacle constraints
			for (int i = 0; i < obstNeighbors.size(); i++)
			{
				const ObstacleVertex* obstNeighborL = obstNeighbors[i];
				const ObstacleVertex* obstNeighborR = obstNeighbors[i]->nextObstacle; 

				Vec2 relativePosition1 = obstNeighborL->p - position;
				Vec2 relativePosition2 = obstNeighborR->p - position;

				Vec2 segUnitDir = obstNeighborR->p - obstNeighborL->p;
				const float s = (Utility::MathUtility::Dot(relativePosition1 * -1.0f, segUnitDir)) / Utility::MathUtility::SquaredLength(segUnitDir);

				// squared distance to infinite line on which the line segment lies
				const float distSqLine = Utility::MathUtility::SquaredLength(relativePosition1 * -1.0f - segUnitDir * s);

				// first check if the agent is currently colliding with the obstacle
				const float distSq1 = Utility::MathUtility::SquaredLength(relativePosition1);
				const float distSq2 = Utility::MathUtility::SquaredLength(relativePosition2);

				segUnitDir.Normalize();
				const float radiusSq = clearance.clearance * clearance.clearance;

				// Collision case 1: agent is left of obstacle line and intersects with left obstacle vertex
				if (s < 0.0f && distSq1 <= radiusSq) {

					// If the obstacle point is convex, we want to push the agent away from this point to avoid further penetration
					if (obstNeighborL->isConvex) {
						Constraint c;
						c.Init(Vec2(0.0f, 0.0f), (relativePosition1 * -1.0f).Normalized());

						outConstraints.push_back(c);
					}

					// If the point is concave, then we don't want to push the agent away, because we might push the agent into the neighboring obstacle segment.
					// Therefore we skip this iteration, and rely on the constraint of the neighboring obstacle line.
					continue;
				}

				// Collision case 2: agent is right of obstacle line and intersects with right obstacle vertex
				else if (s > 1.0f && distSq2 <= radiusSq) {

					// same rules apply for collision case 1, except we push the agent away from the right vertex
					Vec2 rightNeighborDir = (obstNeighborR->nextObstacle->p - obstNeighborR->p);
					rightNeighborDir.Normalize();
					if (obstNeighborR->isConvex && Utility::MathUtility::Determinant(relativePosition2, rightNeighborDir) >= 0.0f) {
						Constraint c;
						c.Init(Vec2(0.0f, 0.0f), (relativePosition2 * -1.0f).Normalized());

						outConstraints.push_back(c);
					}

					continue;
				}

				// Collision case 3: agent lies within obstacle segment and intersects with obstacle line
				else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
					Constraint c;

					// we want to push the agent away from the segment (opposite direction of segment normal)
					// the inverted obstacle segment normal is the normalized vector that is clockwise perpendicular to the obstacle segment
					Vec2 invObstNormal = Utility::MathUtility::Right(segUnitDir);
					c.Init(Vec2(0.0f, 0.0f), invObstNormal);

					outConstraints.push_back(c);

					continue;
				}

				// Now we know that the agent does not intersect with the obstacle segment. We can calculate the VO of the obstacle segment
				// and construct the constraint this obstacle imposes on the agent.

				Vec2 leftLegDirection, rightLegDirection;

				// This means that the agent is left of the left obstacle vertex AND that the disk of the agent falls partly behind the obstacle line.
				// This means that for calculating the obstacle constraint we should only take into account the left vertex, because from this angle, the only
				// straight line collision we may have in the future is with the left vertex. Of course, if this vertex is not convex, we know for sure that
				// we are colliding with the obstacle left of this vertex, so we should correct that and we can ignore this obstacle.
				if (s < 0.0f && distSqLine <= radiusSq) {

					// if the left vertex is not convex, we are colliding, and we can ignore this obstacle.
					if (!obstNeighborL->isConvex) {
						continue;
					}

					// since obstNeighborL is the only obstacle point we take into account, we set obstNeighborR to obstNeighborL
					obstNeighborR = obstNeighborL;

					// In this case, the VO (velocity obstacle) is represented as a cone because we are considering a single point (like the agent VO).
					// The directions of the legs (leftLegDirection and rightLegDirection) are determined by the agent's radius. These legs represent
					// the tangents from the agent's position to the obstacle point, accounting for the agent's clearance (its radius).
					//
					// For example, if the agent follows the 'leftLegDirection', it will move just to the right of the obstacle's left side (tangentially),
					// while maintaining clearance defined by its radius. 
					//
					// To calculate this, we offset 'relativePosition1' (the vector from the agent to the obstacle) by the agent's clearance. 
					// This is done by adjusting 'relativePosition1' in the direction of the tangent line of the agent's circle. The tangent directions
					// are determined based on the geometry of the circle (agent) relative to the obstacle point.
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vec2(relativePosition1.x * leg1 - relativePosition1.y * clearance.clearance, relativePosition1.x * clearance.clearance + relativePosition1.y * leg1) / distSq1;
					rightLegDirection = Vec2(relativePosition1.x * leg1 + relativePosition1.y * clearance.clearance, -relativePosition1.x * clearance.clearance + relativePosition1.y * leg1) / distSq1;
				}
				// This means that the agent is right of the right obstacle vertex AND that the disk of the agent falls partly behind the obstacle line.
				else if (s > 1.0f && distSqLine <= radiusSq) {

					if (!obstNeighborR->isConvex) {
						continue;
					}

					obstNeighborL = obstNeighborR;

					// The below calculations are a mirrored version of the calculations done in "if (s < 0.0f && distSqLine <= radiusSq) {...}"
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					leftLegDirection = Vec2(relativePosition2.x * leg2 - relativePosition2.y * clearance.clearance, relativePosition2.x * clearance.clearance + relativePosition2.y * leg2) / distSq2;
					rightLegDirection = Vec2(relativePosition2.x * leg2 + relativePosition2.y * clearance.clearance, -relativePosition2.x * clearance.clearance + relativePosition2.y * leg2) / distSq2;
				}
				// The agent is in front of the obstacle segment. This is similar to the previous calculations, with the only exception that we now create the tangent lines separately for the
				// left and right obstacle vertices.
				else {
					if (obstNeighborL->isConvex) 
					{
						const float leg1 = std::sqrt(distSq1 - radiusSq);
						leftLegDirection = Vec2(relativePosition1.x * leg1 - relativePosition1.y * clearance.clearance, relativePosition1.x * clearance.clearance + relativePosition1.y * leg1) / distSq1;
					}
					else 
					{
						// The left point is not convex, meaning there is no well-defined left tangent line that the agent can follow to avoid a collision.
						// In this case, we treat the obstacle segment itself as the left boundary for the VO. The left direction is set to align with the 
						// obstacle segment's direction (reversed), ensuring that any movement further to the right (except up to the right tangent of the 
						// obstacle) would result in a collision with the obstacle.
						leftLegDirection = segUnitDir * -1.0f;
					}

					if (obstNeighborR->isConvex) 
					{
						const float leg2 = std::sqrt(distSq2 - radiusSq);
						rightLegDirection = Vec2(relativePosition2.x * leg2 + relativePosition2.y * clearance.clearance, -relativePosition2.x * clearance.clearance + relativePosition2.y * leg2) / distSq2;
					}
					else 
					{
						// The right point is not convex, meaning there is no well-defined right tangent line that the agent can follow to avoid a collision.
						// The same logic applies for when the left point is not convex, only we set the rightLegDirection to point to the opposite side.
						rightLegDirection = segUnitDir;
					}
				}

				// Now we have calculated the left and right leg directions of the VO. 
				// However, our current left and right leg directions may point into the left resp. right (convex) neighbors
				// Let's adjust for these cases...

				const ObstacleVertex* leftNeighbor = obstNeighborL->prevObstacle;

				bool isLeftLegForeign = false;
				bool isRightLegForeign = false;

				// In the case below, the obstacle is convex, and the left VO leg initially points into the left neighbor of the obstacle.
				// This could cause the agent to choose a velocity that leads into a collision with the left neighbor. To prevent this,
				// we adjust the left leg of the VO to be parallel to the edge of the left neighboring obstacle.
				Vec2 leftNeighborDir = obstNeighborL->p - leftNeighbor->p;
				leftNeighborDir.Normalize();
				if (obstNeighborL->isConvex && Utility::MathUtility::Determinant(leftLegDirection, leftNeighborDir * -1.0f) >= 0.0f) {
					leftLegDirection = leftNeighborDir * -1.0f;
					isLeftLegForeign = true;
				}

				// ... and we apply the same logic to the right obstacle
				Vec2 rightNeighborDir = obstNeighborR->nextObstacle->p - obstNeighborR->p;
				rightNeighborDir.Normalize();
				if (obstNeighborR->isConvex && Utility::MathUtility::Determinant(rightLegDirection, rightNeighborDir) <= 0.0f) {
					rightLegDirection = rightNeighborDir;
					isRightLegForeign = true;
				}

				float obstLookaheadRecip = 1.0f / m_lookAheadObstacle;

				// calculate cutoff centers, aka the left- and right lower bounds of the VO
				const Vec2 leftCutoff = (obstNeighborL->p - position) * obstLookaheadRecip;
				const Vec2 rightCutoff = (obstNeighborR->p - position) * obstLookaheadRecip;
				const Vec2 cutoffVec = rightCutoff - leftCutoff;

				// now that we know the VO, we can project the current velocity on this VO 

				// we first check if we should project the velocity on the left or right cutoff circles.
				const float t = (obstNeighborL == obstNeighborR ? 0.5f : (Utility::MathUtility::Dot((velocity - leftCutoff), cutoffVec)) / Utility::MathUtility::SquaredLength(cutoffVec));
				const float tLeft = Utility::MathUtility::Dot((velocity - leftCutoff), leftLegDirection);
				const float tRight = Utility::MathUtility::Dot((velocity - rightCutoff), rightLegDirection);

				// we project the velocity on the left cutoff circle in two cases:
				// 1: the velocity lies in the lower-left part of the left cutoff circle that is the left corner of the VO
				// 2: there is only one cutoff-circle. in this case we project on the circle if the velocity lies in the
				//     lower part of the circle that forms the angled corner of the VO
				if ((t < 0.0f && tLeft < 0.0f) || (obstNeighborL == obstNeighborR && tLeft < 0.0f && tRight < 0.0f)) {
					Vec2 unitW = (velocity - leftCutoff).Normalized();

					Constraint c;
					Point pOnCircle = leftCutoff + unitW * obstLookaheadRecip * clearance.clearance;

					c.Init(pOnCircle, unitW);
					outConstraints.push_back(c);
					continue;
				}
				// the same logic applies for the right cutoff circle...
				else if (t > 1.0f && tRight < 0.0f) {
					Vec2 unitW = (velocity - rightCutoff).Normalized();

					Constraint c;
					Point pOnCircle = rightCutoff + unitW * obstLookaheadRecip * clearance.clearance;

					c.Init(pOnCircle, unitW);
					outConstraints.push_back(c);

					continue;
				}

				// now we know we should not project on the cutoff circles, but on either the left- or right bounding line of the VO, or the cutoff line (i.e. lower bound of the VO)
				// of course we choose the projection with the least distance
				const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstNeighborL == obstNeighborR) ? std::numeric_limits<float>::infinity() : Utility::MathUtility::SquaredLength(velocity - (leftCutoff + cutoffVec * t)));
				const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : Utility::MathUtility::SquaredLength(velocity - (leftCutoff + leftLegDirection * tLeft)));
				const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : Utility::MathUtility::SquaredLength(velocity - (rightCutoff + rightLegDirection * tRight)));

				// distance to cutoff line is shortest: project velocity on cutoff line
				if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {

					// remember that we don't actually need to calculate the closest point on the cutoff line to the velocity.
					// we just need to construct a constraint which consists of any point on the cutoff line and the correct normal.
					Vec2 normal = Utility::MathUtility::Left(segUnitDir * -1.0f);
					Point pOnCutoff = leftCutoff + normal * obstLookaheadRecip * clearance.clearance;

					Constraint c;
					c.Init(pOnCutoff, normal);
					outConstraints.push_back(c);

					continue;
				}
				// distance to left leg is shortest
				else if (distSqLeft <= distSqRight) {
					// if the left leg is foreign, then we rely on the constraint of the left neighboring obstacle and we skip this obstacle
					if (isLeftLegForeign) {
						continue;
					}

					Vec2 normal = Utility::MathUtility::Left(leftLegDirection);
					Point pOnLeftLeg = leftCutoff + normal * clearance.clearance * obstLookaheadRecip;

					Constraint c;
					c.Init(pOnLeftLeg, normal);
					outConstraints.push_back(c);

					continue;
				}
				// distance to right leg is shortest
				else {
					if (isRightLegForeign) {
						continue;
					}

					Vec2 normal = Utility::MathUtility::Right(rightLegDirection);
					Point pOnRightLeg = rightCutoff + normal * clearance.clearance * obstLookaheadRecip;

					Constraint c;
					c.Init(pOnRightLeg, normal);
					outConstraints.push_back(c);

					continue;
				}
			}

			outNObstacleConstraints = outConstraints.size();

			// Calculate agent constraints
			int nNeighbors = agentNeighbors.size();

			for (int i = 0; i < nNeighbors; i++)
			{
				const Entity& neighbor = agentNeighbors[i];
				const PositionComponent& nPosition = simulator->GetPositionData()[neighbor];
				const VelocityComponent& nVelocity = simulator->GetVelocityData()[neighbor];
				const ClearanceComponent& nClearance = simulator->GetClearanceData()[neighbor];

				Point VOPos((nPosition.x - position.x) / m_lookAheadAgent, (nPosition.y - position.y) / m_lookAheadAgent);
				float VOPosLength = Utility::MathUtility::Length(VOPos);
				float combinedRadius = nClearance.clearance + clearance.clearance;
				float VORadius = combinedRadius / m_lookAheadAgent;
				float VORadiusSqr = VORadius * VORadius;

				Vec2 relVel(velocity.x - nVelocity.dx, velocity.y - nVelocity.dy);
				Vec2 relPos(nPosition.x - position.x, nPosition.y - position.y);
				float relPosLength = Utility::MathUtility::Length(relPos);

				if (relPosLength < combinedRadius)
				{
					// neighbors are colliding. in the next timestep, agents should be collision free.

					// Vector from cutoff center to relative velocity.
					const Vec2& w = relVel - relPos / stepSize;
					const float wLength = w.Length();
					const Vec2 unitW = w / wLength;

					const Vec2& U = unitW * (combinedRadius / stepSize - wLength);

					Constraint c;
					c.Init(velocity + U * 0.5f, unitW);
					outConstraints.push_back(c);
					
					continue;
				}

				// calculate left and right leg of the VO. These legs are tangent to the VO circle. We can calculate the angle (relative to VOPos) using basic trigonometry.
				float tanAngleFactor = VORadius / VOPosLength;
				float tanHalfAngle = atan(tanAngleFactor);
				Vec2 VOLeftLeg = Utility::MathUtility::RotateVector(VOPos, tanHalfAngle); // note: positive angle is anti-clockwise rotation
				Vec2 VORightLeg = Utility::MathUtility::RotateVector(VOPos, -tanHalfAngle);

				// if the distance of the relative velocity to the VO circle centre is smaller than its radius, the relative velocity lies inside the VO.
				// if not, than the rel velocity only lies in the VO if it lies above the VO circle position
				float sqDistFromCircleCentre = Utility::MathUtility::SquareDistance(VOPos, relVel);
				bool liesBelowCircleMiddle = Utility::MathUtility::IsLeftOfVector(VOLeftLeg - VOPos, relVel - VOPos);

				if (liesBelowCircleMiddle)
				{
					// the constraint is the half-plane at the edge of the circle of the VO
					float distToEdge = VORadius - sqrtf(sqDistFromCircleCentre);
					
					Vec2 lineNormal = (relVel - VOPos);
					lineNormal.Normalize();
					Point pointOnLine = velocity + lineNormal * distToEdge * 0.5f;

					Constraint c;
					c.Init(pointOnLine, lineNormal);
					outConstraints.push_back(c);
				}
				else
				{
					const Vec2& leftPerpendicular = Utility::MathUtility::Left(VOPos) / VOPosLength;

					// find closest point to the VO edges. First determine whether RelVel lies closer to the left or right VO leg.
					bool closerToLeftLeg = Utility::MathUtility::Dot(leftPerpendicular, relVel) >= 0;
					if (closerToLeftLeg)
					{
						const Vec2& leftLegNormalized = VOLeftLeg.Normalized();
						const Vec2& U = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(leftLegNormalized, relVel) - relVel;

						Constraint c;
						c.Init(velocity + U * 0.5f, Utility::MathUtility::Left(leftLegNormalized));
						outConstraints.push_back(c);
					}
					else
					{
						const Vec2& rightLegNormalized = VORightLeg.Normalized();
						const Vec2& U = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(rightLegNormalized, relVel) - relVel;

						Constraint c;
						c.Init(velocity + U * 0.5f, Utility::MathUtility::Right(rightLegNormalized));
						outConstraints.push_back(c);
					}
				}
			}
		}

		// applies randomized linear programming to find the optimal velocity, given the ORCA constraints.
		// returns the number of constraints (if success) or the index of the constraint that failed.
		int ORCA::RandomizedLP(const std::vector<Constraint>& constraints, const Vec2& optVelocity, const float maxSpeed, bool useDirOpt, Vec2& outVelocity) const
		{

			// 1. make random permutation of constraints
			// 
			// For now: loop through constraints from 0..N.
			// 

			int nConstraints = constraints.size();
			
			if (useDirOpt)
			{
				// we optimize in the direction dirOpt. This option is required for 3D LP.
				// note that we assume that dirOpt is normalized!
				outVelocity = optVelocity * maxSpeed;
			}
			else if (Utility::MathUtility::Length(optVelocity) > maxSpeed) {

				// optimal velocity bigger than maximum allowed velocity: clamp to max velocity
				outVelocity = optVelocity.Normalized() * maxSpeed;
			}
			else
			{
				// initialize solution with preferred velocity
				outVelocity = optVelocity;
			}

			if (nConstraints == 0) return nConstraints;

			//std::cout << "starting LP for prefVel = (" << prefVel.x << ", " << prefVel.y << "). " << nConstraints << " constraints." << std::endl;

			for (int i = 0; i < nConstraints; i++)
			{
				const Constraint& h = constraints[i];
				// if current solution satisfies this constraint, we don't need to do anything
				if (h.Contains(outVelocity))
				{
					continue;
				}

				// the current solution does not satisfy the constraint.
				// according to LP theory, this means that the new solution (if it exists) lies on the bounding line
				// of the new half plane constraint.

				// First we test whether the new constraint h intersects with the maximum speed constraint M (circle)
				// There are 3 options...
				// 1. There are no intersections. Two options...
				//		A) M lies entirely in h, which means that we can ignore h as it does not affect our current solution.
				//		B) M lies entirely outside of h, which means there is no solution. We return an error.
				// 2. There is one intersection. Two options...
				//		A) M lies entirely in h, which means that we can ignore h as it does not affect our current solution.
				//		B) M lies entirely outside of h. This means that there is only 1 possible solution, namely the intersection between h and M.
				//		    Now we must check if that one solution satisfies all other constraints. If yes, return solution. If no, return error.
				// 3. There are two intersections. Calculate Left and Right (x-coordinates that bound the feasible region of possible solutions). 

				
				// TODO:
				// figure out why left = -dirPointDot - discrSqrt  and right=...
				// if you figured this out, fix code below, still doesn't work.
				// probably has something to do with parametrization ||Position(t)||^2 = Radius^2
				// 
				// 
				// We parameterize position(t) = line.point + t * line.direction
				// We find the intersection with a circle by solving the following equation: ||position(t)||^2 = maxSpeed^2
				// When we write this in the form at^2 + bt + c = 0, and we know discriminant = b^2 - 4ac,
				// then discriminant = (line.point * line.direction)^2 + maxSpeed^2 - ||line.point||^2
				Vec2 dir = Utility::MathUtility::Right(h.Normal());
				float dirPointDot = Utility::MathUtility::Dot(dir, h.PointOnLine());
				float discriminant = dirPointDot * dirPointDot + maxSpeed * maxSpeed - Utility::MathUtility::Dot(h.PointOnLine(), h.PointOnLine());

				// CASE 1: no intersections or CASE 2: one intersection
				if (discriminant <= 0.0f)
				{
					// program failed at index i
					return i;
				}

				// CASE 3: two intersections
				else if (discriminant > 0.0f)
				{
					// We can rewrite ||position(t)||^2 = maxSpeed^2 to at^2 + bt + c = 0 form
					// Now we know the values of a, b and c we can apply the ABC-formula to get tLeft and tRight...
					float discriminantSqrt = sqrtf(discriminant);
					float left = -dirPointDot - discriminantSqrt;
					float right = -dirPointDot + discriminantSqrt;

					// loop through previous constraints to update Left and Right
					// we know the optimal solution lies on h, so we check the line-line intersections between h and all previous
					// constraints. This provides us with a feasible velocity range [left..right] on the edge of constraint h. 
					// line-line interesection source: https://stackoverflow.com/a/565282/2843415
					for (int j = 0; j < i; j++)
					{
						const Constraint& h_j = constraints[j];

						float denominator = Utility::MathUtility::Determinant(Utility::MathUtility::Right(h.Normal()), Utility::MathUtility::Right(h_j.Normal()));
						float numerator = Utility::MathUtility::Determinant(Utility::MathUtility::Right(h_j.Normal()), h.PointOnLine() - h_j.PointOnLine());

						// check if the two constraints are parallel
						if (std::fabs(denominator) <= Utility::EPSILON) {
							if (numerator < 0.0f) {
								return i;
							}
							else {
								continue;
							}
						}

						const float t = numerator / denominator;

						if (denominator >= 0.0f) {
							// constraint j is the new bound on the right
							right = std::min(right, t);
						}
						else {
							// constraint j is the new bound on the left
							left = std::max(left, t);
						}

						if (left > right) {
							return i;
						}
					}

					// now that we've updated Left/Right with all constraints so far, we can update the new optimal velocity by projecting the preferred velocity
					//  on the line from Left to Right

					// we optimize in the provided direction, so we take the left or right extreme (dependent on the direction of the velocity
					//  relative to the constraint).
					if (useDirOpt)
					{
						if (Utility::MathUtility::Dot(optVelocity, Utility::MathUtility::Right(h.Normal())) > 0)
						{
							outVelocity = h.PointOnLine() + Utility::MathUtility::Right(h.Normal()) * right;
						}
						else
						{
							outVelocity = h.PointOnLine() + Utility::MathUtility::Right(h.Normal()) * left;
						}
					}
					// we optimize the velocity by taking the closest point on the line of constraint h in the range [left..right]
					else
					{
						float t = Utility::MathUtility::Dot(Utility::MathUtility::Right(h.Normal()), (optVelocity - h.PointOnLine()));

						if (t < left) {
							outVelocity = h.PointOnLine() + Utility::MathUtility::Right(h.Normal()) * left;
						}
						else if (t > right) {
							outVelocity = h.PointOnLine() + Utility::MathUtility::Right(h.Normal()) * right;
						}
						else {
							outVelocity = h.PointOnLine() + Utility::MathUtility::Right(h.Normal()) * t;
							int pause = 0;
						}
					}
				}
			}

			return nConstraints;
		}
	
		// applies randomized linear programming to find the optimal velocity, given the ORCA constraints. The difference with ORCA::RandomizedLP(..) is that ORCA::RandomizedLP3D(..)
		//  relaxes the (non-static) obstacle constraints to find the "best possible" solution. This best possible solution does not completely avoid collision, but finds the velocity 
		//  that minimizes the collision.
		void ORCA::RandomizedLP3D(int nObstacleConstraints, const std::vector<Constraint>& constraints, const float maxSpeed, int failedIndex, Vec2& outVelocity) const
		{
			// loop through constraints (starting from failedIndex)

			// maximum distance to constraints. note that penetration distance is a negative number.
			float maxPenetration = 0.0f;

			int totalConstraints = constraints.size();
			for (int i = failedIndex; i < totalConstraints; i++)
			{
				const Constraint& ci = constraints[i];

				// if the optimized velocity violates the current constraint less than the maximum constraint penetration, we can ignore this constraint
				//if (Utility::MathUtility::Dot(ci.Normal(), outVelocity - ci.PointOnLine()) >= maxPenetration) continue;
				Vec2 dir = Utility::MathUtility::Right(ci.Normal());
				if (Utility::MathUtility::Determinant(dir, ci.PointOnLine() - outVelocity) <= maxPenetration) continue;

				// the optimized velocity violates the current constraint more the the max constraint penetration: optimize for this constraint.

				// add static constraints as they are, since we don't want to project static constraints.
				std::vector<Constraint> projectedConstraints(nObstacleConstraints); 
				for (int i = 0; i < nObstacleConstraints; i++) projectedConstraints[i] = constraints[i];

				// loop through all constraints so far, starting from first agent constraint
				for (int j = nObstacleConstraints; j < i; j++)
				{
					const Constraint& cj = constraints[j];
					Constraint newC;

					float determinant = Utility::MathUtility::Determinant(Utility::MathUtility::Right(ci.Normal()), Utility::MathUtility::Right(cj.Normal()));

					Vec2 newCPoint;

					// check if constraint lines are parallel
					if (std::fabs(determinant) <= Utility::EPSILON)
					{
						// constraint lines are parallel
						
						// constraint lines point in the same direction: no need to optimize
						if (Utility::MathUtility::Dot(ci.Normal(), cj.Normal()) > 0)
						{
							continue;
						}

						// constraint lines point in the opposite direction: optimize by creating an equidistant line, which in this case is the bisector
						newCPoint = (ci.PointOnLine() + cj.PointOnLine()) * 0.5f;
					}
					else
					{
						// constraint lines are not parallel

						// Our goal is to optimize the new velocity in the direction of ci, where we want to minimize the maximum penetration to all other constraints.
						// In this case, we want to minimize the maximum penetration to cj. We do this by creating a new constraint which edge is equidistant from ci 
						// and cj. If we project outVelocity anywhere on this edge, we know we minimized the maximum penetration of ci and cj. To create this constraint,
						// we want to make sure its edge goes through the intersection point of ci and cj. The direction of this constraint should be the relative velocity 
						// (i.e. cj.dir - ci.dir). This  relative velocity will push outVelocity from its optimization direction (ci) towards the other constraint (cj), 
						// and it will result in an edge that is equidistant from ci and cj.
						float t = Utility::MathUtility::Determinant(Utility::MathUtility::Right(cj.Normal()), ci.PointOnLine() - cj.PointOnLine()) / determinant;

						newCPoint = ci.PointOnLine() + Utility::MathUtility::Right(ci.Normal()) * t;
					}

					newC.Init(newCPoint, (cj.Normal() - ci.Normal()).Normalized());
					projectedConstraints.push_back(newC);
				}

				const Vec2 tempVel = outVelocity;
				if (RandomizedLP(projectedConstraints, ci.Normal(), maxSpeed, true, outVelocity) < projectedConstraints.size())
				{
					// solving the linear program for the projected constraints should always return a solution, so this should not happen. If it happens, it is due to
					// floating point errors and we revert back to the previous solution.
					outVelocity = tempVel;
				}

				//maxPenetration = Utility::MathUtility::Dot(ci.Normal(), outVelocity - ci.PointOnLine());
				maxPenetration = Utility::MathUtility::Determinant(dir, ci.PointOnLine() - outVelocity);
			}
		}
}

}