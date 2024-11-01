#include "RVO.h"

#include "Simulator.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"

#include <cmath>

namespace ECM {

	namespace Simulation {

		void RVO::GetRVOVelocity(Simulator* simulator, const Entity& entity, float stepSize, float maxSpeed, int nNeighbors, Vec2& outVelocity)
		{
			std::vector<Entity> neighbors;
			simulator->FindNNearestNeighbors(entity, nNeighbors, neighbors);

			std::vector<Constraint> constraints(neighbors.size());
			int nConstraints;
			GenerateConstraints(simulator, entity, neighbors, stepSize, nConstraints, constraints);

			const VelocityComponent& prefVelComp = simulator->GetPreferredVelocityData()[entity];
			Vec2 prefVel(prefVelComp.dx, prefVelComp.dy);

			int failedIndex = RandomizedLP(nConstraints, constraints, prefVel, maxSpeed, false, outVelocity);
			if (failedIndex < nConstraints)
			{
				RandomizedLP3D(nConstraints, constraints, maxSpeed, failedIndex, outVelocity);
			}
		}



		/*
		* TODO (2024-10-24)
		* - check the usage of simstepTime and lookahead. It looks like agents are not fast enough with collision avoidance.
		* - add static obstacle constraints
		*/
		// generates the ORCA constraints
		void RVO::GenerateConstraints(Simulator* simulator, const Entity& entity, const std::vector<Entity>& neighbors, float stepSize, int& outNConstraints, std::vector<Constraint>& outConstraints)
		{
			int nNeighbors = neighbors.size();
			int counter = 0;

			const PositionComponent& position = simulator->GetPositionData()[entity];
			const VelocityComponent& velocity = simulator->GetVelocityData()[entity];
			const ClearanceComponent& clearance = simulator->GetClearanceData()[entity];

			// for each neighboring agent, generate the VO
			for (int i = 0; i < nNeighbors; i++)
			{
				const Entity& neighbor = neighbors[i];
				const PositionComponent& nPosition = simulator->GetPositionData()[neighbor];
				const VelocityComponent& nVelocity = simulator->GetVelocityData()[neighbor];
				const ClearanceComponent& nClearance = simulator->GetClearanceData()[neighbor];

				Point VOPos((nPosition.x - position.x) / m_lookAhead, (nPosition.y - position.y) / m_lookAhead);
				float VOPosLength = Utility::MathUtility::Length(VOPos);
				float combinedRadius = nClearance.clearance + clearance.clearance;
				float VORadius = combinedRadius / m_lookAhead;
				float VORadiusSqr = VORadius * VORadius;

				Vec2 relVel(velocity.dx - nVelocity.dx, velocity.dy - nVelocity.dy);
				Vec2 relPos(nPosition.x - position.x, nPosition.y - position.y);
				float relPosLength = Utility::MathUtility::Length(relPos);

				if (relPosLength < combinedRadius)
				{
					// neighbors are colliding. in the next timestep, agents should be collision free.

					/* Vector from cutoff center to relative velocity. */
					const Vec2& w = relVel - relPos / stepSize;
					const float wLength = w.Length();
					const Vec2 unitW = w / wLength;

					const Vec2& U = unitW * (combinedRadius / stepSize - wLength);

					outConstraints[counter].Init(Vec2(velocity.dx, velocity.dy) + U * 0.5f, unitW);

					counter++;

					continue;
				}

				// calculate left and right leg of the VO. These legs are tangent to the VO circle. We can calculate the angle (relative to VOPos) using basic trigonometry.
				float tanAngleFactor = VORadius / VOPosLength;
				float tanHalfAngle = atan(tanAngleFactor);
				Vec2 VOLeftLeg = Utility::MathUtility::RotateVector(VOPos, tanHalfAngle); // note: positive angle is anti-clockwise rotation
				Vec2 VORightLeg = Utility::MathUtility::RotateVector(VOPos, -tanHalfAngle);

				// Rel velocity (RelVel) ligt in VO wanneer:
				// 1. RelVel tussen de twee legs ligt
				// 2. De afstand van de cirkel tot RelVel kleiner is dan de cirkel radius
				//    OF RelVel ligt boven de relatieve middellijn van de cirkel

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
					Point pointOnLine = Point(velocity.dx, velocity.dy) + lineNormal * distToEdge * 0.5f;

					outConstraints[counter].Init(pointOnLine, lineNormal);

					counter++;
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

						outConstraints[counter].Init(Vec2(velocity.dx, velocity.dy) + U * 0.5f, Utility::MathUtility::Left(leftLegNormalized));

						counter++;
					}
					else
					{
						const Vec2& rightLegNormalized = VORightLeg.Normalized();
						const Vec2& U = Utility::MathUtility::GetClosestPointOnLineThroughOrigin(rightLegNormalized, relVel) - relVel;

						outConstraints[counter].Init(Vec2(velocity.dx, velocity.dy) + U * 0.5f, Utility::MathUtility::Right(rightLegNormalized));

						counter++;
					}
				}
			}

			outNConstraints = counter;
		}

		// applies randomized linear programming to find the optimal velocity, given the ORCA constraints.
		// returns the number of constraints (if success) or the index of the constraint that failed.
		int RVO::RandomizedLP(int nConstraints, const std::vector<Constraint>& constraints, const Vec2& optVelocity, const float maxSpeed, bool useDirOpt, Vec2& outVelocity) const
		{

			// 1. make random permutation of constraints
			// 
			// For now: loop through constraints from 0..N.
			// 
			
			if (useDirOpt)
			{
				// we optimize in the direction dirOpt. This option is required for 3D LP.
				// note that we assume that dirOpt is normalized!
				outVelocity = optVelocity * maxSpeed;
			}
			else if (Utility::MathUtility::Length(optVelocity) > maxSpeed) {
				// optimal velocity bigger than maximum allowed velocity; correct

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


				float a = powf(h.Slope(), 2) + 1;
				float b = 2 * h.Slope() * h.YIntercept();

				
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

				//if (h.IsVertical())
				//{
				//	// in case of a vertical line, we only need to know whether the vertical line crosses or touches the circle
				//	// we don't calculate a discriminant, but rather encode the number of intersections (<0 = no inersections, >0 = two intersections, ==0 = one intersection).
				//
				//	discriminant = (maxSpeed - abs(h.XIntercept()));
				//}
				//else
				//{
				//	//discriminant = 4 * (powf(maxSpeed, 2) * (powf(h.Slope(), 2) + 1) - powf(h.YIntercept(), 2));
				//	discriminant = powf(maxSpeed, 2) * drSq - powf(det, 2.0f);
				//}

				// CASE 1: no intersections
				if (discriminant < 0.0f)
				{
					//std::cout << "CASE 1: no intersections" << std::endl;
					//
					//// option 1: M is contained in h. We can ignore this constraint.
					//Vec2 vecToCircle = Point() - h.PointOnLine();
					//if (Utility::MathUtility::Dot(vecToCircle, h.Normal()) > 0.0f) continue;
					//
					//// option 2: M is not contained in h. We must return an error.
					//std::cout << "M is not contained in h. Could not find RVO solution!" << std::endl;
					return i;
				}

				// CASE 2: only one intersection
				//else if (fabs(discriminant) < Utility::EPSILON)
				//{
				//	//std::cout << "CASE 2: only one intersection" << std::endl;
				//
				//	// option 1: M is contained in h. We can ignore this constraint.
				//	Vec2 vecToCircle = Point() - h.PointOnLine();
				//	if (Utility::MathUtility::Dot(vecToCircle, h.Normal()) > 0.0f) continue;
				//
				//	// option 2: M is not contained in h. There is only 1 possible solution, namely the intersection point.
				//	// we can now loop over all other constraints and see if the point satisfies those constraints. If so,
				//	// then we can return this point as the solution. If there is only 1 constraint which the point does not
				//	// satisfy, then we must return an error.
				//
				//	Vec2 tempVelocity = outVelocity;
				//
				//	if (h.IsVertical())
				//	{
				//		outVelocity = Vec2(h.XIntercept(), 0.0f); // vertical tangent of circle with origin (0,0) must be at y=0.
				//	}
				//	else
				//	{
				//		// we know the x coordinate of the intersections can be calculated as x = (-b +- sqrt(D)) / 2a.
				//		// since D = 0, this becomes x = -b / 2a
				//		//float x = -b / 2 * a;
				//		//float y = h.Slope() * x + h.YIntercept();
				//
				//		float t = -dirPointDot;
				//		outVelocity = h.PointOnLine() + dir * t;
				//	}
				//
				//	// lastly, we check if the new velocity is satisfied by all other constraints
				//	for (int j = 0; j < constraints.size(); j++)
				//	{
				//		if (!constraints[j].Contains(outVelocity))
				//		{
				//			std::cout << "NO SOLUTION: 1 intersection with max speed circle" << std::endl;
				//
				//			// revert back to the last feasible solution
				//			outVelocity = tempVelocity;
				//			return i;
				//		}
				//	}
				//
				//	return nConstraints;
				//}

				// CASE 3: two intersections
				else if (discriminant > 0.0f)
				{
					//std::cout << "CASE 3: two intersections" << std::endl;

					// We can rewrite ||position(t)||^2 = maxSpeed^2 to at^2 + bt + c = 0 form
					// Now we know the values of a, b and c we can apply the ABC-formula to get tLeft and tRight...
					float discriminantSqrt = sqrtf(discriminant);
					float left = -dirPointDot - discriminantSqrt;
					float right = -dirPointDot + discriminantSqrt;

					// if h is vertical, than we cannot parameterize for x. In this case we parameterize for y. We calculate the intersection point using
					//  pythagoras theorem.
					//if (h.IsVertical())
					//{
					//	// assuming half-plane points to the right, i.e. normal = (1, 0)...
					//	left = sqrtf(powf(maxSpeed, 2.0f) - powf(h.XIntercept(), 2.0f));
					//	right = -left;
					//}
					//else
					//{
					//	//left = (-b - sqrt(discriminant)) / (2 * a);
					//	//right = (-b + sqrt(discriminant)) / (2 * a);
					//
					//
					//}


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
	
		// applies randomized linear programming to find the optimal velocity, given the ORCA constraints. The difference with RVO::RandomizedLP(..) is that RVO::RandomizedLP3D(..)
		//  relaxes the (non-static) obstacle constraints to find the "best possible" solution. This best possible solution does not completely avoid collision, but finds the velocity 
		//  that minimizes the collision.
		void RVO::RandomizedLP3D(int nConstraints, const std::vector<Constraint>& constraints, const float maxSpeed, int failedIndex, Vec2& outVelocity) const
		{
			// loop through constraints (starting from failedIndex)

			// maximum distance to constraints. note that penetration distance is a negative number.
			float maxPenetration = 0.0f;

			for (int i = failedIndex; i < nConstraints; i++)
			{
				const Constraint& ci = constraints[i];

				// if the optimized velocity violates the current constraint less than the maximum constraint penetration, we can ignore this constraint
				//if (Utility::MathUtility::Dot(ci.Normal(), outVelocity - ci.PointOnLine()) >= maxPenetration) continue;
				Vec2 dir = Utility::MathUtility::Right(ci.Normal());
				if (Utility::MathUtility::Determinant(dir, ci.PointOnLine() - outVelocity) <= maxPenetration) continue;

				// the optimized velocity violates the current constraint more the the max constraint penetration: optimize for this constraint.

				std::vector<Constraint> projectedConstraints; // TODO: add static constraints as they are, since we don't want to project static constraints.

				// loop through all constraints so far
				for (int j = 0; j < i; j++)
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
				if (RandomizedLP(projectedConstraints.size(), projectedConstraints, ci.Normal(), maxSpeed, true, outVelocity) < projectedConstraints.size())
				{
					// solving the linear program for the projected constraints should always return a solution, so this should not happen. If it happens, it is due to
					// floating point errors and we revert back to the previous solution.
					outVelocity = tempVel;
				}

				//maxPenetration = Utility::MathUtility::Dot(ci.Normal(), outVelocity - ci.PointOnLine());
				maxPenetration = Utility::MathUtility::Determinant(dir, ci.PointOnLine() - outVelocity);
				if (maxPenetration < -Utility::EPSILON)
				{
					std::cout << "this should not happen" << std::endl;
				}
			}
		}
}

}