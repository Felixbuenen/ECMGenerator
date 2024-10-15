#include "RVO.h"

#include "Simulator.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"

#include <cmath>

namespace ECM {

	// ----------------
	// TODO 2024-09-03:
	// - first tests with ORCA done. asin(radius/posLength) returns > 1.0f, which means that 
	// - next step: test ORCA with dynamic obstacles only!
	// - next next step: add static obstacle avoidance
	// ----------------

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
			if (!RandomizedLP(nConstraints, constraints, prefVel, maxSpeed, outVelocity))
			{
				std::cout << "no avoidance force could be applied" << std::endl;
			}
		}

		/*
		* TODO (2024-10-07):
		* - Belangrijk om te kijken naar timeStep (delta simulatie tijd) en timeHorizon. TimeHorizon fungeert als een lookahead.
		* - Nu zijn agents namelijk iedere keer te laat, aka ze gaan door elkaar heen. Of ze worden niet snel genoeg of niet sterk genoeg gecorrigeerd.
		* - LookAhead kan het oplossen. Daarnaast moet ik nog een keer goed kijken of de juiste parameters (lookahead, timestep, dt..) gebruikt worden.. wordt
		*    de agent wel voldoende geupdatet? of is de update te zwak, waardoor agents alsnog op elkaar af gaan?
		* - OOK handig om bij spawn agents te testen of agents niet in het begin al overlappen, anders werkt orca ook niet
		*/
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

					outConstraints[counter].Init(Vec2(velocity.dx, velocity.dy) + U * 0.5f, Utility::MathUtility::Right(unitW));

					counter++;

					continue;
				}

				const Vec2& leftPerpendicular = Utility::MathUtility::Left(VOPos) / VOPosLength;

				// calculate left and right leg of the VO. These legs are tangent to the VO circle. We can calculate the angle (relative to VOPos) using basic trigonometry.
				float tanAngleFactor = VORadius / VOPosLength;
				float tanAngle = asinf(tanAngleFactor);
				Vec2 VOLeftLeg = Utility::MathUtility::RotateVector(VOPos, tanAngle); // note: positive angle is anti-clockwise rotation
				Vec2 VORightLeg = Utility::MathUtility::RotateVector(VOPos, -tanAngle);

				// Rel velocity (RelVel) ligt in VO wanneer:
				// 1. RelVel tussen de twee legs ligt
				// 2. De afstand van de cirkel tot RelVel kleiner is dan de cirkel radius
				//    OF RelVel ligt boven de relatieve middellijn van de cirkel

				// if the distance of the relative velocity to the VO circle centre is smaller than its radius, the relative velocity lies inside the VO.
				// if not, than the rel velocity only lies in the VO if it lies above the VO circle position
				float sqDistFromCircleCentre = Utility::MathUtility::SquareDistance(VOPos, relVel);
				bool liesBelowCircleMiddle = Utility::MathUtility::IsLeftOfVector(VOLeftLeg - VOPos, relVel - VOPos);

				// --------
				// At this point we know the point lies within the VO.
				// Now we calculate the constraint that is imposed by this VO.
				// --------

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


		bool RVO::RandomizedLP(int nConstraints, const std::vector<Constraint>& constraints, const Vec2& prefVel, const float maxSpeed, Vec2& outVelocity) const
		{

			// 1. make random permutation of constraints
			// 
			// For now: loop through constraints from 0..N.
			// 
			
			// note we add the speed constraint using a circle of radius "Speed" around the agent's position

			// initialize solution with preferred velocity
			outVelocity = prefVel;

			if (nConstraints == 0) return true;

			std::cout << std::endl;
			std::cout << "starting LP for prefVel = (" << prefVel.x << ", " << prefVel.y << "). " << nConstraints << " constraints." << std::endl;

			for (int i = 0; i < nConstraints; i++)
			{
				const Constraint& h = constraints[i];
				// if current solution satisfies this constraint, we don't need to do anything
				if (h.Contains(outVelocity))
				{
					std::cout << "constraint " << i << " satisfied. Continuing..." << std::endl;
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

				
				// We know the following formulas for the line h and circle M at origin:
				// Line:	slope * x + yIntersept = y
				// Circle:	x^2 + y^2 = maxSpeed^2 (note that maxSpeed = the radius of the circle)
				// If we want to find the intersection point(s), we can write the equation line = circle in the form ax^2 + bx + c = 0. This allows us to
				//  calculate the discriminant that we need to determine the number of intersection points. We get...
				//		
				//		(slope^2 + 1) * x^2 + 2 * slope * yIntercept * x + yIntercept^2 - maxSpeed^2 = 0
				// 
				// So we get the following coefficients...
				// a = (slope^2 + 1)
				// b = 2 * slope * yIntercept
				// c = yIntercept^2 - maxSpeed^2

				float a = powf(h.Slope(), 2) + 1;
				float b = 2 * h.Slope() * h.YIntercept();

				// discriminant = b^2 - 4ac
				//				= 4 * (maxSpeed^2 * (slope^2 + 1) - yIntercept^2) 
				float discriminant;
				if (h.IsVertical())
				{
					// in case of a vertical line, we only need to know whether the vertical line crosses or touches the circle
					// we don't calculate a discriminant, but rather encode the number of intersections (-1 = no inersections, 1 = two intersections, 0 = one intersection).

					discriminant = (maxSpeed - abs(h.XIntercept()));
				}
				else
				{
					discriminant = 4 * (powf(maxSpeed, 2) * (powf(h.Slope(), 2) + 1) - powf(h.YIntercept(), 2));
				}

				// CASE 1: no intersections
				if (discriminant < 0.0f)
				{
					std::cout << "CASE 1: no intersections" << std::endl;

					// option 1: M is contained in h. We can ignore this constraint.
					Vec2 vecToCircle = Point() - h.PointOnLine();
					if (Utility::MathUtility::Dot(vecToCircle, h.Normal()) > 0.0f) continue;

					// option 2: M is not contained in h. We must return an error.
					std::cout << "Could not find RVO solution!" << std::endl;
					return false;
				}

				// CASE 2: only one intersection
				else if (discriminant < Utility::EPSILON && discriminant > -Utility::EPSILON)
				{
					std::cout << "CASE 2: only one intersection" << std::endl;

					// option 1: M is contained in h. We can ignore this constraint.
					Vec2 vecToCircle = Point() - h.PointOnLine();
					if (Utility::MathUtility::Dot(vecToCircle, h.Normal()) > 0.0f) continue;

					// option 2: M is not contained in h. There is only 1 possible solution, namely the intersection point.
					// we can now loop over all other constraints and see if the point satisfies those constraints. If so,
					// then we can return this point as the solution. If there is only 1 constraint which the point does not
					// satisfy, then we must return an error.
					if (h.IsVertical())
					{
						outVelocity = Vec2(h.XIntercept(), 0.0f); // vertical tangent of circle with origin (0,0) must be at y=0.
					}
					else
					{
						// we know the x coordinate of the intersections can be calculated as x = (-b +- sqrt(D)) / 2a.
						// since D = 0, this becomes x = -b / 2a
						float x = -b / 2 * a;
						float y = h.Slope() * x + h.YIntercept();
						outVelocity = Vec2(x, y);
					}

					// lastly, we check if the new velocity is satisfied by all other constraints
					for (int j = 0; j < constraints.size(); j++)
					{
						if (!constraints[j].Contains(outVelocity))
						{
							std::cout << "NO SOLUTION: 1 intersection with max speed circle" << std::endl;
							return false;
						}
					}

					return true;
				}

				// CASE 3: two intersections
				else if (discriminant > 0.0f)
				{
					std::cout << "CASE 3: two intersections" << std::endl;

					// calculate Left and Right: the x-coordinates of the left-most and right-most point of the feasible region.
					// this is determined by the two intersection points between h and M
					
					float left, right;
					// if h is vertical, than we cannot parameterize for x. In this case we parameterize for y. We calculate the intersection point using
					//  pythagoras theorem.
					if (h.IsVertical())
					{
						// assuming half-plane points to the right, i.e. normal = (1, 0)...
						left = sqrtf(powf(maxSpeed, 2.0f) - powf(h.XIntercept(), 2.0f));
						right = -left;
					}
					else
					{
						left = (-b - sqrt(discriminant)) / (2 * a);
						right = (-b + sqrt(discriminant)) / (2 * a);
					}

					// loop through previous constraints to update Left and Right
					for (int j = 0; j < i; j++)
					{
						const Constraint& h_j = constraints[j];

						// first check if the two constraints are parallel. in this case there are no intersections and we need to ensure
						//  that the current solution is completely contained in the constraint
						if (h_j.Slope() > (h.Slope() - Utility::EPSILON) && h_j.Slope() < (h.Slope() + Utility::EPSILON))
						{
							// check if h contained in h_j. If so, we can ignore h_j as it does not impose new constraints.
							//  if not, than there exists no solution and we can abort the LP algorithm.
							if(h.Contains(h_j.PointOnLine()))
							{
								continue;
							}
							else
							{
								std::cout << "NO SOLUTION: parallel lines" << std::endl;
								return false;
							}
						}
						// constraints are not parallel: calculate intersection point and update Left/Right
						else
						{
							float intersect = 0.0f;
							if (h.IsVertical())
							{
								intersect = h_j.Slope() * h.XIntercept() + h_j.YIntercept();

							}
							else if (h_j.IsVertical())
							{
								intersect = h.Slope() * h_j.XIntercept() + h.YIntercept();
							}
							else
							{
								// find intersection point by solving line intersection equation:
								// m1*x + b1 = m2*x + b2
								intersect = (h_j.YIntercept() - h.YIntercept()) / (h.Slope() - h_j.Slope());
							}

							// if the normal of h_j points to the left of the normal of h, than the intersection becomes the new right bound.
							//  else it becomes the new left bound.
							Vec2 diff = h_j.Normal() - h.Normal();
							if (diff.x < 0.0f)
							{
								right = intersect;
							}
							else
							{
								left = intersect;
							}

							if (left > right)
							{
								std::cout << "NO SOLUTION" << std::endl;
								return false;
							}
						}
					}
					// now that we've updated Left/Right with all constraints so far, we can update the new optimal velocity by projecting the preferred velocity
					//  on the line from Left to Right
					if (h.IsVertical())
					{
						outVelocity = Utility::MathUtility::GetClosestPointOnSegment(prefVel, Segment(h.XIntercept(), left, h.XIntercept(), right));
					}
					else
					{
						float yLeft = h.Slope() * left + h.YIntercept();
						float yRight = h.Slope() * right + h.YIntercept();
						outVelocity = Utility::MathUtility::GetClosestPointOnSegment(prefVel, Segment(left, yLeft, right, yRight));
					}
				}
			}

			std::cout << "ended with velocity (" << outVelocity.x << ", " << outVelocity.y << ")" << std::endl;


			return true;
		}
	}

}