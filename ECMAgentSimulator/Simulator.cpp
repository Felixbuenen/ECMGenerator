#include "Simulator.h"

#include "ECM.h"
#include "UtilityFunctions.h"
#include "ECMPathPlanner.h"
#include "Environment.h"
#include "ECMDataTypes.h"

#include <math.h>

namespace ECM {

	namespace Simulation {

		void Simulator::Initialize()
		{
			m_LastEntityIdx = -1;
			m_ActiveAgents = new bool[m_MaxNumEntities];

			// initialize data
			m_Entities = new Entity[m_MaxNumEntities];
			m_Positions = new PositionComponent[m_MaxNumEntities];
			//m_Goals = new PositionComponent[m_MaxNumEntities];
			m_Velocities = new VelocityComponent[m_MaxNumEntities];
			m_Clearances = new ClearanceComponent[m_MaxNumEntities];
			m_Paths = new PathComponent[m_MaxNumEntities];

			for (int i = 0; i < m_MaxNumEntities; i++)
			{
				m_Entities[i] = i;

				// set sub-arrays to nullptrs as default
				// if a path component is not assigned, and we delete it on cleanup, 
				// we can't delete[] the path.X and path.Y data (as it is not assigned).
				// on cleanup, we explicitly check for nullptr's to prevent this.
				m_Paths[i].x = nullptr;
				m_Paths[i].y = nullptr;
			}

			printf("SIMULATOR: Data for %d agents was created.\n", m_MaxNumEntities);
		}

		void Simulator::ClearSimulator()
		{
			// delete data
			for (int i = 0; i < m_MaxNumEntities; i++)
			{
				if (m_Paths[i].x != nullptr)
				{
					delete[] m_Paths[i].x;
					m_Paths[i].x = nullptr;
				}
				if (m_Paths[i].y != nullptr)
				{
					delete[] m_Paths[i].y;
					m_Paths[i].y = nullptr;
				}
			}

			delete[] m_Entities;
			delete[] m_Positions;
			delete[] m_Velocities;
			delete[] m_Paths;
			delete[] m_ActiveAgents;
			delete[] m_Clearances;
			//delete[] m_Goals;

			printf("SIMULATOR: Data was destroyed.\n");
		}

		int Simulator::SpawnAgent(const Point& start, const Point& goal, float clearance, float preferredSpeed)
		{
			int idx = m_freeEntitySpaces.top();
			m_freeEntitySpaces.pop();
			m_LastEntityIdx = m_LastEntityIdx < idx ? idx : m_LastEntityIdx;

			// set parameters
			m_Positions[idx].x = start.x;
			m_Positions[idx].y = start.y;
			m_Velocities[idx].dx = 0.0f;
			m_Velocities[idx].dy = 0.0f;
			m_Clearances[idx].clearance = clearance;

			m_ActiveAgents[idx] = true;

			// plan path
			const float preferredAddClearance = 10.0f;
			PathPlanning::Corridor dummy;
			std::vector<Segment> portal;
			PathPlanning::Path path;
			m_Planner->GetPath(*m_Environment, start, goal, clearance, preferredAddClearance, dummy, portal, path);
			PathComponent& pComponent = m_Paths[idx];
			pComponent.x = new float[(int)path.size()];
			pComponent.y = new float[(int)path.size()];
			pComponent.numPoints = path.size();
			pComponent.currentIndex = 0;
			for (int j = 0; j < path.size(); j++)
			{
				pComponent.x[j] = path[j].x;
				pComponent.y[j] = path[j].y;
			}

			return idx;
		}

		void Simulator::DestroyAgent(int idx)
		{
			m_ActiveAgents[idx] = false;
			m_freeEntitySpaces.push(idx);
		}

		void Simulator::Update(float dt)
		{
			UpdateMaxAgentIndex();

			UpdateVelocitySystem(dt);
			UpdatePositionSystem(dt);
		}

		void Simulator::UpdateMaxAgentIndex()
		{
			int emptyCounter = 0;
			for (int i = m_LastEntityIdx; i >= 0; i--)
			{
				if (m_ActiveAgents[i]) break;

				emptyCounter++;
			}

			m_LastEntityIdx = m_LastEntityIdx - emptyCounter;
		}

		void Simulator::UpdatePositionSystem(float dt)
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				// note that currently this is redundant, but this method is clean for when we want to have different entities.
				// likely we don't have different entities, so maybe just use i for indexation.
				const Entity& e = m_Entities[i];

				const VelocityComponent& vel = m_Velocities[e];
				PositionComponent& pos = m_Positions[e];


				pos.x += vel.dx * dt;
				pos.y += vel.dy * dt;
			}
		}

		void Simulator::UpdateVelocitySystem(float dt)
		{
			// Let's start simple:
			// 1. calculate future position (velocity * lookAhead)
			// 2. iterate through all path segments to find the segment where the future position lies closest
			// 3. project the future position on this line segment. this is the attraction point.

			const float reachedRadius = 2.5f;
			const float arrivalRadius = 50.0f;
			const float mass = 1.0f;
			const float massRecip = 1.0f / mass;
			const float speed = 30.0f;
			const float attractionLookAheadMultiplier = 0.5f;

			// force weights
			const float pathFollowSteeringWeight = 1.0f;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				// note that currently this is redundant, but this method is clean for when we want to have different entities.
				// likely we don't have different entities, so maybe just use i for indexation.
				const Entity& e = m_Entities[i];

				const PathComponent& path = m_Paths[e];
				const PositionComponent& pos = m_Positions[e];

				//float distFromTarget = Utility::MathUtility::Distance(pos.x, pos.y, path.x[path.currentIndex], path.y[path.currentIndex]);
				float distFromArrival = Utility::MathUtility::Distance(pos.x, pos.y, path.x[path.numPoints-1], path.y[path.numPoints - 1]);
				float arrivalMultiplier = 1.0f;

				Point currentPosition(pos.x, pos.y);
				Vec2 currentVelocity(m_Velocities[e].dx, m_Velocities[e].dy);
				Point attractionPoint;

				// arrival force
				if (distFromArrival < arrivalRadius)
				{
					arrivalMultiplier = distFromArrival / arrivalRadius;
					attractionPoint.x = path.x[path.numPoints - 1];
					attractionPoint.y = path.y[path.numPoints - 1];
				}
				else
				{
					Point futurePosition = currentPosition + currentVelocity * attractionLookAheadMultiplier;
					float closestPoint = Utility::MAX_FLOAT;
					// calculate attraction point
					std::vector<Point> points;
					for (int j = 0; j < path.numPoints - 1; j++)
					{
						Point p = Utility::MathUtility::GetClosestPointOnSegment(futurePosition, Segment(path.x[j], path.y[j], path.x[j + 1], path.y[j + 1]));
						float dist = Utility::MathUtility::Distance(futurePosition, p);

						if (dist < closestPoint)
						{
							attractionPoint = p;
							closestPoint = dist;
						}
					}
				}


				Vec2 desiredVelocity = (attractionPoint - currentPosition);
				desiredVelocity.Normalize();
				desiredVelocity = desiredVelocity * speed;

				Vec2 steering = (desiredVelocity - currentVelocity) * pathFollowSteeringWeight;

				// TODO:
				// 1. obstacle avoidance
				const ClearanceComponent& clearance = m_Clearances[e];
				ApplyBoundaryForce(steering, pos, clearance);

			
				// 2. dynamic obstacle avoidance


				float steeringForce = Utility::MathUtility::Length(steering);
				float steeringMultiplier = std::min(steeringForce, speed);
				steering.Normalize();
				steering = steering * steeringMultiplier;
				steering = steering * massRecip;


				Vec2 finalVelocity = currentVelocity + steering;
				float strength = Utility::MathUtility::Length(finalVelocity);
				float multiplier = std::min(strength, speed);
				finalVelocity.Normalize();
				finalVelocity = finalVelocity * multiplier * arrivalMultiplier;

				m_Velocities[e].dx = finalVelocity.x;
				m_Velocities[e].dy = finalVelocity.y;
			}
		}

		void Simulator::ApplyPathFollowForce(Vec2& steering)
		{

		}

		// TODO:
		// > proper parameter values
		// > calculate weight such that agent can never cross boundary
		void Simulator::ApplyBoundaryForce(Vec2& steering, const PositionComponent& pos, const ClearanceComponent& clearance)
		{
			// query current position in ECM (returns ECM cell)
			ECMCell* cell = m_Ecm->GetECMCell(pos.x, pos.y);

			if (cell == nullptr) return;

			// from ECM cell, get the obstacle segment
			const Segment& obstacle = cell->boundary;
			
			// calculate the distance from the position to the segment. If below threshold, return;
			float preferredSafeDistance = 100.0f; // TODO: make variable
			Point closestPoint = Utility::MathUtility::GetClosestPointOnSegment(Point(pos.x, pos.y), obstacle);
			float distance = Utility::MathUtility::Distance(Point(pos.x, pos.y), closestPoint);
			if ((distance - clearance.clearance) > preferredSafeDistance) return;
			
			// otherwise calculate the force direction (normal of segment)
			Vec2 oNormal = Point(pos.x, pos.y) - closestPoint;
			oNormal.Normalize();

			// calculate the weight of the force (scalar value, see paper Indicative Routes for Path Planning and Crowd Simulation
			// apply force to steering
			const float repulsiveSteepness = 0.5f;
			float weight = (preferredSafeDistance + clearance.clearance - distance) / std::pow((distance - clearance.clearance), repulsiveSteepness);

			//float weight = (distance / distThreshold);

			steering = steering + oNormal * weight;
		}

	}

}