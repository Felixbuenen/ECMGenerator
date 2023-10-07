#include "Simulator.h"

#include "ECM.h"
#include "UtilityFunctions.h"
#include "ECMPathPlanner.h"
#include "Environment.h"

namespace ECM {

	namespace Simulation {

		void Simulator::InitAgents(int count, float clearance)
		{
			m_NDefaultEntities = count;

			// initialize data
			m_DefaultEntities = new Entity[count];
			m_Positions = new PositionComponent[count];
			m_Goals = new PositionComponent[count];
			m_Velocities = new VelocityComponent[count];
			m_Clearances = new ClearanceComponent[count];
			m_Paths = new PathComponent[count];

			for (int i = 0; i < count; i++)
			{
				m_Clearances[i].clearance = clearance;
				m_Velocities[i].dx = 0.0f;
				m_Velocities[i].dy = 0.0f;
			}

			printf("SIMULATOR: Data for %d agents was created.\n", count);
		}

		void Simulator::Initialize()
		{
			// set default indices
			for (int i = 0; i < m_NDefaultEntities; i++)
			{
				m_DefaultEntities[i] = i;
			}
			
			// calculate paths using positions and goals data
			for (int i = 0; i < m_NDefaultEntities; i++)
			{
				const Entity& e = m_DefaultEntities[i];

				const PositionComponent& start = m_Positions[e];
				const PositionComponent& goal = m_Goals[e];
				const ClearanceComponent& cl = m_Clearances[e];

				// TODo: switching to different types is terribly inefficient. Either use existing structs or find a way to efficiently move/cast data.
				const float preferredAddClearance = 10.0f;
				PathPlanning::Corridor dummy;
				std::vector<Segment> portal;
				PathPlanning::Path path;
				m_Planner->GetPath(*m_Environment, Point(start.x, start.y), Point(goal.x, goal.y), cl.clearance, preferredAddClearance, dummy, portal, path);

				PathComponent& pComponent = m_Paths[e];
				pComponent.x = new float[(int)path.size()];
				pComponent.y = new float[(int)path.size()];
				pComponent.numPoints = path.size();
				pComponent.currentIndex = 0;

				for (int j = 0; j < path.size(); j++)
				{
					pComponent.x[j] = path[j].x;
					pComponent.y[j] = path[j].y;
				}

				Vec2 initVelocity = (path[1] - path[0]);
				initVelocity.Normalize();

				m_Velocities[e].dx = initVelocity.x * 30;
				m_Velocities[e].dy = initVelocity.y * 30;
			}
		}

		void Simulator::Update(float dt)
		{
			UpdatePathFollowForce(dt);
			UpdatePositionSystem(dt);
			//UpdateVelocitySystem(dt);
		}

		void Simulator::ClearSimulator()
		{
			// delete data
			for (int i = 0; i < m_NDefaultEntities; i++)
			{
				delete[] m_Paths[i].x;
				delete[] m_Paths[i].y;
			}

			delete[] m_DefaultEntities;
			delete[] m_Positions;
			delete[] m_Velocities;
			delete[] m_Paths;

			printf("SIMULATOR: Data was destroyed.\n");
		}

		void Simulator::UpdatePositionSystem(float dt)
		{
			for (int i = 0; i < m_NDefaultEntities; i++)
			{
				// note that currently this is redundant, but this method is clean for when we want to have different entities.
				// likely we don't have different entities, so maybe just use i for indexation.
				const Entity& e = m_DefaultEntities[i]; 

				const VelocityComponent& vel = m_Velocities[e];
				PositionComponent& pos = m_Positions[e];


				pos.x += vel.dx * dt;
				pos.y += vel.dy * dt;
			}
		}

		void Simulator::UpdatePathFollowForce(float dt)
		{
			// TODO:
			// Kijk of een chasing rabbit algoritme beter is. dwz, ga naar een punt iets verder op het pad ipv het volgende target punt. Je ziet namelijk dat als er een
			//  afwijking ontstaat, de agent heel traag bijwerkt. 
			// Volgens mij is deze methode ook meer gangbaar.
			// Zie voorbeeld van coding train.

			// Let's start simple:
			// 1. calculate future position (velocity * lookAhead)
			// 2. iterate through all path segments to find the segment where the future position lies closest
			// 3. project the future position on this line segment. this is the attraction point.

			const float reachedRadius = 2.5f;
			const float arrivalRadius = 50.0f;
			const float mass = 50.0f;
			const float massRecip = 1.0f / mass;
			const float speed = 60.0f;
			const float attractionLookAheadMultiplier = 0.5f;

			for (int i = 0; i < m_NDefaultEntities; i++)
			{
				// note that currently this is redundant, but this method is clean for when we want to have different entities.
				// likely we don't have different entities, so maybe just use i for indexation.
				const Entity& e = m_DefaultEntities[i];

				PathComponent& path = m_Paths[e];
				const PositionComponent& pos = m_Positions[e];

				//float distFromTarget = Utility::MathUtility::Distance(pos.x, pos.y, path.x[path.currentIndex], path.y[path.currentIndex]);
				float distFromArrival = Utility::MathUtility::Distance(pos.x, pos.y, path.x[path.numPoints-1], path.y[path.numPoints - 1]);
				float arrivalMultiplier = 1.0f;

				// arrival force
				if (distFromArrival < arrivalRadius)
				{
					arrivalMultiplier = distFromArrival / arrivalRadius;
				}

				Vec2 currentVelocity(m_Velocities[e].dx, m_Velocities[e].dy);

				Point currentPosition(pos.x, pos.y);
				 
				Point futurePosition = currentPosition + currentVelocity * attractionLookAheadMultiplier;
				float closestPoint = Utility::MAX_FLOAT;
				Point attractionPoint;
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

				Vec2 desiredVelocity = (attractionPoint - currentPosition);
				desiredVelocity.Normalize();
				desiredVelocity = desiredVelocity * speed;

				Vec2 steering = desiredVelocity - currentVelocity;
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

		void Simulator::UpdateVelocitySystem(float dt)
		{
			// for this we need:
			// > position (to check if we reached target).
			// > path target
			// > velocity (only if we need to update the velocity)
		}

	}

}