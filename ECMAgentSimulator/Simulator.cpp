#include "Simulator.h"

#include "ECM.h"
#include "UtilityFunctions.h"
#include "ECMPathPlanner.h"
#include "Environment.h"
#include "ECMDataTypes.h"
#include "Area.h"
#include "KDTree.h"

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
				m_Paths[i].x = static_cast<float*>(nullptr);
				m_Paths[i].y = static_cast<float*>(nullptr);
				m_Paths[i].currentIndex = -1;
				m_Paths[i].numPoints = 0;
			}

			m_KDTree = new KDTree();

			printf("SIMULATOR: Data for %d agents was created.\n", m_MaxNumEntities);


			// --------------- DEBUG -------------

			// TODO:
			// 1. make a simple ImGui menu with a spawn and goal area
			// 2. implement click-drag behavior
			// 3. start with assumption that there are as many spawn as goal areas. 
			// 4. next, implement connection logic

			//GoalArea ga;
			//ga.ID = 0;
			//ga.Position = Point(0.0f, 390.0f);
			//ga.HalfHeight = 70;
			//ga.HalfWidth = 320;

			AreaConnector con;
			con.goalID = 0;

			//SpawnArea sa;
			//sa.HalfWidth = 250;
			//sa.HalfHeight = 60;
			//sa.ID = 0;
			//sa.Position = Point(0.0f, -375.0f);
			//sa.spawnRate = 2000;
			//sa.spawnConfiguration.clearanceMin = 7.0f;
			//sa.spawnConfiguration.clearanceMax = 7.0f;
			//sa.spawnConfiguration.preferredSpeedMin = 5.0f;
			//sa.spawnConfiguration.preferredSpeedMax = 5.0f;
			//
			//sa.connectors.push_back(con);
			//
			//m_SpawnAreas.push_back(sa);
			//m_GoalAreas.push_back(ga);

			// --------------- DEBUG -------------
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

			delete m_KDTree;

			printf("SIMULATOR: Data was destroyed.\n");
		}

		int Simulator::SpawnAgent(const Point& start, const Point& goal, float clearance, float preferredSpeed)
		{
			if (m_freeEntitySpaces.empty()) return -1;

			m_NumEntities++;

			int idx = m_freeEntitySpaces.top();
			m_freeEntitySpaces.pop();
			m_LastEntityIdx = m_LastEntityIdx < idx ? idx : m_LastEntityIdx;

			// set parameters
			m_Positions[idx].x = start.x;
			m_Positions[idx].y = start.y;
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

			// calculate initial velocity (in the direction of the next path vertex
			Vec2 dir = Point(pComponent.x[1], pComponent.y[1]) - Point(pComponent.x[0], pComponent.y[0]);
			dir.Normalize();
			
			m_Velocities[idx].dx = dir.x;
			m_Velocities[idx].dy = dir.y;

			return idx;
		}

		void Simulator::DestroyAgent(int idx)
		{
			m_NumEntities--;

			m_ActiveAgents[idx] = false;
			m_freeEntitySpaces.push(idx);
		}

		void Simulator::Update(float dt)
		{
			dt *= m_SpeedScale;

			// convert to ms
			m_CurrentStepDuration += dt * 1000.0f;

			UpdateMaxAgentIndex();
			UpdateSpawnAreas(dt);

			// we update the simulation every fixed time interval (commonly every 100ms)
			// we divide the simstep time by the speedscale in order to maintain the same simulation accuracy for different playback speeds
			if (m_CurrentStepDuration >= (m_SimStepTime / m_SpeedScale))
			{
				UpdateVelocitySystem();
				m_CurrentStepDuration -= m_SimStepTime;
			}

			UpdatePositionSystem(dt);
		}

		void Simulator::AddSpawnArea(const Point& position, const Vec2& halfSize, const SpawnConfiguration& config)
		{
			SpawnArea sa;
			sa.HalfWidth = halfSize.x;
			sa.HalfHeight = halfSize.y;
			sa.ID = 1; // TODO: get ID through method
			sa.Position = position;
			sa.spawnRate = config.spawnRate;
			sa.spawnConfiguration.clearanceMin = config.clearanceMin;
			sa.spawnConfiguration.clearanceMax = config.clearanceMax;
			sa.spawnConfiguration.preferredSpeedMin = config.preferredSpeedMin;
			sa.spawnConfiguration.preferredSpeedMax = config.preferredSpeedMax;

			AreaConnector conn;
			conn.goalID = 0; // TODO: make dynamic. connection shouldn't be made here.

			sa.connectors.push_back(conn);
			m_SpawnAreas.push_back(sa);
		}

		void Simulator::AddGoalArea(const Point& position, const Vec2& halfSize)
		{
			GoalArea ga;
			ga.ID = 0;
			ga.Position = position;
			ga.HalfHeight = halfSize.y;
			ga.HalfWidth = halfSize.x;

			m_GoalAreas.push_back(ga);
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

		void Simulator::UpdateSpawnAreas(float dt)
		{
			for (SpawnArea& area : m_SpawnAreas)
			{
				area.timeSinceLastSpawn += dt;

				int agentsToSpawn = area.timeSinceLastSpawn * area.spawnRate;
				for (int i = 0; i < agentsToSpawn; i++)
				{
					Point start = area.GetRandomPositionInArea();
					
					// TODO: implement weighted random goal area selection
					Point goal = m_GoalAreas[area.connectors[0].goalID].GetRandomPositionInArea();

					// TODO: implement random values
					float clearance = area.spawnConfiguration.clearanceMin;
					float speed = area.spawnConfiguration.preferredSpeedMin;

					SpawnAgent(start, goal, clearance, speed);
				}

				area.timeSinceLastSpawn -= (float)agentsToSpawn / area.spawnRate;
			}
		}

		void Simulator::UpdatePositionSystem(float dt)
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				// note that currently this is redundant, but this method is clean for when we want to have different entities.
				// likely we don't have different entities, so maybe just use i for indexation.
				const Entity& e = m_Entities[i];

				const VelocityComponent& vel = m_Velocities[e];
				PositionComponent& pos = m_Positions[e];


				pos.x += vel.dx * dt;
				pos.y += vel.dy * dt;
			}
		}

		// TODO: currently all forces are calculated for each agent.
		// It is likely more efficient to calculate each force invidividually in batches for all agents.
		// Now, the loop must maintain all the local variables which will clutter the cache.
		void Simulator::UpdateVelocitySystem()
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

			const bool deleteAgent = true;
			const float deleteDistance = 2.0f;

			// force weights
			const float pathFollowSteeringWeight = 1.0f;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				// note that currently this is redundant, but this method is clean for when we want to have different entities.
				// likely we don't have different entities, so maybe just use i for indexation.
				const Entity& e = m_Entities[i];

				const PathComponent& path = m_Paths[e];
				const PositionComponent& pos = m_Positions[e];

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

					if (distFromArrival < deleteDistance) {
						DestroyAgent(e);
					}
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

				// TODO: batch processing
				// 1. obstacle avoidance
				const ClearanceComponent& clearance = m_Clearances[e];
				ApplyBoundaryForce(steering, pos, clearance);

			
				// 2. dynamic obstacle avoidance
				m_KDTree->Clear();
				m_KDTree->Construct(this);


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