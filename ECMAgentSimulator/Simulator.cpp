#include "Simulator.h"

#include "ECM.h"
#include "UtilityFunctions.h"
#include "ECMPathPlanner.h"
#include "Environment.h"
#include "ECMDataTypes.h"
#include "Area.h"
#include "KDTree.h"
#include "ORCA.h"
#include "IRMPathFollower.h"

#include <math.h>

namespace ECM {

	namespace Simulation {

		// TODO: dependency injection
		void Simulator::Initialize()
		{
			m_LastEntityIdx = -1;
			m_ActiveAgents = new bool[m_MaxNumEntities];

			// initialize data
			m_Entities = new Entity[m_MaxNumEntities];
			m_Positions = new PositionComponent[m_MaxNumEntities];
			m_AttractionPoints = new PositionComponent[m_MaxNumEntities];
			//m_Goals = new PositionComponent[m_MaxNumEntities];
			m_Velocities = new VelocityComponent[m_MaxNumEntities];
			m_PreferredVelocities = new VelocityComponent[m_MaxNumEntities];
			m_Forces = new VelocityComponent[m_MaxNumEntities];
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
			m_ORCA = new ORCA();
			m_PathFollower = new IRMPathFollower();

			printf("SIMULATOR: Data for %d agents was created.\n", m_MaxNumEntities);

			m_CurrentStepDuration = m_SimStepTime;
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
			delete[] m_AttractionPoints;
			delete[] m_Velocities;
			delete[] m_PreferredVelocities;
			delete[] m_Forces;
			delete[] m_Paths;
			delete[] m_ActiveAgents;
			delete[] m_Clearances;
			//delete[] m_Goals;

			delete m_KDTree;
			delete m_ORCA;
			delete m_PathFollower;

			printf("SIMULATOR: Data was destroyed.\n");
		}

		int Simulator::SpawnAgent(const Point& start, const Point& goal, float clearance, float preferredSpeed)
		{
			if (m_freeEntitySpaces.empty()) 
				return -1;
			if (!ValidSpawnLocation(start, clearance)) 
				return -1;

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
			const float preferredAddClearance = 0.0f; // DEBUG
			PathPlanning::Corridor dummy;
			std::vector<Segment> portal;
			PathPlanning::Path path;
			m_Planner->FindPath(*m_Environment, start, goal, clearance, preferredAddClearance, dummy, portal, path);
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
			//Vec2 dir = Point(pComponent.x[1], pComponent.y[1]) - Point(pComponent.x[0], pComponent.y[0]);
			//dir.Normalize();
			
			m_PreferredVelocities[idx].dx = 0.0f;
			m_PreferredVelocities[idx].dy = 0.0f;
			m_Velocities[idx].dx = 0.0f;
			m_Velocities[idx].dy = 0.0f;
			m_Forces[idx].dx = 0.0f;
			m_Forces[idx].dy = 0.0f;

			return idx;
		}

		void Simulator::DestroyAgent(int idx)
		{
			m_NumEntities--;

			m_ActiveAgents[idx] = false;
			m_freeEntitySpaces.push(idx);
		}

		void Simulator::FindNNearestNeighbors(const Entity& agent, int n, std::vector<Entity>& outNeighbors) const
		{
			// TODO: use KD tree
			// for now do a brute force method

			std::vector<std::tuple<float, int>> distances;
			Point agentPos(m_Positions[agent].x, m_Positions[agent].y);

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity& other = m_Entities[i];
				if (other == agent) continue;

				distances.emplace_back(Utility::MathUtility::SquareDistance(agentPos, Point(m_Positions[other].x, m_Positions[other].y)), other);
			}

			std::sort(distances.begin(), distances.end(),
				[](const std::tuple<float, int>& a, const std::tuple<float, int>& b) {
					return std::get<0>(a) < std::get<0>(b);
				});

			for (int i = 0; i < n && i < distances.size(); ++i) {
				outNeighbors.push_back(std::get<1>(distances[i]));
			}
		}

		void Simulator::FindNearestObstacles(const Entity& agent, float rangeSquared, std::vector<const Obstacle*>& outObstacles) const
		{
			// TODO: use KD tree
			// for now do a brute force method

			std::vector<std::tuple<float, int>> distances;
			Point agentPos(m_Positions[agent].x, m_Positions[agent].y);

			const std::vector<Obstacle>& obstacles = GetEnvironment()->GetObstacles();

			for (int i = 0; i < obstacles.size(); i++)
			{
				const Obstacle* obst = &obstacles[i];
				const Obstacle* obstNext = obstacles[i].nextObstacle
					;
				float signedLeftDist = Utility::MathUtility::LineLeftDistance(obst->p, obstNext->p, agentPos);
				float sqDist = std::powf(signedLeftDist, 2.0f) / Utility::MathUtility::SquareDistance(obst->p, obstNext->p);

				if (sqDist < rangeSquared) {
					// if signedLeftDist < 0.0f, then the agent is right of the obstacle segment and therefor "in front of" the obstacle.
					// only test against these lines.
					if (signedLeftDist < 0.0f) {
						Point p = Utility::MathUtility::GetClosestPointOnSegment(agentPos, obst->p, obstNext->p);
						if (Utility::MathUtility::SquareDistance(p, agentPos) < rangeSquared)
						{
							outObstacles.push_back(obst);
						}
					}
				}
			}
		}


		bool Simulator::ValidSpawnLocation(const Point& location, float clearance) const
		{
			// TODO: use kd-tree

			float clearanceSquared = clearance * clearance;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				Vec2 diff(location.x - m_Positions[i].x, location.y - m_Positions[i].y);
				float diffLengthSqrd = diff.LengthSquared();
				if (diffLengthSqrd < clearanceSquared) return false;
			}

			return true;
		}

		void Simulator::Update(float dt)
		{
			dt *= m_SpeedScale;

			// convert to ms
			m_CurrentStepDuration += dt;

			UpdateMaxAgentIndex();
			UpdateSpawnAreas(dt);

			// we update the simulation every fixed time interval (commonly every 100ms)
			// we divide the simstep time by the speedscale in order to maintain the same simulation accuracy for different playback speeds
			//if (m_CurrentStepDuration >= (m_SimStepTime / m_SpeedScale))
			//{
			//	UpdateForceSystem(dt);
			//	m_CurrentStepDuration -= m_SimStepTime;
			//}

			UpdateForceSystem(dt);
			UpdateVelocitySystem(dt);
			UpdatePositionSystem(dt);
		}

		void Simulator::Reset()
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				DestroyAgent(m_Entities[i]);
			}
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
				const int maxSpawnAttempts = 10;
				for (int i = 0; i < agentsToSpawn; i++)
				{
					// TODO: implement random values
					float clearance = area.spawnConfiguration.clearanceMin;
					float speed = area.spawnConfiguration.preferredSpeedMin;

					bool foundValidLocation = false;
					Point start;

					for (int spawnAttempts = 0; spawnAttempts < maxSpawnAttempts; spawnAttempts++)
					{
						if (spawnAttempts > 0)
						{
							int test = 0;
						}
						start = area.GetRandomPositionInArea();
						foundValidLocation = ValidSpawnLocation(start, clearance);
						if (foundValidLocation) break;
					}

					if (!foundValidLocation)
					{
						std::cout << "Could not find a valid spawn position in the spawn area!" << std::endl;
						continue;
					}

					// TODO: implement weighted random goal area selection
					Point goal = m_GoalAreas[area.connectors[0].goalID].GetRandomPositionInArea();

					SpawnAgent(start, goal, clearance, speed);
				}

				area.timeSinceLastSpawn -= (float)agentsToSpawn / area.spawnRate;
			}
		}

		void Simulator::UpdateAttractionPointSystem()
		{
			const bool deleteAgent = true;
			const float deleteDistanceSq = 2.0f * 2.0f;
			const float arrivalRadiusSq = 20.0f * 20.0f;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;
				
				const Entity& e = m_Entities[i];
				
				PathComponent& path = m_Paths[e];
				const PositionComponent& pos = m_Positions[e];

				float distFromArrival = Utility::MathUtility::SquareDistance(pos.x, pos.y, path.x[path.numPoints - 1], path.y[path.numPoints - 1]);

				Point currentPosition(pos.x, pos.y);
				Vec2 currentVelocity(m_Velocities[e].dx, m_Velocities[e].dy);

				if (distFromArrival < arrivalRadiusSq)
				{
					m_AttractionPoints[e].x = path.x[path.numPoints - 1];
					m_AttractionPoints[e].y = path.y[path.numPoints - 1];

					if (distFromArrival < deleteDistanceSq) {
						DestroyAgent(e);
					}
				}
				else
				{
					Point attractionPoint = m_PathFollower->FindAttractionPoint(*m_Ecm, m_Ecm->GetECMGraph(), currentPosition, path);
					m_AttractionPoints[e].x = attractionPoint.x;
					m_AttractionPoints[e].y = attractionPoint.y;
				}
			}
		}

		void Simulator::UpdatePositionSystem(float dt)
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity& e = m_Entities[i];

				const VelocityComponent& vel = m_Velocities[e];
				PositionComponent& pos = m_Positions[e];

				// DEBUG
				//pos.x += vel.dx * dt;
				//pos.y += vel.dy * dt;
				pos.x += (vel.dx * m_SimStepTime);
				pos.y += (vel.dy * m_SimStepTime);
			}
		}

		// TODO: currently all forces are calculated for each agent.
		// It is likely more efficient to calculate each force invidividually in batches for all agents.
		// Now, the loop must maintain all the local variables which will clutter the cache.
		void Simulator::UpdateForceSystem(float dt)
		{
			UpdateAttractionPointSystem();
			ApplySteeringForce();
			ApplyObstacleAvoidanceForce(dt);
		}

		void Simulator::UpdateVelocitySystem(float dt)
		{
			// TODO: make global
			const float mass = 0.8f;
			const float massRecip = 1.0f / mass;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				Entity e = m_Entities[i];

				// a = F / m
				m_Velocities[e].dx += m_Forces[e].dx * massRecip * m_SimStepTime;
				m_Velocities[e].dy += m_Forces[e].dy * massRecip * m_SimStepTime;
			}
		}


		void Simulator::ApplySteeringForce()
		{
			// TODO: make global or agent specific..
			const float speed = 10.0f;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity& e = m_Entities[i];
				Point currentPosition(m_Positions[e].x, m_Positions[e].y);
				Point attractionPoint(m_AttractionPoints[e].x, m_AttractionPoints[e].y);

				Vec2 desiredVelocity = (attractionPoint - currentPosition);
				desiredVelocity.Normalize();
				desiredVelocity = desiredVelocity * speed;
				
				m_PreferredVelocities[e].dx = desiredVelocity.x;
				m_PreferredVelocities[e].dy = desiredVelocity.y;
			}			
		}

		void Simulator::ApplyObstacleAvoidanceForce(float dt)
		{
			// todo: make global
			const float speed = 10.0f;
			const int numRVONeighbors = 20;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity e = m_Entities[i];

				Vec2 outVel;
				m_ORCA->GetVelocity(this, e, m_SimStepTime, speed, numRVONeighbors, outVel);

				m_Forces[e].dx = (outVel.x - m_Velocities[e].dx);
				m_Forces[e].dy = (outVel.y - m_Velocities[e].dy);
			}
		}
	}

}