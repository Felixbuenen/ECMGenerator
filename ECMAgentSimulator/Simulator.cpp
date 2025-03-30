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
			m_PreferredSpeed = new SpeedComponent[m_MaxNumEntities];
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
			delete[] m_PreferredSpeed;
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

		void Simulator::UpdatePath(const Entity& e, const Point& location, const Point& goal)
		{
			// remove any existing path data
			if (m_Paths[e].numPoints > 0)
			{
				delete[] m_Paths[e].x;
				delete[] m_Paths[e].y;
				m_Paths[e].currentIndex = 0;
				m_Paths[e].numPoints = 0;
			}

			const float preferredAddClearance = 0.0f; // DEBUG
			PathPlanning::Corridor dummy;
			std::vector<Segment> portal;
			PathPlanning::Path path;
			m_Planner->FindPath(*m_Environment, location, goal, m_Clearances[e].clearance, preferredAddClearance, dummy, portal, path);
			
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
			m_PreferredSpeed[idx].speed = preferredSpeed;

			m_ActiveAgents[idx] = true;

			// plan path
			UpdatePath(idx, start, goal);
	
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

		void Simulator::FindNearestObstacles(const Entity& agent, float rangeSquared, std::vector<const ObstacleVertex*>& outObstacles) const
		{
			// TODO: use KD tree
			// for now do a brute force method
			std::vector<std::tuple<float, int>> distances;
			Point agentPos(m_Positions[agent].x, m_Positions[agent].y);

			const std::vector<Obstacle>& obstacles = GetEnvironment()->GetObstacles();

			for (const Obstacle& o : obstacles)
			{
				const auto& obstVerts = o.verts;
				for (int i = 0; i < obstVerts.size(); i++)
				{
					const ObstacleVertex* obst = obstVerts[i];
					const ObstacleVertex* obstNext = obst->nextObstacle
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

		// TODO: we either use dt or m_SimStepUpdate, not both... 
		void Simulator::Update(float dt)
		{
			UpdateMaxAgentIndex();
			UpdateSpawnAreas();
			UpdateForceSystem();
			UpdateVelocitySystem();
			UpdatePositionSystem();
		}

		void Simulator::Reset()
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				DestroyAgent(m_Entities[i]);
			}
		}


		int Simulator::AddSpawnArea(const Point& position, const Vec2& halfSize, const SpawnConfiguration& config)
		{
			SpawnArea sa;
			sa.HalfWidth = halfSize.x;
			sa.HalfHeight = halfSize.y;
			sa.ID = m_SpawnAreas.size();
			sa.Position = position;
			//sa.spawnRate = config.spawnRate;
			sa.spawnConfiguration.clearanceMin = config.clearanceMin;
			sa.spawnConfiguration.clearanceMax = config.clearanceMax;
			sa.spawnConfiguration.preferredSpeedMin = config.preferredSpeedMin;
			sa.spawnConfiguration.preferredSpeedMax = config.preferredSpeedMax;

			m_SpawnAreas.push_back(sa);

			return sa.ID;
		}

		int Simulator::AddGoalArea(const Point& position, const Vec2& halfSize)
		{
			GoalArea ga;
			ga.ID = m_GoalAreas.size();
			ga.Position = position;
			ga.HalfHeight = halfSize.y;
			ga.HalfWidth = halfSize.x;

			m_GoalAreas.push_back(ga);

			return ga.ID;
		}

		int Simulator::AddObstacleArea(const Point& position, const Vec2& halfSize, bool updateECM)
		{
			// TODO:
			// ONDERSTAANDE IS EEN AANNAME, MEER TESTEN IS NODIG!
			// Nu is zeer waarschijnlijk het probleem dat we obstacles met floating point precisie aan Boost geven.
			// Echter Boost werkt (naar wat ik weet) met fixed point coordinates. Oftewel integers.
			// Als je floating point precisie geeft, komen er onbetrouwbare resultaten uit boost.
			// Dit genereert incosistente voronoi diagrams, en dat lijdt tot gaten in de 
			// Ik moet dus het volgende doen:
			// > definieer hoeveel precisie je wilt. Centimeters? Hoe sla je deze coordinaten op? uints? dan kun je heel veel
			//   coordinaten nog opslaan.
			// > deze uints geven we aan boost. Hier moeten we ook een vertaalslag maken naar world coordinates. bijv, als we 
			//   een obstacle object toevoegen, weten we de screen coordinates (in floats). Deze moet vertaald worden naar
			//   world coordinates (ook in floats). Deze moet ook vertaald worden zodat boost het kan lezen (naar ints).

			ObstacleArea oa;
			oa.ID = m_ObstacleAreas.size() == 0 ? 0 : m_ObstacleAreas[m_ObstacleAreas.size() - 1].ID + 1;
			oa.Position = position;
			oa.HalfHeight = halfSize.y;
			oa.HalfWidth = halfSize.x;

			oa.obstacleVerts.push_back(Point(position.x + halfSize.x, position.y + halfSize.y));
			oa.obstacleVerts.push_back(Point(position.x - halfSize.x, position.y + halfSize.y));
			oa.obstacleVerts.push_back(Point(position.x - halfSize.x, position.y - halfSize.y));
			oa.obstacleVerts.push_back(Point(position.x + halfSize.x, position.y - halfSize.y));

			m_Environment->AddObstacle(oa.obstacleVerts, updateECM);

			return oa.ID;
		}

		void Simulator::RemoveArea(Simulation::SimAreaType areaType, int ID)
		{
			if (areaType == SimAreaType::SPAWN)
			{
				for (int i = 0; i < m_SpawnAreas.size(); i++)
				{
					if (m_SpawnAreas[i].ID == ID)
					{
						m_SpawnAreas.erase(m_SpawnAreas.begin() + i);
					}
				}
			}
			if (areaType == SimAreaType::GOAL)
			{
				for (int i = 0; i < m_GoalAreas.size(); i++)
				{
					if (m_GoalAreas[i].ID == ID)
					{
						m_GoalAreas.erase(m_GoalAreas.begin() + i);
					}
				}
			}
		}

		void Simulator::ConnectSpawnGoalAreas(int spawnID, int goalID, float spawnRate)
		{
			// check if goal area is not already connected
			for (int ga : m_SpawnAreas[spawnID].connectedGoalAreas)
			{
				if (ga == goalID) return;
			}

			SpawnArea& sa = m_SpawnAreas[spawnID];
			sa.connectedGoalAreas.push_back(goalID);
			sa.timeSinceLastSpawn.push_back(0.0f);
			sa.spawnRate.push_back(spawnRate);
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

		void Simulator::UpdateSpawnAreas()
		{
			for (SpawnArea& area : m_SpawnAreas)
			{
				for (int ga = 0; ga < area.connectedGoalAreas.size(); ga++)
				{
					area.timeSinceLastSpawn[ga] += m_SimStepTime;

					int agentsToSpawn = area.timeSinceLastSpawn[ga] * area.spawnRate[ga];
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
						Point goal = m_GoalAreas[area.connectedGoalAreas[ga]].GetRandomPositionInArea();

						SpawnAgent(start, goal, clearance, speed);
					}

					area.timeSinceLastSpawn[ga] -= (float)agentsToSpawn / area.spawnRate[ga];
				}
			}
		}

		void Simulator::UpdateAttractionPointSystem()
		{
			// TODO: make global
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
					Point attractionPoint;
					bool success = m_PathFollower->FindAttractionPoint(*m_Ecm, m_Ecm->GetECMGraph(), currentPosition, path, attractionPoint);

					if (success)
					{
						m_AttractionPoints[e].x = attractionPoint.x;
						m_AttractionPoints[e].y = attractionPoint.y;
					}

					// couldn't find an IRM attraction point, which means the agent was pushed away too far from its global path. We need to recalculate
					// the path.
					else
					{
						// temp
						std::cout << "Recalculate path..." << std::endl;
						Point goal(path.x[path.numPoints - 1], path.y[path.numPoints - 1]);
						UpdatePath(e, currentPosition, goal);
					}
				}
			}
		}

		void Simulator::UpdatePositionSystem()
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity& e = m_Entities[i];

				const VelocityComponent& vel = m_Velocities[e];
				PositionComponent& pos = m_Positions[e];

				pos.x += (vel.dx * m_SimStepTime);
				pos.y += (vel.dy * m_SimStepTime);
			}
		}

		// TODO: currently all forces are calculated for each agent.
		// It is likely more efficient to calculate each force invidividually in batches for all agents.
		// Now, the loop must maintain all the local variables which will clutter the cache.
		void Simulator::UpdateForceSystem()
		{
			UpdateAttractionPointSystem();
			ApplySteeringForce();
			ApplyObstacleAvoidanceForce();
		}

		void Simulator::UpdateVelocitySystem()
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
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity& e = m_Entities[i];
				Point currentPosition(m_Positions[e].x, m_Positions[e].y);
				Point attractionPoint(m_AttractionPoints[e].x, m_AttractionPoints[e].y);
				float speed = m_PreferredSpeed[e].speed;


				Vec2 desiredVelocity = (attractionPoint - currentPosition);
				desiredVelocity.Normalize();
				desiredVelocity = desiredVelocity * speed;
				
				m_PreferredVelocities[e].dx = desiredVelocity.x;
				m_PreferredVelocities[e].dy = desiredVelocity.y;
			}			
		}

		void Simulator::ApplyObstacleAvoidanceForce()
		{
			// todo: make global
			const int numRVONeighbors = 5;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity e = m_Entities[i];
				float speed = m_PreferredSpeed[i].speed;

				Vec2 outVel;
				m_ORCA->GetVelocity(this, e, m_SimStepTime, speed, numRVONeighbors, outVel);

				m_Forces[e].dx = (outVel.x - m_Velocities[e].dx);
				m_Forces[e].dy = (outVel.y - m_Velocities[e].dy);
			}
		}
	}

}