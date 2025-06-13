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
#include "Timer.h"
#include "../ECMApplication/ThreadPool.h"

#include <math.h>

namespace ECM {

	namespace Simulation {

		// TODO: dependency injection
		void Simulator::Initialize()
		{
			m_LastEntityIdx = -1;
			m_ActiveAgents = new bool[m_MaxNumEntities];
			m_CellVisit = new int[m_MaxNumEntities];

			// initialize data
			m_Entities = new Entity[m_MaxNumEntities];
			m_Positions = new PositionComponent[m_MaxNumEntities];

			m_AttractionPoints = new PositionComponent[m_MaxNumEntities];
			m_PrevVelocities = new VelocityComponent[m_MaxNumEntities];
			m_Velocities = new VelocityComponent[m_MaxNumEntities];
			m_PreferredVelocities = new VelocityComponent[m_MaxNumEntities];
			m_PreferredSpeed = new SpeedComponent[m_MaxNumEntities];
			m_Forces = new VelocityComponent[m_MaxNumEntities];
			m_Clearances = new ClearanceComponent[m_MaxNumEntities];
			m_Paths = new PathComponent[m_MaxNumEntities];

			for (int i = 0; i < m_MaxNumEntities; i++)
			{
				m_Entities[i] = i;
				m_CellVisit[i] = -1;

				// set sub-arrays to nullptrs as default
				// if a path component is not assigned, and we delete it on cleanup, 
				// we can't delete[] the path.X and path.Y data (as it is not assigned).
				// on cleanup, we explicitly check for nullptr's to prevent this.
				m_Paths[i].x = static_cast<float*>(nullptr);
				m_Paths[i].y = static_cast<float*>(nullptr);
				m_Paths[i].currentIndex = -1;
				m_Paths[i].numPoints = 0;
				m_PrevVelocities[i].dx = 0.0f;
				m_PrevVelocities[i].dy = 0.0f;
			}


			m_KDTree = new KDTree();

			// TODO: put in config file
			const int numOrcaNeighbors = 5;
			const int numThreads = 20; // todo: maak variable
			for (int i = 0; i < numThreads; i++)
			{
				m_ORCAPerThread.push_back(new ORCA(numOrcaNeighbors));
			}
			m_PathFollower = new IRMPathFollower();
			m_ThreadPool = new ThreadPool(numThreads);

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

			int numOrcaInstances = m_ORCAPerThread.size();
			for (int i = 0; i < numOrcaInstances; i++) delete m_ORCAPerThread[i];

			delete[] m_Entities;
			delete[] m_Positions;
			delete[] m_CellVisit;
			delete[] m_AttractionPoints;
			delete[] m_PrevVelocities;
			delete[] m_Velocities;
			delete[] m_PreferredVelocities;
			delete[] m_PreferredSpeed;
			delete[] m_Forces;
			delete[] m_Paths;
			delete[] m_ActiveAgents;
			delete[] m_Clearances;

			delete m_KDTree;
			delete m_PathFollower;
			delete m_ThreadPool;

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

		std::vector<int> Simulator::GetConnectedAreas(int sourceID, SimAreaType type)
		{
			if (type == SPAWN)
			{
				SpawnArea* sa = GetSpawnArea(sourceID);
				if (sa != nullptr)
				{
					return sa->connectedGoalAreas;
				}
				
				return std::vector<int>();
			}

			if (type == GOAL)
			{
				GoalArea* ga = GetGoalArea(sourceID);
				std::vector<int> result;

				if(ga != nullptr)
				{
					for (auto iter = m_SpawnAreas.begin(); iter != m_SpawnAreas.end(); iter++)
					{
						const std::vector<int>& goalIDs = iter->second.connectedGoalAreas;

						for (int j = 0; j < goalIDs.size(); j++)
						{
							if (m_GoalAreas[j].ID == sourceID)
							{
								result.push_back(iter->first);
								break;
							}
						}

					}
				}

				return result;
			}

			return std::vector<int>();
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

		// Find n nearest neighbors to agent 'agent'. Expects 
		void Simulator::FindNNearestNeighbors(const Entity& agent, int n, std::vector<Entity>& outNeighbors, int& outNNeighbors)
		{
			if (outNeighbors.size() != n)
			{
				std::cout << "ERROR: FindNNearestNeighbors() expects std::vector<Entity>& outNeighbors to be of size n." << std::endl;
				return;
			}

			m_KDTree->KNearestAgents(this, agent, n, outNeighbors, outNNeighbors);
		}

		void Simulator::FindNearestNeighborsInRange(const Entity& agent, float radius, int maxNumAgents, std::vector<Entity>& outNeighbors, int& outNNeighbors)
		{
			if (outNeighbors.size() != maxNumAgents)
			{
				std::cout << "ERROR: FindNNearestNeighbors() expects std::vector<Entity>& outNeighbors to be of size n." << std::endl;
				return;
			}

			m_KDTree->AgentsInRange(this, agent, radius, maxNumAgents, outNeighbors, outNNeighbors);
		}



		// Brute force method (N^2) of finding neighbors. Not used anymore.
		void Simulator::FindNNearestNeighborsDeprecated(const Entity& agent, int n, std::vector<Entity>& outNeighbors, int& outNNeighbors)
		{
			outNeighbors.clear();
			outNeighbors.resize(n);
			Point agentPos(m_Positions[agent].x, m_Positions[agent].y);
			
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;
			
				const Entity& other = m_Entities[i];
				if (other == agent) continue;
			
				float dist = Utility::MathUtility::SquareDistance(agentPos, Point(m_Positions[other].x, m_Positions[other].y));
				m_NNDistances[i] = std::make_tuple(dist, other);
			}
			
			std::sort(m_NNDistances.begin(), m_NNDistances.end(),
				[](const std::tuple<float, int>& a, const std::tuple<float, int>& b) {
					return std::get<0>(a) < std::get<0>(b);
				});
			
			outNNeighbors = m_NNDistances.size() < n ? m_NNDistances.size() : n;
			
			for (int i = 0; i < outNNeighbors; ++i) {
				outNeighbors[i] = std::get<1>(m_NNDistances[i]);
			}
		}

		void Simulator::FindNearestObstacles(const Entity& agent, float rangeSquared, std::vector<const ObstacleVertex*>& outObstacles) const
		{
			// TODO: use KD tree. 
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
					float sqDist = (signedLeftDist * signedLeftDist) / Utility::MathUtility::SquareDistance(obst->p, obstNext->p);

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
			volatile MultipassTimer updateTimer("Update()");

			UpdateMaxAgentIndex();
			UpdateSpawnAreas();
			UpdateECMCellVisits();

			m_KDTree->Construct(this);

			// TEMP
			//for (int i = 0; i <= m_LastEntityIdx; i++)
			//{
			//	if (!m_ActiveAgents[i]) continue;
			//
			//	const Entity& e = m_Entities[i];
			//	m_PrevVelocities[e] = m_Velocities[e];
			//}
			// TEMP

			// agent forces
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


		int Simulator::AddSpawnArea(const Point& position, const Vec2& halfSize, const SpawnConfiguration& config, int ID)
		{
			SpawnArea sa;
			sa.HalfWidth = halfSize.x;
			sa.HalfHeight = halfSize.y;
			if (ID == -1)
			{
				sa.ID = m_NextSpawnID;
				m_NextSpawnID++;
			}
			else
			{
				sa.ID = ID;
			}
			sa.Position = position;
			//sa.spawnRate = config.spawnRate;
			sa.spawnConfiguration.clearanceMin = config.clearanceMin;
			sa.spawnConfiguration.clearanceMax = config.clearanceMax;
			sa.spawnConfiguration.preferredSpeedMin = config.preferredSpeedMin;
			sa.spawnConfiguration.preferredSpeedMax = config.preferredSpeedMax;

			m_SpawnAreas.emplace(sa.ID, sa);

			return sa.ID;
		}

		int Simulator::AddGoalArea(const Point& position, const Vec2& halfSize, int ID)
		{
			GoalArea ga;
			if(ID == -1)
			{
				ga.ID = m_NextGoalID;
				m_NextGoalID++;
			}
			else
			{
				ga.ID = ID;
			}
			ga.Position = position;
			ga.HalfHeight = halfSize.y;
			ga.HalfWidth = halfSize.x;

			m_GoalAreas.emplace(ga.ID, ga);

			return ga.ID;
		}

		int Simulator::AddObstacleArea(const Point& position, const Vec2& halfSize, bool updateECM)
		{
			// TODO:
			// ONDERSTAANDE IS EEN AANNAME, MEER TESTEN IS NODIG!
			// Nu is zeer waarschijnlijk het probleem dat we obstacles met floating point precisie aan Boost geven.
			// Echter Boost werkt met fixed point coordinates. Oftewel integers.
			// Als je floating point precisie geeft, komen er onbetrouwbare resultaten uit boost.
			// Dit genereert incosistente voronoi diagrams, en dat lijdt tot gaten in de 
			// TODO:
			// > definieer hoeveel precisie (bijv. centimeters?). Hoe sla je deze coordinaten op? uints? dan kun je heel veel
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
				m_SpawnAreas.erase(ID);
			}
			if (areaType == SimAreaType::GOAL)
			{
				m_GoalAreas.erase(ID);
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

		void Simulator::DeconnectSpawnGoalAreas(int spawnID, int goalID)
		{
			if (spawnID < 0 || spawnID >= m_SpawnAreas.size()) return;

			int indexToRemove = -1;
			for (int i = 0; i < m_SpawnAreas[spawnID].connectedGoalAreas.size(); i++)
			{
				if (m_SpawnAreas[spawnID].connectedGoalAreas[i] == goalID)
				{
					indexToRemove = i;
					break;
				}
			}

			if (indexToRemove == -1) return;

			m_SpawnAreas[spawnID].connectedGoalAreas.erase(m_SpawnAreas[spawnID].connectedGoalAreas.begin() + indexToRemove);
		}

		SpawnArea* Simulator::GetSpawnArea(int ID)
		{
			auto iter = m_SpawnAreas.find(ID);
			if (iter != m_SpawnAreas.end())
			{
				return &(iter->second);
			}

			return nullptr;
		}

		GoalArea* Simulator::GetGoalArea(int ID)
		{
			auto iter = m_GoalAreas.find(ID);
			if (iter != m_GoalAreas.end())
			{
				return &(iter->second);
			}

			return nullptr;
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
			for (auto iter = m_SpawnAreas.begin(); iter != m_SpawnAreas.end(); iter++)
			{
				SpawnArea& area = iter->second;
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

		void Simulator::UpdateECMCellVisits()
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const ECMCell* cell = m_Ecm->FindECMCell(m_Positions[i].x, m_Positions[i].y, m_CellVisit[i]);
				if (cell)
				{
					m_CellVisit[i] = cell->idx;
				}
			}
		}


		void Simulator::UpdateAttractionPointSystem()
		{
			// TODO: make global
			const float deleteDistanceSq = 4.0f;
			const float arrivalRadiusSq = 400.0f;

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
					bool success = m_PathFollower->FindAttractionPoint(*m_Ecm, m_Ecm->GetECMGraph(), currentPosition, path, m_CellVisit[e], attractionPoint);

					if (success)
					{
						m_AttractionPoints[e].x = attractionPoint.x;
						m_AttractionPoints[e].y = attractionPoint.y;
					}

					// couldn't find an IRM attraction point, which means the agent was pushed away too far from its global path. We need to recalculate
					// the path.
					else
					{
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

		void Simulator::UpdateForceSystem()
		{
			UpdateAttractionPointSystem();
			ApplySteeringForce();
			ApplyObstacleAvoidanceForce();
		}

		void Simulator::UpdateVelocitySystem()
		{
			// TODO: make global
			constexpr float mass = 0.8f;
			constexpr float massRecip = 1.0f / mass;
			const float massRecipDT = massRecip * m_SimStepTime;

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				Entity e = m_Entities[i];

				// a = F / m
				//m_Velocities[e].dx += m_Forces[e].dx * massRecipDT;
				//m_Velocities[e].dy += m_Forces[e].dy * massRecipDT;
			}
		}


		void Simulator::ApplySteeringForce()
		{
			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;

				const Entity& e = m_Entities[i];

				Vec2 desiredVelocity(m_AttractionPoints[e].x - m_Positions[e].x, m_AttractionPoints[e].y - m_Positions[e].y);
				desiredVelocity.Normalize();
				desiredVelocity = desiredVelocity * m_PreferredSpeed[e].speed;
				
				m_PreferredVelocities[e].dx = desiredVelocity.x;
				m_PreferredVelocities[e].dy = desiredVelocity.y;
			}			
		}

		void Simulator::ApplyObstacleAvoidanceForce()
		{
			volatile MultipassTimer orcaTimer("ApplyObstacleAvoidanceForce()");

			m_NNDistances.clear();
			m_NNDistances.resize(GetNumAgents());

			const int agentCount = m_LastEntityIdx + 1;
			const int numThreads = 20; // should be made variable
			const int minAgentsPerBatch = 20;
			const int maxAgentsPerBatch = 200;

			const int agentsPerBatch = std::max(std::min(agentCount / numThreads, maxAgentsPerBatch), minAgentsPerBatch);
		
			// prevent division by zero
			//if (agentsPerBatch <= 0) return;

			const int numBatches = std::ceil((float)agentCount / (float)agentsPerBatch);

			std::atomic<int> remainingBatches = numBatches;
			std::mutex orcaMutex;
			std::condition_variable waitCv;

			int stop = 0;

			for (int b = 0; b < numBatches; b++)
			{
				int start = b * agentsPerBatch;
				int stop = std::min(start + agentsPerBatch - 1, m_LastEntityIdx);
			
				// add ORCA task to thread pool. This task calculates the ORCA velocity for a certain batch and then writes the result to the velocity buffer.
				m_ThreadPool->AddTask(
					[start, stop, &remainingBatches, &orcaMutex, &waitCv, this](int threadID) { for (int i = start; i <= stop; i++)
				{
					if (!m_ActiveAgents[i]) continue;
			
					const Entity e = m_Entities[i];
			
					Vec2 outVel;
					m_ORCAPerThread[threadID]->GetVelocity(this, e, m_SimStepTime, m_PreferredSpeed[i].speed, outVel);
					m_Velocities[e].dx = outVel.x;
					m_Velocities[e].dy = outVel.y;
				}
			
				// if all batches are done, notify condition var that tasks are finished
				if (--remainingBatches == 0) 
				{
					std::lock_guard<std::mutex> lg(orcaMutex);
					waitCv.notify_one();
				}
				});
			
			}
			
			// wait until all ORCA threads are finished
			{
				std::unique_lock<std::mutex> lk(orcaMutex);
				waitCv.wait(lk, [&remainingBatches]() { return remainingBatches == 0; });
			}

			// - single thread -
			//for (int i = 0; i <= m_LastEntityIdx; i++)
			//{
			//	if (!m_ActiveAgents[i]) continue;
			//
			//	const Entity e = m_Entities[i];
			//
			//	Vec2 outVel;
			//	m_ORCAPerThread[0]->GetVelocity(this, e, m_SimStepTime, m_PreferredSpeed[i].speed, outVel);
			//	m_Velocities[e].dx = outVel.x;
			//	m_Velocities[e].dy = outVel.y;
			//}

			for (int i = 0; i <= m_LastEntityIdx; i++)
			{
				if (!m_ActiveAgents[i]) continue;
			
				const Entity e = m_Entities[i];
			
				m_Forces[e].dx = (m_Velocities[e].dx - m_PrevVelocities[e].dx);
				m_Forces[e].dy = (m_Velocities[e].dy - m_PrevVelocities[e].dy);
			
				m_PrevVelocities[e].dx = m_Velocities[e].dx;
				m_PrevVelocities[e].dy = m_Velocities[e].dy;
			}

			if (MultipassTimer::PrintAverages(20))
			{
				std::cout << "(With " << GetNumAgents() << " agents in scene)" << std::endl;
				std::cout << "---------------------------------------------" << std::endl;
				std::cout << std::endl;
			}
		}
	}

}