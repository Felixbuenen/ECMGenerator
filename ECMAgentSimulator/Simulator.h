#pragma once

#include <iostream>
#include <stack>
#include <map>

#include "Area.h"

namespace ECM {

	class ECM;
	class Environment;

	struct Vec2;
	struct Point;

	namespace PathPlanning {
		class ECMPathPlanner;
	}


	namespace Simulation {

		typedef int Entity;
		class KDTree;
		class ORCA;
		class IRMPathFollower;

		struct PositionComponent
		{
			float x;
			float y;
		};

		struct VelocityComponent
		{
			float dx;
			float dy;
		};

		struct ClearanceComponent
		{
			float clearance;
		};

		struct SpeedComponent
		{
			float speed;
		};

		struct PathComponent
		{
			int currentIndex;
			int numPoints;
			float* x;
			float* y;
		};

		class Simulator
		{
		public:

			Simulator(ECM* ecm, PathPlanning::ECMPathPlanner* planner, Environment* environment, int maxAgents, float simStepTime)
				: m_Ecm(ecm), m_Planner(planner), m_Environment(environment), m_MaxNumEntities(maxAgents), m_SimStepTime(simStepTime)
			{
				for (int i = maxAgents - 1; i >= 0; i--)
				{
					m_freeEntitySpaces.push(i);
				}
			}
			~Simulator()
			{
				ClearSimulator();
			}

			int SpawnAgent(const Point& start, const Point& goal, float clearance, float preferredSpeed);
			void DestroyAgent(int idx);

			void Initialize();
			void Update(float dt);
			void Reset();

			inline void AddPosition(Entity entity, float x, float y) {
				m_Positions[entity].x = x;
				m_Positions[entity].y = y;
			}

			int AddSpawnArea(const Point& position, const Vec2& halfSize, const SpawnConfiguration& config, int ID = -1); // TODO: expand with agent profile
			int AddGoalArea(const Point& position, const Vec2& halfSize, int ID = -1);
			int AddObstacleArea(const Point& position, const Vec2& halfSize, bool updateECM = false);
			void RemoveArea(Simulation::SimAreaType areaType, int ID);
			void ConnectSpawnGoalAreas(int spawnID, int goalID, float spawnRate = 0.0f);
			void DeconnectSpawnGoalAreas(int spawnID, int goalID);
			
			std::map<int, SpawnArea>& GetSpawnAreas() { return m_SpawnAreas; }
			std::map<int, GoalArea>& GetGoalAreas() { return m_GoalAreas; }
			std::vector<ObstacleArea>& GetObstacleAreas() { return m_ObstacleAreas; }

			SpawnArea* GetSpawnArea(int ID);
			GoalArea* GetGoalArea(int ID);

			void FindNNearestNeighbors(const Entity& agent, int n, std::vector<Entity>& outNeighbors, int& outNNeighbors); // acceleration struct (KD-tree)
			void FindNNearestNeighborsDeprecated(const Entity& agent, int n, std::vector<Entity>& outNeighbors, int& outNNeighbors); // brute-force
			void FindNearestObstacles(const Entity& agent, float rangeSquared, std::vector<const ObstacleVertex*>& outObstacles) const;
			bool ValidSpawnLocation(const Point& location, float clearance) const;

			void UpdatePath(const Entity& e, const Point& location, const Point& goal);

			// GETTERS
			inline int GetNumAgents() const { return m_NumEntities; }
			inline int GetLastIndex() const { return m_LastEntityIdx; }
			inline PositionComponent* GetPositionData() const { return m_Positions; }
			inline VelocityComponent* GetVelocityData() const { return m_Velocities; }
			inline VelocityComponent* GetPreferredVelocityData() const { return m_PreferredVelocities; }
			inline PathComponent* GetPathData() const { return m_Paths; }
			inline ClearanceComponent* GetClearanceData() const { return m_Clearances; }
			inline PositionComponent* GetAttractionPointData() const { return m_AttractionPoints; }
			inline PathPlanning::ECMPathPlanner* GetECMPathPlanner() { return m_Planner; }
			inline bool* GetActiveFlags() const { return m_ActiveAgents; }
			inline KDTree* GetKDTree() const { return m_KDTree; }
			inline Environment* GetEnvironment() const { return m_Environment; }
			inline float GetSimulationStepTime() const { return m_SimStepTime; }
			
			std::vector<int> GetConnectedAreas(int sourceID, SimAreaType type);

			// DEBUG
			int NN_TO_DRAW = 0;
			std::vector<int> NEAREST_NEIGHBORS;
			// DEBUG

		private:
			void ClearSimulator();

			// the max agent index represents the active agent with the highest ID.
			// keeping track of this allows the simulator to only look at agents up until
			// this ID in the agent pool.
			void UpdateMaxAgentIndex();
			void UpdateSpawnAreas();

			// SYSTEMS
			void UpdateAttractionPointSystem();
			void UpdatePositionSystem();
			void UpdateForceSystem();
			void UpdateVelocitySystem();

			void ApplySteeringForce();
			void ApplyObstacleAvoidanceForce();

		private:
			ECM* m_Ecm;
			PathPlanning::ECMPathPlanner* m_Planner;
			Environment* m_Environment;
			KDTree* m_KDTree;
			ORCA* m_ORCA;
			IRMPathFollower* m_PathFollower;

			// TODO: should probably be in a separate class
			std::vector<std::tuple<float, int>> m_NNDistances;

			int m_MaxNumEntities;
			int m_NumEntities;
			Entity* m_Entities;
			std::stack<int> m_freeEntitySpaces;
			bool* m_ActiveAgents;
			int m_LastEntityIdx;

			float m_SimStepTime;

			std::map<int, SpawnArea> m_SpawnAreas;
			std::map<int, GoalArea> m_GoalAreas;
			std::vector<ObstacleArea> m_ObstacleAreas;
			int m_NextSpawnID = 0;
			int m_NextGoalID = 0;

			// COMPONENTS
			PositionComponent* m_Positions;
			PositionComponent* m_AttractionPoints;
			VelocityComponent* m_PreferredVelocities;
			VelocityComponent* m_Forces;
			VelocityComponent* m_Velocities;
			ClearanceComponent* m_Clearances;
			SpeedComponent* m_PreferredSpeed;
			PathComponent* m_Paths;
		};

	}
}

