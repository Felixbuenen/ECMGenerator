#pragma once

#include <iostream>
#include <stack>

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
		class RVO;

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

			// TODO: add simulation step time (commonly 100ms)
			Simulator(std::shared_ptr<ECM> ecm, PathPlanning::ECMPathPlanner* planner, Environment* environment, int maxAgents, float simStepTime) 
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

			void SetSpeed(float speed) { m_SpeedScale = speed; }

			void Initialize();
			void Update(float dt);

			inline void AddPosition(Entity entity, float x, float y) { 
				m_Positions[entity].x = x;
				m_Positions[entity].y = y;
			}

			void AddSpawnArea(const Point& position, const Vec2& halfSize, const SpawnConfiguration& config ); // TODO: expand with agent profile
			void AddGoalArea(const Point& position, const Vec2& halfSize);
			const std::vector<SpawnArea>& GetSpawnAreas() const { return m_SpawnAreas; }
			const std::vector<GoalArea>& GetGoalAreas() const { return m_GoalAreas; }

			void FindNNearestNeighbors(const Entity& agent, int n, std::vector<Entity>& outNeighbors) const;
			void FindNearestObstacles(const Entity& agent, float rangeSquared, std::vector<const Obstacle*>& outObstacles) const;
			bool ValidSpawnLocation(const Point& location, float clearance) const;

			// GETTERS
			inline int GetNumAgents() const { return m_NumEntities; }
			inline int GetLastIndex() const { return m_LastEntityIdx; }
			inline PositionComponent* GetPositionData() const { return m_Positions; }
			inline VelocityComponent* GetVelocityData() const { return m_Velocities; }
			inline VelocityComponent* GetPreferredVelocityData() const { return m_PreferredVelocities; }
			inline PathComponent* GetPathData() const { return m_Paths; }
			inline ClearanceComponent* GetClearanceData() const { return m_Clearances; }
			inline bool* GetActiveFlags() const { return m_ActiveAgents; }
			inline KDTree* GetKDTree() const { return m_KDTree; }
			inline Environment* GetEnvironment() const { return m_Environment; }

		private:
			void ClearSimulator();

			// the max agent index represents the active agent with the highest ID.
			// keeping track of this allows the simulator to only look at agents up until
			// this ID in the agent pool.
			void UpdateMaxAgentIndex();
			void UpdateSpawnAreas(float dt);

			// SYSTEMS
			void UpdateAttractionPointSystem();
			void UpdatePositionSystem(float dt);
			void UpdateVelocitySystem(float dt);

			void ApplySteeringForce();
			void ApplyObstacleAvoidanceForce(float dt);
			void ApplyBoundaryForce(Vec2& steering, const PositionComponent& pos, const ClearanceComponent& clearance);
			

		private:
			std::shared_ptr<ECM> m_Ecm;
			PathPlanning::ECMPathPlanner* m_Planner;
			Environment* m_Environment;
			KDTree* m_KDTree;
			RVO* m_RVO;

			int m_MaxNumEntities;
			int m_NumEntities;
			Entity* m_Entities;
			std::stack<int> m_freeEntitySpaces;
			bool* m_ActiveAgents;
			int m_LastEntityIdx;

			float m_SpeedScale = 1.0f;
			float m_SimStepTime;
			float m_CurrentStepDuration = 0.0f;

			std::vector<SpawnArea> m_SpawnAreas;
			std::vector<GoalArea> m_GoalAreas;

			// COMPONENTS
			PositionComponent* m_Positions;
			PositionComponent* m_AttractionPoints;
			VelocityComponent* m_PreferredVelocities;
			VelocityComponent* m_Velocities;
			ClearanceComponent* m_Clearances;
			PathComponent* m_Paths;
		};

	}
}

