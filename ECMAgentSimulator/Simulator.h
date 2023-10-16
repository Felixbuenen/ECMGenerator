#pragma once

#include <iostream>
#include <stack>

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
			Simulator(std::shared_ptr<ECM> ecm, PathPlanning::ECMPathPlanner* planner, Environment* environment, int maxAgents) 
				: m_Ecm(ecm), m_Planner(planner), m_Environment(environment), m_MaxNumEntities(maxAgents)
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

			// ADD COMPONENT DATA
			inline void AddPosition(Entity entity, float x, float y) { 
				m_Positions[entity].x = x;
				m_Positions[entity].y = y;
			}
			//inline void AddGoal(Entity entity, float x, float y)
			//{ 
			//	m_Goals[entity].x = x;
			//	m_Goals[entity].y = y;
			//}

			inline int GetNumAgents() const { return m_NumEntities; }
			inline int GetLastIndex() const { return m_LastEntityIdx; }
			inline PositionComponent* GetPositionData() const { return m_Positions; }
			inline VelocityComponent* GetVelocityData() const { return m_Velocities; }
			inline PathComponent* GetPathData() const { return m_Paths; }
			inline ClearanceComponent* GetClearanceData() const { return m_Clearances; }
			inline bool* GetActiveFlags() const { return m_ActiveAgents; }

		private:
			void ClearSimulator();

			// the max agent index represents the active agent with the highest ID.
			// keeping track of this allows the simulator to only look at agents up until
			// this ID in the agent pool.
			void UpdateMaxAgentIndex();

			// SYSTEMS
			void UpdatePositionSystem(float dt);
			void UpdateVelocitySystem(float dt);

			void ApplyPathFollowForce(Vec2& steering);
			void ApplyBoundaryForce(Vec2& steering, const PositionComponent& pos, const ClearanceComponent& clearance);
			

		private:
			std::shared_ptr<ECM> m_Ecm;
			PathPlanning::ECMPathPlanner* m_Planner;
			Environment* m_Environment;

			int m_MaxNumEntities;
			int m_NumEntities;
			Entity* m_Entities;
			std::stack<int> m_freeEntitySpaces;
			bool* m_ActiveAgents;
			int m_LastEntityIdx;

			// COMPONENTS
			PositionComponent* m_Positions;
			//PositionComponent* m_Goals; // probably remove this, encapsulated in the path component
			VelocityComponent* m_Velocities;
			ClearanceComponent* m_Clearances;
			PathComponent* m_Paths;
		};

	}
}

