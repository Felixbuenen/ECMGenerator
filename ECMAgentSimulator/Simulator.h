#pragma once

#include <iostream>

namespace ECM {

	class ECM;
	class Environment;

	struct Vec2;

	namespace PathPlanning {
		class ECMPathPlanner;
	}


	namespace Simulation {

		typedef int Entity;

		// 16 bytes
		struct PositionComponent
		{
			float x;
			float y;
		};

		// 16 bytes
		struct VelocityComponent
		{
			float dx;
			float dy;
		};

		struct ClearanceComponent
		{
			float clearance;
		};

		// 16 + 16*X bytes
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
			Simulator(std::shared_ptr<ECM> ecm, PathPlanning::ECMPathPlanner* planner, Environment* environment) 
				: m_Ecm(ecm), m_Planner(planner), m_Environment(environment)
			{
			}
			~Simulator()
			{
				ClearSimulator();
			}

			// initializes memory for the agents
			void InitAgents(int count, float clearance);
			
			void Initialize();
			void Update(float dt);

			// ADD COMPONENT DATA
			inline void AddPosition(Entity entity, float x, float y) { 
				m_Positions[entity].x = x;
				m_Positions[entity].y = y;
			}
			inline void AddGoal(Entity entity, float x, float y)
			{ 
				m_Goals[entity].x = x;
				m_Goals[entity].y = y;
			}

			inline int GetNumAgents() const { return m_NDefaultEntities; }
			inline PositionComponent* GetPositionData() const { return m_Positions; }
			inline VelocityComponent* GetVelocityData() const { return m_Velocities; }
			inline PathComponent* GetPathData() const { return m_Paths; }
			inline ClearanceComponent* GetClearanceData() const { return m_Clearances; }

		private:
			void ClearSimulator();

			// SYSTEMS
			void UpdatePositionSystem(float dt);
			void UpdateVelocitySystem(float dt);

			void ApplyPathFollowForce(Vec2& steering);
			void ApplyBoundaryForce(Vec2& steering, const PositionComponent& pos, const ClearanceComponent& clearance);
			

		private:
			std::shared_ptr<ECM> m_Ecm;
			PathPlanning::ECMPathPlanner* m_Planner;
			Environment* m_Environment;

			int m_NDefaultEntities;
			Entity* m_DefaultEntities; // contains all entities

			// COMPONENTS
			PositionComponent* m_Positions;
			PositionComponent* m_Goals;
			VelocityComponent* m_Velocities;
			ClearanceComponent* m_Clearances;
			PathComponent* m_Paths;
		};

	}
}

