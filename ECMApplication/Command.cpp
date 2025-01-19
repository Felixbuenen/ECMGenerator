#include "Command.h"
#include "Simulator.h"
#include "Application.h"

namespace ECM {

	namespace WindowApplication {

		void CMD_AddSimulationArea::Execute()
		{
			Simulation::Simulator* sim = m_Application->GetSimulator();

			if (m_AreaType == Simulation::SimAreaType::SPAWN)
			{
				Simulation::SpawnConfiguration defaultConfig;
				Point worldCoords = m_Application->GetECMRenderer()->ScreenToWorldCoordinates(m_ScreenPos.x, m_ScreenPos.y);
				m_ID = sim->AddSpawnArea(worldCoords, Vec2(50.0f, 50.0f), defaultConfig);
			}

			if (m_AreaType == Simulation::SimAreaType::GOAL)
			{
				Point worldCoords = m_Application->GetECMRenderer()->ScreenToWorldCoordinates(m_ScreenPos.x, m_ScreenPos.y);
				m_ID = sim->AddGoalArea(worldCoords, Vec2(50.0f, 50.0f));
			}
		}

		void CMD_AddSimulationArea::Undo()
		{
			Simulation::Simulator* sim = m_Application->GetSimulator();
			sim->RemoveArea(m_AreaType, m_ID);
		}
	}

}