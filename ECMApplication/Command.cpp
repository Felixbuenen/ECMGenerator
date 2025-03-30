#include "Command.h"
#include "Simulator.h"
#include "Application.h"
#include "EnvironmentEditor.h"

namespace ECM {

	namespace WindowApplication {

		void CMD_SelectArea::Execute()
		{
			m_EnvEditor->SelectArea(m_Area);
		}

		void CMD_SelectArea::Undo()
		{
			m_EnvEditor->SelectArea(m_PrevArea);
		}

		void CMD_SelectAreaConnection::Execute()
		{
			m_EnvEditor->SelectAreaConnection(m_AreaConnection);
		}

		void CMD_SelectAreaConnection::Undo()
		{
			m_EnvEditor->SelectAreaConnection(m_PrevAreaConnection);
		}

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

			if (m_AreaType == Simulation::SimAreaType::OBSTACLE)
			{
				Point worldCoords = m_Application->GetECMRenderer()->ScreenToWorldCoordinates(m_ScreenPos.x, m_ScreenPos.y);

				// TEMP FIX
				worldCoords.x = (int)worldCoords.x;
				worldCoords.y = (int)worldCoords.y;
				// TEMP FIX

				m_ID = sim->AddObstacleArea(worldCoords, Vec2(50.0f, 50.0f), true);
			}
		}

		void CMD_AddSimulationArea::Undo()
		{
			Simulation::Simulator* sim = m_Application->GetSimulator();
			sim->RemoveArea(m_AreaType, m_ID);
		}

		void CMD_TransformSimulationArea::Execute()
		{
			m_Area->Translate(m_TranslationDelta);
			m_Area->Scale(m_ScaleDelta);
		}

		void CMD_TransformSimulationArea::Undo()
		{
			m_Area->Translate(m_TranslationDelta * -1.0f);
			m_Area->Scale(m_ScaleDelta * -1.0f);
		}
	}

}