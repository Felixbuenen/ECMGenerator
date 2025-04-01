#include "Command.h"

#include "Application.h"
#include "EnvironmentEditor.h"
#include "Simulator.h"

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

		CMD_RemoveSimulationArea::CMD_RemoveSimulationArea(Simulation::SimAreaType areaType, int ID, Application* application)
			: m_AreaType(areaType), m_ID(ID), m_Application(application)
		{
			if (areaType == Simulation::SPAWN)
			{
				Simulation::SpawnArea* sa = application->GetSimulator()->GetSpawnArea(ID);

				m_Position = sa->Position;
				m_HalfSize.x = sa->HalfWidth;
				m_HalfSize.y = sa->HalfHeight;
			}
			else if (areaType == Simulation::GOAL)
			{
				Simulation::GoalArea* ga = application->GetSimulator()->GetGoalArea(ID);

				m_Position = ga->Position;
				m_HalfSize.x = ga->HalfWidth;
				m_HalfSize.y = ga->HalfHeight;
			}

		}

		void CMD_RemoveSimulationArea::Execute()
		{
			// first remove connections
			Simulation::Simulator* sim = m_Application->GetSimulator();
			m_ConnectedAreas = sim->GetConnectedAreas(m_ID, m_AreaType);
			if (m_AreaType == Simulation::SPAWN)
			{
				for (int gaID : m_ConnectedAreas)
				{
					sim->DeconnectSpawnGoalAreas(m_ID, gaID);
				}
			}
			else if (m_AreaType == Simulation::GOAL)
			{
				for (int saID : m_ConnectedAreas)
				{
					sim->DeconnectSpawnGoalAreas(saID, m_ID);
				}
			}

			// then remove area
			m_Application->GetSimulator()->RemoveArea(m_AreaType, m_ID);
		}

		void CMD_RemoveSimulationArea::Undo()
		{
			// first restore area
			if (m_AreaType == Simulation::SPAWN)
			{
				m_Application->GetSimulator()->AddSpawnArea(m_Position, m_HalfSize, Simulation::SpawnConfiguration(), m_ID);
			}
			else if (m_AreaType == Simulation::GOAL)
			{
				m_Application->GetSimulator()->AddGoalArea(m_Position, m_HalfSize, m_ID);
			}

			// then add connections
			if (m_AreaType == Simulation::SPAWN)
			{
				for (int id : m_ConnectedAreas)
				{
					m_Application->GetSimulator()->ConnectSpawnGoalAreas(m_ID, id, 0.5f);
				}
			}
			else if (m_AreaType == Simulation::GOAL)
			{
				for (int id : m_ConnectedAreas)
				{
					m_Application->GetSimulator()->ConnectSpawnGoalAreas(id, m_ID, 0.5f);
				}
			}
		}

		void CMD_AddSimulationAreaConnection::Execute()
		{
			m_Application->GetSimulator()->ConnectSpawnGoalAreas(m_SpawnID, m_GoalID, m_SpawnRate);
		}

		void CMD_AddSimulationAreaConnection::Undo()
		{
			m_Application->GetSimulator()->DeconnectSpawnGoalAreas(m_SpawnID, m_GoalID);
		}

		void CMD_RemoveSimulationAreaConnection::Execute()
		{
			m_Application->GetSimulator()->DeconnectSpawnGoalAreas(m_SpawnID, m_GoalID);
		}

		void CMD_RemoveSimulationAreaConnection::Undo()
		{
			m_Application->GetSimulator()->ConnectSpawnGoalAreas(m_SpawnID, m_GoalID, m_SpawnRate);
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